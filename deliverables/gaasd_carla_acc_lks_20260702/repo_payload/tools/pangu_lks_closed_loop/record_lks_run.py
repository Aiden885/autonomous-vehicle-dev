#!/usr/bin/env python3.8
"""Record one LKS CARLA run and produce machine-readable curve metrics."""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import zmq


LANE_TOPIC = "gaasd.carla.lane_tracking.v1"
EGO_TOPIC = "gaasd.carla.ego_state.v1"
CHASSIS_TOPIC = "gaasd.carla.chassis_feedback.v1"
DRIVER_TOPIC = "gaasd.carla.driver_state.v1"


def add_carla_paths(root: str) -> None:
    paths = [os.path.join(root, "PythonAPI", "carla")]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def nested(data: Dict[str, Any], *keys: str, default: Any = 0.0) -> Any:
    node: Any = data
    for key in keys:
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return node


class Recorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.running = True
        self.start_wall = time.time()
        self.latest: Dict[str, Dict[str, Any]] = {}
        self.rows: List[Dict[str, Any]] = []
        self.collisions: List[Dict[str, Any]] = []
        self.collision_sensor: Optional[Any] = None

    def stop(self, *_args: Any) -> None:
        self.running = False

    def attach_collision_sensor(self) -> None:
        try:
            add_carla_paths(self.args.carla_root)
            import carla  # type: ignore

            client = carla.Client(self.args.host, self.args.port)
            client.set_timeout(5.0)
            world = client.get_world()
            ego = None
            deadline = time.monotonic() + 15.0
            while ego is None and time.monotonic() < deadline:
                for actor in world.get_actors().filter("vehicle.*"):
                    if actor.attributes.get("role_name") == self.args.ego_role_name:
                        ego = actor
                        break
                if ego is None:
                    time.sleep(0.1)
            if ego is None:
                print("[LKS recorder] collision sensor skipped: ego not found", file=sys.stderr)
                return
            blueprint = world.get_blueprint_library().find("sensor.other.collision")
            self.collision_sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=ego)
            self.collision_sensor.listen(self.on_collision)
        except Exception as exc:
            print(f"[LKS recorder] collision sensor skipped: {exc}", file=sys.stderr)

    def on_collision(self, event: Any) -> None:
        impulse = event.normal_impulse
        self.collisions.append(
            {
                "wall_time_sec": time.time() - self.start_wall,
                "frame": int(event.frame),
                "other_actor_id": int(event.other_actor.id),
                "other_actor_type": str(event.other_actor.type_id),
                "impulse": math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2),
            }
        )

    def add_message(self, topic: str, message: Dict[str, Any]) -> None:
        self.latest[topic] = message
        if topic != LANE_TOPIC:
            return
        header = message.get("header", {})
        lane = message.get("payload", {})
        ego = self.latest.get(EGO_TOPIC, {}).get("payload", {})
        chassis = self.latest.get(CHASSIS_TOPIC, {}).get("payload", {})
        driver = self.latest.get(DRIVER_TOPIC, {}).get("payload", {})
        pose = ego.get("pose", {})
        velocity = ego.get("velocity", {})
        self.rows.append(
            {
                "wall_time_sec": time.time() - self.start_wall,
                "sim_time_sec": float(header.get("sim_time_sec", 0.0)),
                "frame_id": int(header.get("frame_id", 0)),
                "speed_mps": float(velocity.get("speed_mps", chassis.get("speed_mps", 0.0))),
                "x_m": float(pose.get("x_m", 0.0)),
                "y_m": float(pose.get("y_m", 0.0)),
                "yaw_rad": float(pose.get("yaw_rad", 0.0)),
                "lane_valid": int(bool(lane.get("valid", False))),
                "c0_m": float(lane.get("c0_m", 0.0)),
                "c1": float(lane.get("c1", 0.0)),
                "c2_per_m": float(lane.get("c2_per_m", 0.0)),
                "c3_per_m2": float(lane.get("c3_per_m2", 0.0)),
                "curvature_per_m": float(lane.get("curvature_per_m", 0.0)),
                "lateral_offset_m": float(lane.get("lateral_offset_m", 0.0)),
                "heading_error_rad": float(lane.get("heading_error_rad", 0.0)),
                "road_id": int(lane.get("road_id", 0)),
                "lane_id": int(lane.get("lane_id", 0)),
                "junction_id": int(lane.get("junction_id", 0)),
                "steer_norm": float(chassis.get("steer_norm", 0.0)),
                "steering_angle_rad": float(chassis.get("steering_angle_rad", 0.0)),
                "throttle": float(chassis.get("throttle", 0.0)),
                "brake": float(chassis.get("brake", 0.0)),
                "last_command_id": int(chassis.get("last_command_id", 0)),
                "driver_brake_pressed": int(bool(driver.get("brake_pressed", False))),
                "driver_steer_norm": float(driver.get("driver_steer_norm", 0.0)),
            }
        )

    def write_results(self) -> Dict[str, Any]:
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = output_dir / f"lks_run_{stamp}.csv"
        summary_path = output_dir / f"lks_run_{stamp}.json"
        latest_path = output_dir / "latest_summary.json"
        if self.rows:
            with csv_path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.rows)

        abs_offsets = [abs(row["lateral_offset_m"]) for row in self.rows]
        speeds = [row["speed_mps"] for row in self.rows]
        steer_angles = [abs(row["steering_angle_rad"]) for row in self.rows]
        curve_rows = [
            row for row in self.rows
            if abs(row["curvature_per_m"]) >= self.args.curve_threshold
        ]
        valid_count = sum(row["lane_valid"] for row in self.rows)
        road_ids = sorted({row["road_id"] for row in self.rows})
        junction_ids = sorted({row["junction_id"] for row in self.rows if row["junction_id"] > 0})
        collision_episodes = 0
        last_collision_time: Optional[float] = None
        for collision in self.collisions:
            event_time = float(collision["wall_time_sec"])
            if last_collision_time is None or event_time - last_collision_time > 0.5:
                collision_episodes += 1
            last_collision_time = event_time
        summary = {
            "status": "complete" if self.rows else "no_data",
            "csv_path": str(csv_path) if self.rows else "",
            "sample_count": len(self.rows),
            "wall_duration_sec": time.time() - self.start_wall,
            "sim_duration_sec": (
                self.rows[-1]["sim_time_sec"] - self.rows[0]["sim_time_sec"]
                if len(self.rows) >= 2 else 0.0
            ),
            "max_speed_mps": max(speeds) if speeds else 0.0,
            "final_speed_mps": speeds[-1] if speeds else 0.0,
            "max_abs_lateral_offset_m": max(abs_offsets) if abs_offsets else 0.0,
            "rms_lateral_offset_m": (
                math.sqrt(sum(value * value for value in abs_offsets) / len(abs_offsets))
                if abs_offsets else 0.0
            ),
            "max_abs_steering_angle_rad": max(steer_angles) if steer_angles else 0.0,
            "lane_valid_ratio": valid_count / len(self.rows) if self.rows else 0.0,
            "curve_threshold_per_m": self.args.curve_threshold,
            "curve_sample_count": len(curve_rows),
            "curve_max_abs_lateral_offset_m": (
                max(abs(row["lateral_offset_m"]) for row in curve_rows) if curve_rows else None
            ),
            "curve_max_abs_curvature_per_m": (
                max(abs(row["curvature_per_m"]) for row in curve_rows) if curve_rows else 0.0
            ),
            "road_ids": road_ids,
            "junction_ids": junction_ids,
            "collision_count": collision_episodes,
            "collision_event_count": len(self.collisions),
            "collisions": self.collisions,
            "curve_covered": len(curve_rows) >= self.args.min_curve_samples,
        }
        summary["passed"] = bool(
            summary["curve_covered"]
            and summary["collision_count"] == 0
            and summary["lane_valid_ratio"] >= 0.99
            and summary["curve_max_abs_lateral_offset_m"] is not None
            and summary["curve_max_abs_lateral_offset_m"] <= self.args.max_curve_offset
        )
        text = json.dumps(summary, ensure_ascii=False, indent=2)
        summary_path.write_text(text + "\n", encoding="utf-8")
        latest_path.write_text(text + "\n", encoding="utf-8")
        print(text)
        return summary

    def run(self) -> int:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        self.attach_collision_sensor()
        context = zmq.Context.instance()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.LINGER, 0)
        for topic in (LANE_TOPIC, EGO_TOPIC, CHASSIS_TOPIC, DRIVER_TOPIC):
            socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        socket.connect(self.args.endpoint)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        deadline = time.monotonic() + self.args.duration_sec
        try:
            while self.running and time.monotonic() < deadline:
                events = dict(poller.poll(200))
                if socket not in events:
                    continue
                topic_bytes, payload_bytes = socket.recv_multipart()
                try:
                    self.add_message(
                        topic_bytes.decode("utf-8"),
                        json.loads(payload_bytes.decode("utf-8")),
                    )
                except (ValueError, TypeError, KeyError):
                    continue
        finally:
            if self.collision_sensor is not None:
                try:
                    self.collision_sensor.stop()
                    self.collision_sensor.destroy()
                except RuntimeError:
                    pass
            socket.close(0)
            self.write_results()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--duration-sec", type=float, default=180.0)
    parser.add_argument("--output-dir", default="/tmp/lks2-pangu-carla/results")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--curve-threshold", type=float, default=0.001)
    parser.add_argument("--min-curve-samples", type=int, default=20)
    parser.add_argument("--max-curve-offset", type=float, default=0.8)
    return Recorder(parser.parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
