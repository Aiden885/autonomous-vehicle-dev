#!/usr/bin/env python3
"""Minimal CARLA <-> GAASD bridge.

This is the first runnable bridge skeleton for integration work. It publishes
the core GAASD CARLA JSON topics and accepts a basic control_cmd.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import zmq
except ImportError as exc:  # pragma: no cover - environment guard
    raise SystemExit("pyzmq is required: python3 -m pip install pyzmq") from exc


PROTOCOL = "gaasd_carla_bridge"
PROTOCOL_VERSION = "0.3.0"
TOPIC_PREFIX = "gaasd.carla"


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def default_config() -> Dict[str, Any]:
    return {
        "carla": {
            "host": "127.0.0.1",
            "port": 2000,
            "timeout_sec": 5.0,
            "root": "/home/aiden/snap/code/app/carla-package",
            "python_api_paths": [],
            "sync_mode": True,
            "fixed_delta_seconds": 0.05,
            "realtime_pacing": True,
            "map_name": "",
            "ego_role_name": "hero",
            "auto_spawn_ego": True,
            "ego_blueprint_filter": "vehicle.*model3*",
            "ego_spawn_point_index": 0,
        },
        "zmq": {
            "publish_bind": "tcp://127.0.0.1:5701",
            "control_bind": "tcp://127.0.0.1:5702",
            "status_period_sec": 1.0,
        },
        "objects": {
            "max_objects": 64,
            "include_vehicles": True,
            "include_walkers": False,
        },
        "control": {
            "max_accel_mps2": 4.0,
            "max_brake_mps2": 8.0,
            "max_abs_steer_rad": 0.6,
            "speed_kp": 0.6,
            "speed_ki": 0.0,
            "speed_kd": 0.05,
            "speed_integral_limit": 2.0,
            "speed_deadband_mps": 0.03,
            "max_throttle": 1.0,
            "max_brake": 1.0,
            "timeout_sec": 0.2,
            "timeout_brake": 1.0,
            "prefer_speed_target": True,
            "lane_keep_enabled": False,
            "lane_keep_override_zero_steer": False,
            "lane_keep_zero_steer_epsilon_rad": 1.0e-6,
            "lane_keep_kp": 0.12,
            "lane_keep_ki": 0.0,
            "lane_keep_kd": 0.04,
            "lane_keep_heading_kp": 0.8,
            "lane_keep_integral_limit": 2.0,
            "lane_keep_max_steer": 0.25,
        },
    }


def add_carla_python_paths(cfg: Dict[str, Any]) -> None:
    carla_cfg = cfg["carla"]
    paths: List[str] = list(carla_cfg.get("python_api_paths", []))
    root = carla_cfg.get("root", "")

    if root:
        paths.append(os.path.join(root, "PythonAPI", "carla"))
        egg_dir = os.path.join(root, "PythonAPI", "carla", "dist")
        if sys.version_info.major == 3:
            paths.extend(glob.glob(os.path.join(egg_dir, "carla-*-py3*.egg")))
        else:
            paths.extend(glob.glob(os.path.join(egg_dir, "carla-*-py2*.egg")))

    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def import_carla(cfg: Dict[str, Any]):
    carla_cfg = cfg["carla"]
    root = carla_cfg.get("root", "")
    py37_eggs = []
    if root:
        py37_eggs = glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3.7-*.egg"))

    if py37_eggs and sys.version_info >= (3, 9):
        version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
        raise SystemExit(
            "CARLA 0.9.15 in this package provides a py3.7 egg. "
            f"Current Python is {version}, which is not safe for this egg. "
            "Use python3.8, for example: tools/carla_bridge/run-bridge.sh"
        )

    add_carla_python_paths(cfg)
    try:
        import carla  # type: ignore
    except ImportError as exc:
        searched = "\n".join(sys.path[:20])
        raise SystemExit(
            "Cannot import CARLA Python API. Set carla.root or carla.python_api_paths "
            "in config.\nFirst sys.path entries:\n" + searched
        ) from exc
    return carla


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if math.isfinite(number):
        return number
    return default


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_enu_rad(carla_yaw_deg: float) -> float:
    return math.radians(-carla_yaw_deg)


def carla_xy_to_enu(x: float, y: float) -> Tuple[float, float]:
    return x, -y


def vector_to_enu(vec: Any) -> Tuple[float, float, float]:
    return finite(vec.x), -finite(vec.y), finite(vec.z)


def speed_2d(vec: Any) -> float:
    return math.hypot(finite(vec.x), finite(vec.y))


def angular_yaw_rate_to_enu_rps(vec: Any) -> float:
    return -math.radians(finite(vec.z))


def actor_length_width_height(actor: Any) -> Tuple[float, float, float]:
    bb = actor.bounding_box
    return 2.0 * finite(bb.extent.x), 2.0 * finite(bb.extent.y), 2.0 * finite(bb.extent.z)


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return 0.0
    if isinstance(value, dict):
        return {key: json_safe(val) for key, val in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


@dataclass
class LastCommand:
    timestamp: float = 0.0
    command_id: int = 0
    throttle: float = 0.0
    brake: float = 0.0
    steer_norm: float = 0.0


class CarlaGaasdBridge:
    def __init__(self, cfg: Dict[str, Any], dry_run: bool = False) -> None:
        self.cfg = cfg
        self.dry_run = dry_run
        self.running = True
        self.carla = None
        self.client = None
        self.world = None
        self.ego = None
        self.original_world_settings: Optional[Dict[str, Any]] = None
        self.sequence: Dict[str, int] = {}
        self.last_status_time = 0.0
        self.last_error = ""
        self.last_command = LastCommand()
        self.control_cmd_count = 0
        self.control_timeout_applied = False
        self.speed_pid_prev_error = 0.0
        self.speed_pid_integral_error = 0.0
        self.speed_pid_prev_time = 0.0
        self.lane_keep_prev_error = 0.0
        self.lane_keep_integral_error = 0.0
        self.lane_keep_prev_time = 0.0

        self.context = zmq.Context.instance()
        self.pub = self.context.socket(zmq.PUB)
        self.sub = self.context.socket(zmq.SUB)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

    def setup_zmq(self) -> None:
        zmq_cfg = self.cfg["zmq"]
        self.pub.bind(zmq_cfg["publish_bind"])
        self.sub.bind(zmq_cfg["control_bind"])
        print(f"[Bridge] PUB bind {zmq_cfg['publish_bind']}")
        print(f"[Bridge] SUB bind {zmq_cfg['control_bind']}")

    def connect_carla(self) -> None:
        if self.dry_run:
            print("[Bridge] dry-run mode, skip CARLA connection")
            return

        self.carla = import_carla(self.cfg)
        carla_cfg = self.cfg["carla"]
        self.client = self.carla.Client(carla_cfg["host"], int(carla_cfg["port"]))
        self.client.set_timeout(float(carla_cfg["timeout_sec"]))
        self.world = self.client.get_world()

        map_name = carla_cfg.get("map_name", "")
        if map_name:
            current = self.world.get_map().name
            if not current.endswith(map_name):
                self.world = self.client.load_world(map_name)

        if carla_cfg.get("sync_mode", True):
            settings = self.world.get_settings()
            self.original_world_settings = {
                "synchronous_mode": settings.synchronous_mode,
                "fixed_delta_seconds": settings.fixed_delta_seconds,
            }
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(carla_cfg.get("fixed_delta_seconds", 0.05))
            self.world.apply_settings(settings)

        self.ego = self.find_or_spawn_ego()

    def find_or_spawn_ego(self):
        role_name = self.cfg["carla"].get("ego_role_name", "hero")
        actors = self.world.get_actors().filter("vehicle.*")
        for actor in actors:
            if actor.attributes.get("role_name") == role_name:
                print(f"[Bridge] found ego id={actor.id} role_name={role_name}")
                return actor

        if not self.cfg["carla"].get("auto_spawn_ego", True):
            print("[Bridge] ego not found and auto_spawn_ego=false")
            return None

        blueprints = self.world.get_blueprint_library().filter(
            self.cfg["carla"].get("ego_blueprint_filter", "vehicle.*model3*")
        )
        if not blueprints:
            raise RuntimeError("no ego blueprint matched")

        blueprint = blueprints[0]
        blueprint.set_attribute("role_name", role_name)
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("no spawn points available")

        index = int(self.cfg["carla"].get("ego_spawn_point_index", 0)) % len(spawn_points)
        actor = self.world.try_spawn_actor(blueprint, spawn_points[index])
        if actor is None:
            raise RuntimeError(f"failed to spawn ego at spawn point {index}")

        print(f"[Bridge] spawned ego id={actor.id} role_name={role_name}")
        return actor

    def stop(self, *_args: Any) -> None:
        self.running = False

    def next_sequence(self, topic: str) -> int:
        current = self.sequence.get(topic, 0) + 1
        self.sequence[topic] = current
        return current

    def make_header(self, topic: str, snapshot: Any = None) -> Dict[str, Any]:
        now_ms = int(time.time() * 1000)
        frame = 0
        sim_time = 0.0
        dt = float(self.cfg["carla"].get("fixed_delta_seconds", 0.05))
        map_name = ""

        if snapshot is not None:
            frame = int(getattr(snapshot, "frame", 0))
            timestamp = getattr(snapshot, "timestamp", None)
            if timestamp is not None:
                sim_time = finite(getattr(timestamp, "elapsed_seconds", 0.0))
                dt = finite(getattr(timestamp, "delta_seconds", dt), dt)

        if self.world is not None:
            try:
                map_name = self.world.get_map().name
            except RuntimeError:
                map_name = ""

        return {
            "protocol": PROTOCOL,
            "protocol_version": PROTOCOL_VERSION,
            "message_type": topic,
            "frame_id": frame,
            "sequence": self.next_sequence(topic),
            "sim_time_sec": sim_time,
            "delta_time_sec": dt,
            "timestamp_unix_ms": now_ms,
            "source": "carla_bridge",
            "map_name": map_name,
            "ego_role_name": self.cfg["carla"].get("ego_role_name", "hero"),
            "coordinate_frame": "gaasd_map",
        }

    def publish(self, short_topic: str, payload: Dict[str, Any], snapshot: Any = None) -> None:
        topic = f"{TOPIC_PREFIX}.{short_topic}.v1"
        envelope = {
            "header": self.make_header(topic, snapshot),
            "payload": json_safe(payload),
        }
        data = json.dumps(envelope, ensure_ascii=False, separators=(",", ":"))
        self.pub.send_multipart([topic.encode("utf-8"), data.encode("utf-8")])

    def run(self) -> None:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)

        self.setup_zmq()
        self.connect_carla()

        try:
            while self.running:
                try:
                    if self.dry_run:
                        self.publish_bridge_status(None)
                        time.sleep(1.0)
                        continue

                    loop_started = time.monotonic()
                    self.process_control_messages()
                    self.apply_control_timeout_if_needed()
                    snapshot = self.tick_world()
                    self.publish_all(snapshot)
                    self.pace_loop(loop_started)
                except Exception as exc:  # keep bridge alive and visible
                    self.last_error = f"{exc.__class__.__name__}: {exc}"
                    print(f"[Bridge] error: {self.last_error}", file=sys.stderr)
                    self.publish_bridge_status(None)
                    time.sleep(0.2)
        finally:
            self.restore_carla_settings()
            self.close_zmq()

    def restore_carla_settings(self) -> None:
        if self.world is None or self.original_world_settings is None:
            return
        try:
            settings = self.world.get_settings()
            settings.synchronous_mode = bool(self.original_world_settings["synchronous_mode"])
            settings.fixed_delta_seconds = self.original_world_settings["fixed_delta_seconds"]
            self.world.apply_settings(settings)
            print("[Bridge] restored CARLA world settings")
        except Exception as exc:
            print(f"[Bridge] restore settings failed: {exc}", file=sys.stderr)

    def close_zmq(self) -> None:
        self.pub.close(0)
        self.sub.close(0)

    def tick_world(self):
        if self.cfg["carla"].get("sync_mode", True):
            self.world.tick()
            return self.world.get_snapshot()
        return self.world.wait_for_tick()

    def pace_loop(self, loop_started: float) -> None:
        carla_cfg = self.cfg["carla"]
        if not carla_cfg.get("sync_mode", True):
            return
        if not carla_cfg.get("realtime_pacing", True):
            return
        fixed_delta = finite(carla_cfg.get("fixed_delta_seconds", 0.05), 0.05)
        elapsed = time.monotonic() - loop_started
        remaining = fixed_delta - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

    def publish_all(self, snapshot: Any) -> None:
        self.publish_sim_clock(snapshot)
        self.refresh_ego()
        if self.ego is not None:
            ego = self.ego_payload()
            objects = self.collect_objects()
            enriched = self.enrich_objects(ego, objects)
            self.publish_ego_state(ego, snapshot)
            self.publish_object_list(enriched, snapshot)
            self.publish_lead_vehicle(ego, enriched, snapshot)
            self.publish_lane_tracking(self.lane_tracking_payload(), snapshot)
            self.publish_chassis_feedback(snapshot)
        self.publish_bridge_status(snapshot)

    def refresh_ego(self) -> None:
        if self.ego is not None and self.ego.is_alive:
            return
        self.ego = self.find_or_spawn_ego()

    def publish_sim_clock(self, snapshot: Any) -> None:
        settings = self.world.get_settings()
        payload = {
            "synchronous_mode": bool(settings.synchronous_mode),
            "fixed_delta_seconds": finite(settings.fixed_delta_seconds, 0.0),
            "paused": False,
            "real_time_factor": 0.0,
            "carla_server_version": "0.9.15",
        }
        self.publish("sim_clock", payload, snapshot)

    def ego_payload(self) -> Dict[str, Any]:
        transform = self.ego.get_transform()
        velocity = self.ego.get_velocity()
        acceleration = self.ego.get_acceleration()
        angular = self.ego.get_angular_velocity()
        yaw_rad = yaw_to_enu_rad(finite(transform.rotation.yaw))
        x_m, y_m = carla_xy_to_enu(finite(transform.location.x), finite(transform.location.y))
        vx, vy, vz = vector_to_enu(velocity)
        ax, ay, az = vector_to_enu(acceleration)
        longitudinal_acc = ax * math.cos(yaw_rad) + ay * math.sin(yaw_rad)

        road = {"lane_id": 0, "road_id": 0, "junction_id": 0, "s_m": 0.0}
        try:
            waypoint = self.world.get_map().get_waypoint(transform.location)
            road = {
                "lane_id": int(waypoint.lane_id),
                "road_id": int(waypoint.road_id),
                "junction_id": int(waypoint.junction_id),
                "s_m": finite(waypoint.s),
            }
        except RuntimeError:
            pass

        length, width, _height = actor_length_width_height(self.ego)
        return {
            "ego_id": int(self.ego.id),
            "role_name": self.cfg["carla"].get("ego_role_name", "hero"),
            "pose": {
                "x_m": x_m,
                "y_m": y_m,
                "z_m": finite(transform.location.z),
                "yaw_rad": yaw_rad,
                "pitch_rad": math.radians(finite(transform.rotation.pitch)),
                "roll_rad": math.radians(finite(transform.rotation.roll)),
                "heading_deg": (90.0 - math.degrees(yaw_rad)) % 360.0,
            },
            "velocity": {
                "vx_mps": vx,
                "vy_mps": vy,
                "vz_mps": vz,
                "speed_mps": speed_2d(velocity),
                "longitudinal_mps": vx * math.cos(yaw_rad) + vy * math.sin(yaw_rad),
                "lateral_mps": -vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad),
            },
            "acceleration": {
                "ax_mps2": ax,
                "ay_mps2": ay,
                "az_mps2": az,
                "longitudinal_mps2": longitudinal_acc,
                "lateral_mps2": -ax * math.sin(yaw_rad) + ay * math.cos(yaw_rad),
            },
            "angular_velocity": {
                "yaw_rate_rps": angular_yaw_rate_to_enu_rps(angular),
                "pitch_rate_rps": math.radians(finite(angular.y)),
                "roll_rate_rps": math.radians(finite(angular.x)),
            },
            "road": road,
            "vehicle": {
                "length_m": length,
                "width_m": width,
                "wheel_base_m": 0.0,
            },
            "debug": {
                "carla_raw": {
                    "x": finite(transform.location.x),
                    "y": finite(transform.location.y),
                    "z": finite(transform.location.z),
                    "yaw_deg": finite(transform.rotation.yaw),
                }
            },
        }

    def publish_ego_state(self, ego: Dict[str, Any], snapshot: Any) -> None:
        self.publish("ego_state", ego, snapshot)

    def collect_objects(self) -> List[Dict[str, Any]]:
        obj_cfg = self.cfg["objects"]
        actors: List[Any] = []
        world_actors = self.world.get_actors()
        if obj_cfg.get("include_vehicles", True):
            actors.extend(list(world_actors.filter("vehicle.*")))
        if obj_cfg.get("include_walkers", False):
            actors.extend(list(world_actors.filter("walker.pedestrian.*")))

        result = []
        max_objects = int(obj_cfg.get("max_objects", 64))
        ego_id = self.ego.id if self.ego is not None else -1
        for actor in actors:
            if actor.id == ego_id:
                continue
            result.append(self.object_payload(actor))
            if len(result) >= max_objects:
                break
        return result

    def object_payload(self, actor: Any) -> Dict[str, Any]:
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        yaw_rad = yaw_to_enu_rad(finite(transform.rotation.yaw))
        x_m, y_m = carla_xy_to_enu(finite(transform.location.x), finite(transform.location.y))
        vx, vy, _vz = vector_to_enu(velocity)
        length, width, height = actor_length_width_height(actor)
        type_id = actor.type_id
        role_name = actor.attributes.get("role_name", "")
        type_code = 1 if type_id.startswith("vehicle.") else 2 if type_id.startswith("walker.") else 0
        return {
            "object_id": int(actor.id),
            "role_name": role_name,
            "type": "vehicle" if type_code == 1 else "pedestrian" if type_code == 2 else "unknown",
            "type_code": type_code,
            "subtype": type_id,
            "pose": {
                "x_m": x_m,
                "y_m": y_m,
                "z_m": finite(transform.location.z),
                "yaw_rad": yaw_rad,
                "heading_deg": (90.0 - math.degrees(yaw_rad)) % 360.0,
            },
            "velocity": {
                "vx_mps": vx,
                "vy_mps": vy,
                "speed_mps": speed_2d(velocity),
            },
            "acceleration": {"longitudinal_mps2": 0.0},
            "dimension": {
                "length_m": length,
                "width_m": width,
                "height_m": height,
            },
            "debug": {"carla_actor_id": int(actor.id)},
        }

    def enrich_objects(self, ego: Dict[str, Any], objects: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        enriched = []
        for obj in objects:
            rel = self.relative_to_ego(ego, obj)
            item = dict(obj)
            item["relative_to_ego"] = rel
            enriched.append(item)
        return enriched

    def publish_object_list(self, objects: List[Dict[str, Any]], snapshot: Any) -> None:
        self.publish(
            "object_list",
            {"source_type": "ground_truth", "object_count": len(objects), "objects": objects},
            snapshot,
        )

    def relative_to_ego(self, ego: Dict[str, Any], obj: Dict[str, Any]) -> Dict[str, float]:
        yaw = finite(ego["pose"]["yaw_rad"])
        dx = finite(obj["pose"]["x_m"]) - finite(ego["pose"]["x_m"])
        dy = finite(obj["pose"]["y_m"]) - finite(ego["pose"]["y_m"])
        longitudinal = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        ego_half = finite(ego["vehicle"]["length_m"]) * 0.5
        obj_half = finite(obj["dimension"]["length_m"]) * 0.5
        clearance = longitudinal - ego_half - obj_half
        obj_longitudinal_speed = (
            finite(obj["velocity"]["vx_mps"]) * math.cos(yaw)
            + finite(obj["velocity"]["vy_mps"]) * math.sin(yaw)
        )
        rel_speed = obj_longitudinal_speed - finite(ego["velocity"]["longitudinal_mps"])
        ttc = 1.0e6
        if rel_speed < -1.0e-3:
            ttc = max(0.0, clearance) / max(-rel_speed, 1.0e-6)
        return {
            "longitudinal_distance_m": longitudinal,
            "lateral_distance_m": lateral,
            "clearance_m": clearance,
            "relative_speed_mps": rel_speed,
            "ttc_sec": ttc,
        }

    def publish_lead_vehicle(self, ego: Dict[str, Any], objects: List[Dict[str, Any]], snapshot: Any) -> None:
        lead = None
        lead_rel = None
        lead_rule = "same_lane_nearest_front"
        obj_cfg = self.cfg.get("objects", {})
        max_lateral = float(obj_cfg.get("lead_max_lateral_m", 3.5))
        preferred_role = str(obj_cfg.get("preferred_lead_role_name", ""))
        preferred_max_lateral = float(obj_cfg.get("preferred_lead_max_lateral_m", max_lateral))
        lead_key = None
        for obj in objects:
            rel = obj["relative_to_ego"]
            if rel["longitudinal_distance_m"] <= 0.0:
                continue
            is_preferred = bool(preferred_role) and obj.get("role_name") == preferred_role
            lateral_limit = preferred_max_lateral if is_preferred else max_lateral
            if abs(rel["lateral_distance_m"]) > lateral_limit:
                continue
            candidate_key = (0 if is_preferred else 1, rel["longitudinal_distance_m"])
            if lead_key is None or candidate_key < lead_key:
                lead = obj
                lead_rel = rel
                lead_key = candidate_key
                lead_rule = "preferred_role_nearest_front" if is_preferred else "same_lane_nearest_front"

        if lead is None or lead_rel is None:
            payload = {
                "valid": False,
                "object_id": 0,
                "selection_rule": lead_rule,
                "type": "unknown",
                "lead_speed_mps": 0.0,
                "ego_speed_mps": finite(ego["velocity"]["speed_mps"]),
                "relative_speed_mps": 0.0,
                "longitudinal_distance_m": 1.0e6,
                "clearance_m": 1.0e6,
                "lateral_distance_m": 0.0,
                "time_gap_sec": 1.0e6,
                "ttc_sec": 1.0e6,
            }
        else:
            ego_speed = finite(ego["velocity"]["speed_mps"])
            clearance = lead_rel["clearance_m"]
            payload = {
                "valid": True,
                "object_id": lead["object_id"],
                "selection_rule": lead_rule,
                "type": lead["type"],
                "lead_speed_mps": finite(lead["velocity"]["speed_mps"]),
                "ego_speed_mps": ego_speed,
                "relative_speed_mps": lead_rel["relative_speed_mps"],
                "longitudinal_distance_m": lead_rel["longitudinal_distance_m"],
                "clearance_m": clearance,
                "lateral_distance_m": lead_rel["lateral_distance_m"],
                "time_gap_sec": clearance / max(ego_speed, 1.0e-6),
                "ttc_sec": lead_rel["ttc_sec"],
            }
        self.publish("lead_vehicle", payload, snapshot)

    def publish_chassis_feedback(self, snapshot: Any) -> None:
        velocity = self.ego.get_velocity()
        control = self.ego.get_control()
        age = time.time() - self.last_command.timestamp if self.last_command.timestamp else 1.0e6
        payload = {
            "speed_mps": speed_2d(velocity),
            "steering_angle_rad": self.last_command.steer_norm * float(self.cfg["control"]["max_abs_steer_rad"]),
            "steer_norm": finite(getattr(control, "steer", 0.0)),
            "throttle": finite(getattr(control, "throttle", 0.0)),
            "brake": finite(getattr(control, "brake", 0.0)),
            "mode": 1 if age <= float(self.cfg["control"]["timeout_sec"]) else 0,
            "gear": int(getattr(control, "gear", 0)),
            "last_command_id": self.last_command.command_id,
            "last_command_age_sec": age,
        }
        self.publish("chassis_feedback", payload, snapshot)

    def lane_tracking_payload(self) -> Dict[str, Any]:
        payload = {
            "valid": False,
            "lateral_offset_m": 0.0,
            "heading_error_rad": 0.0,
            "lane_id": 0,
            "road_id": 0,
            "junction_id": 0,
            "s_m": 0.0,
        }
        if self.ego is None or self.world is None or self.carla is None:
            return payload

        transform = self.ego.get_transform()
        waypoint = self.world.get_map().get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=self.carla.LaneType.Driving,
        )
        if waypoint is None:
            return payload

        lane_center = waypoint.transform.location
        lane_forward = waypoint.transform.get_forward_vector()
        to_center = self.carla.Vector3D(
            lane_center.x - transform.location.x,
            lane_center.y - transform.location.y,
            0.0,
        )
        right_direction = self.carla.Vector3D(-lane_forward.y, lane_forward.x, 0.0)
        right_norm = math.hypot(right_direction.x, right_direction.y)
        if right_norm <= 1.0e-6:
            return payload
        right_direction.x = right_direction.x / right_norm
        right_direction.y = right_direction.y / right_norm

        # Positive offset means the lane center is to ego's right.
        lateral_offset = finite(to_center.x * right_direction.x + to_center.y * right_direction.y)
        lane_yaw = finite(waypoint.transform.rotation.yaw)
        ego_yaw = finite(transform.rotation.yaw)
        heading_error_deg = ((lane_yaw - ego_yaw + 180.0) % 360.0) - 180.0
        payload.update(
            {
                "valid": True,
                "lateral_offset_m": lateral_offset,
                "heading_error_rad": math.radians(heading_error_deg),
                "lane_id": int(waypoint.lane_id),
                "road_id": int(waypoint.road_id),
                "junction_id": int(waypoint.junction_id),
                "s_m": finite(waypoint.s),
            }
        )
        return payload

    def publish_lane_tracking(self, payload: Dict[str, Any], snapshot: Any) -> None:
        self.publish("lane_tracking", payload, snapshot)

    def publish_bridge_status(self, snapshot: Any) -> None:
        now = time.time()
        period = float(self.cfg["zmq"].get("status_period_sec", 1.0))
        if snapshot is not None and now - self.last_status_time < period:
            return
        self.last_status_time = now
        payload = {
            "state": "running" if not self.dry_run else "starting",
            "connected": self.world is not None,
            "carla_launch_mode": "external",
            "carla_host": self.cfg["carla"]["host"],
            "carla_port": int(self.cfg["carla"]["port"]),
            "ssh_host": "",
            "map_name": self.current_map_name(),
            "ego_spawned": self.ego is not None,
            "last_error": self.last_error,
            "publish_counts": {
                "ego_state": self.sequence.get(f"{TOPIC_PREFIX}.ego_state.v1", 0),
                "object_list": self.sequence.get(f"{TOPIC_PREFIX}.object_list.v1", 0),
                "control_cmd_received": self.control_cmd_count,
            },
        }
        self.publish("bridge_status", payload, snapshot)

    def current_map_name(self) -> str:
        if self.world is None:
            return ""
        try:
            return self.world.get_map().name
        except RuntimeError:
            return ""

    def process_control_messages(self) -> None:
        while True:
            try:
                _topic, payload = self.sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            try:
                msg = json.loads(payload.decode("utf-8"))
                self.control_cmd_count = self.control_cmd_count + 1
                self.apply_control_message(msg)
            except Exception as exc:
                self.last_error = f"control parse error: {exc}"

    def apply_control_message(self, msg: Dict[str, Any]) -> None:
        if self.ego is None:
            return
        payload = msg.get("payload", msg)
        target = payload.get("target", payload)
        safety = payload.get("safety", {})
        enable = bool(payload.get("enable", True))
        command_id = int(payload.get("command_id", self.last_command.command_id + 1))
        ctrl_cfg = self.cfg["control"]

        max_steer = finite(safety.get("max_abs_steer_rad"), float(ctrl_cfg["max_abs_steer_rad"]))
        has_explicit_steer = "steer_rad" in target or "steer" in target
        steer_rad = finite(target.get("steer_rad", target.get("steer", 0.0)))
        zero_steer_epsilon = finite(ctrl_cfg.get("lane_keep_zero_steer_epsilon_rad", 1.0e-6))
        can_override_zero_steer = (
            bool(ctrl_cfg.get("lane_keep_enabled", False))
            and bool(ctrl_cfg.get("lane_keep_override_zero_steer", False))
            and abs(steer_rad) <= max(zero_steer_epsilon, 0.0)
        )
        if not has_explicit_steer or can_override_zero_steer:
            steer_norm = self.compute_lane_keep_steer_norm()
        else:
            steer_norm = clamp(steer_rad / max(max_steer, 1.0e-6), -1.0, 1.0)

        if not enable:
            throttle, brake, steer_norm = 0.0, 1.0, 0.0
            self.reset_speed_pid()
        else:
            throttle, brake = self.longitudinal_to_throttle_brake(target, safety)

        control = self.carla.VehicleControl(throttle=throttle, brake=brake, steer=steer_norm)
        self.ego.apply_control(control)
        self.last_command = LastCommand(time.time(), command_id, throttle, brake, steer_norm)
        self.control_timeout_applied = False

    def compute_lane_keep_steer_norm(self) -> float:
        ctrl_cfg = self.cfg["control"]
        if not bool(ctrl_cfg.get("lane_keep_enabled", False)):
            return 0.0
        lane_tracking = self.lane_tracking_payload()
        if not bool(lane_tracking["valid"]):
            return 0.0
        offset_error = finite(lane_tracking["lateral_offset_m"])
        heading_error = finite(lane_tracking["heading_error_rad"])

        now = time.monotonic()
        dt = now - self.lane_keep_prev_time if self.lane_keep_prev_time > 0.0 else 0.05
        if dt <= 1.0e-6 or dt > 1.0:
            dt = 0.05
        derivative = (offset_error - self.lane_keep_prev_error) / dt
        integral_limit = max(finite(ctrl_cfg.get("lane_keep_integral_limit", 2.0)), 0.0)
        self.lane_keep_integral_error = clamp(
            self.lane_keep_integral_error + offset_error * dt,
            -integral_limit,
            integral_limit,
        )
        self.lane_keep_prev_error = offset_error
        self.lane_keep_prev_time = now

        steer = (
            finite(ctrl_cfg.get("lane_keep_kp", 0.12)) * offset_error
            + finite(ctrl_cfg.get("lane_keep_ki", 0.0)) * self.lane_keep_integral_error
            + finite(ctrl_cfg.get("lane_keep_kd", 0.04)) * derivative
            + finite(ctrl_cfg.get("lane_keep_heading_kp", 0.8)) * heading_error
        )
        max_steer = clamp(finite(ctrl_cfg.get("lane_keep_max_steer", 0.25)), 0.0, 1.0)
        return clamp(steer, -max_steer, max_steer)

    def apply_control_timeout_if_needed(self) -> None:
        if self.ego is None or self.carla is None:
            return
        if not self.last_command.timestamp:
            return
        if self.control_timeout_applied:
            return
        age = time.time() - self.last_command.timestamp
        timeout_sec = float(self.cfg["control"]["timeout_sec"])
        if age <= timeout_sec:
            return
        timeout_brake = clamp(float(self.cfg["control"].get("timeout_brake", 1.0)), 0.0, 1.0)
        control = self.carla.VehicleControl(throttle=0.0, brake=timeout_brake, steer=0.0)
        self.ego.apply_control(control)
        self.reset_speed_pid()
        self.control_timeout_applied = True

    def reset_speed_pid(self) -> None:
        self.speed_pid_prev_error = 0.0
        self.speed_pid_integral_error = 0.0
        self.speed_pid_prev_time = 0.0

    def speed_pid_accel(self, target_speed: float, current_speed: float) -> float:
        ctrl_cfg = self.cfg["control"]
        error = target_speed - current_speed
        deadband = max(finite(ctrl_cfg.get("speed_deadband_mps", 0.03)), 0.0)
        if abs(error) <= deadband:
            error = 0.0

        now = time.monotonic()
        dt = now - self.speed_pid_prev_time if self.speed_pid_prev_time > 0.0 else 0.05
        if dt <= 1.0e-6 or dt > 1.0:
            dt = 0.05

        integral_limit = max(finite(ctrl_cfg.get("speed_integral_limit", 2.0)), 0.0)
        self.speed_pid_integral_error = clamp(
            self.speed_pid_integral_error + error * dt,
            -integral_limit,
            integral_limit,
        )
        derivative = (error - self.speed_pid_prev_error) / dt

        self.speed_pid_prev_error = error
        self.speed_pid_prev_time = now

        return (
            finite(ctrl_cfg.get("speed_kp", 0.6)) * error
            + finite(ctrl_cfg.get("speed_ki", 0.0)) * self.speed_pid_integral_error
            + finite(ctrl_cfg.get("speed_kd", 0.05)) * derivative
        )

    def longitudinal_to_throttle_brake(self, target: Dict[str, Any], safety: Dict[str, Any]) -> Tuple[float, float]:
        ctrl_cfg = self.cfg["control"]
        prefer_speed = bool(ctrl_cfg.get("prefer_speed_target", True))
        has_speed = "target_speed_mps" in target or "speed" in target
        has_accel = "target_accel_mps2" in target or "acceleration" in target

        if prefer_speed and has_speed:
            target_speed = finite(target.get("target_speed_mps", target.get("speed", 0.0)))
            max_speed = finite(safety.get("max_speed_mps"), target_speed)
            target_speed = clamp(target_speed, 0.0, max(max_speed, 0.0))
            current_speed = speed_2d(self.ego.get_velocity())
            accel = self.speed_pid_accel(target_speed, current_speed)
        elif has_accel:
            self.reset_speed_pid()
            accel = finite(target.get("target_accel_mps2", target.get("acceleration", 0.0)))
        else:
            self.reset_speed_pid()
            accel = 0.0

        max_accel = finite(safety.get("max_accel_mps2"), float(ctrl_cfg["max_accel_mps2"]))
        max_brake = finite(safety.get("max_decel_mps2"), float(ctrl_cfg["max_brake_mps2"]))
        accel = clamp(accel, -max_brake, max_accel)
        max_throttle_norm = clamp(finite(ctrl_cfg.get("max_throttle", 1.0)), 0.0, 1.0)
        max_brake_norm = clamp(finite(ctrl_cfg.get("max_brake", 1.0)), 0.0, 1.0)

        if accel >= 0.0:
            return clamp(accel / max(max_accel, 1.0e-6), 0.0, max_throttle_norm), 0.0
        return 0.0, clamp(-accel / max(max_brake, 1.0e-6), 0.0, max_brake_norm)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CARLA to GAASD bridge")
    parser.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "config.example.json"))
    parser.add_argument("--dry-run", action="store_true", help="start ZMQ and status loop without CARLA")
    return parser.parse_args(argv)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    cfg = deep_merge(default_config(), load_json(args.config))
    bridge = CarlaGaasdBridge(cfg, dry_run=args.dry_run)
    bridge.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
