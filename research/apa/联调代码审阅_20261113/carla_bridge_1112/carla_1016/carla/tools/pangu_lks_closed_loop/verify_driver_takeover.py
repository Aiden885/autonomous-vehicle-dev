#!/usr/bin/env python3
"""Verify LKS driver takeover from ZMQ input through CARLA chassis feedback."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

import zmq


COMMAND_TOPIC = "gaasd.carla.driver_state_cmd.v1"
DRIVER_TOPIC = "gaasd.carla.driver_state.v1"
CHASSIS_TOPIC = "gaasd.carla.chassis_feedback.v1"
STATUS_TOPIC = "gaasd.carla.bridge_status.v1"


def make_command(brake: bool, steer: float) -> List[bytes]:
    message = {
        "header": {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": COMMAND_TOPIC,
            "timestamp_unix_ms": int(time.time() * 1000),
            "source": "lks_takeover_verification",
        },
        "payload": {
            "brake_pressed": brake,
            "driver_steer_norm": max(-1.0, min(1.0, steer)),
        },
    }
    return [
        COMMAND_TOPIC.encode("utf-8"),
        json.dumps(message, separators=(",", ":")).encode("utf-8"),
    ]


def median(values: List[float]) -> float:
    return statistics.median(values) if values else 0.0


class TakeoverVerifier:
    def __init__(self, publish_endpoint: str, control_endpoint: str) -> None:
        context = zmq.Context.instance()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.setsockopt(zmq.LINGER, 0)
        self.publisher.connect(control_endpoint)
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.LINGER, 0)
        for topic in (DRIVER_TOPIC, CHASSIS_TOPIC, STATUS_TOPIC):
            self.subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.subscriber.connect(publish_endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)
        self.latest_status_count = 0

    def close(self) -> None:
        self.publisher.close(0)
        self.subscriber.close(0)

    def run_stage(
        self, name: str, brake: bool, steer: float, duration_sec: float
    ) -> Dict[str, Any]:
        start = time.monotonic()
        next_send = start
        chassis: List[Dict[str, Any]] = []
        driver: List[Dict[str, Any]] = []
        while time.monotonic() - start < duration_sec:
            now = time.monotonic()
            if now >= next_send:
                self.publisher.send_multipart(make_command(brake, steer))
                next_send = now + 0.05
            events = dict(self.poller.poll(20))
            if self.subscriber not in events:
                continue
            topic_bytes, payload_bytes = self.subscriber.recv_multipart()
            try:
                topic = topic_bytes.decode("utf-8")
                message = json.loads(payload_bytes.decode("utf-8"))
                payload = message.get("payload", {})
            except (UnicodeDecodeError, ValueError, TypeError):
                continue
            elapsed = time.monotonic() - start
            # Ignore the transition interval and evaluate the settled response.
            if elapsed >= min(0.6, duration_sec * 0.4):
                if topic == CHASSIS_TOPIC:
                    chassis.append(payload)
                elif topic == DRIVER_TOPIC:
                    driver.append(payload)
            if topic == STATUS_TOPIC:
                counts = payload.get("publish_counts", {})
                self.latest_status_count = int(
                    counts.get("driver_state_cmd_received", self.latest_status_count)
                )

        steer_feedback = [float(item.get("steer_norm", 0.0)) for item in chassis]
        brake_feedback = [float(item.get("brake", 0.0)) for item in chassis]
        speed_feedback = [float(item.get("speed_mps", 0.0)) for item in chassis]
        driver_steer = [float(item.get("driver_steer_norm", 0.0)) for item in driver]
        driver_brake = [bool(item.get("brake_pressed", False)) for item in driver]
        return {
            "name": name,
            "requested": {"brake_pressed": brake, "driver_steer_norm": steer},
            "sample_count": {"chassis": len(chassis), "driver_state": len(driver)},
            "median_steer_norm": median(steer_feedback),
            "max_brake": max(brake_feedback) if brake_feedback else 0.0,
            "start_speed_mps": speed_feedback[0] if speed_feedback else 0.0,
            "end_speed_mps": speed_feedback[-1] if speed_feedback else 0.0,
            "median_driver_steer_norm": median(driver_steer),
            "driver_brake_ratio": (
                sum(1 for value in driver_brake if value) / len(driver_brake)
                if driver_brake
                else 0.0
            ),
            "bridge_received_count": self.latest_status_count,
        }


def evaluate(stages: Dict[str, Dict[str, Any]]) -> Tuple[bool, List[str]]:
    failures: List[str] = []
    for name, stage in stages.items():
        if stage["sample_count"]["chassis"] == 0 or stage["sample_count"]["driver_state"] == 0:
            failures.append(f"{name}: missing Bridge samples")
    if stages["left"]["median_driver_steer_norm"] > -0.25:
        failures.append("left: driver_state did not preserve negative takeover input")
    if stages["left"]["median_steer_norm"] > -0.20:
        failures.append("left: CARLA chassis did not steer left")
    if stages["right"]["median_driver_steer_norm"] < 0.25:
        failures.append("right: driver_state did not preserve positive takeover input")
    if stages["right"]["median_steer_norm"] < 0.20:
        failures.append("right: CARLA chassis did not steer right")
    if stages["brake"]["driver_brake_ratio"] < 0.8:
        failures.append("brake: driver_state did not preserve brake input")
    if stages["brake"]["max_brake"] < 0.8:
        failures.append("brake: CARLA chassis did not apply braking")
    if abs(stages["final_release"]["median_driver_steer_norm"]) > 0.02:
        failures.append("release: driver steering input did not reset")
    if stages["final_release"]["driver_brake_ratio"] > 0.1:
        failures.append("release: brake input did not reset")
    return not failures, failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--publish-endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--control-endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--output", default="/tmp/lks2-pangu-carla/results/takeover_latest.json")
    args = parser.parse_args()

    verifier = TakeoverVerifier(args.publish_endpoint, args.control_endpoint)
    time.sleep(0.4)
    try:
        ordered_stages = [
            ("initial_release", False, 0.0, 1.2),
            ("left", False, -0.3, 1.8),
            ("middle_release", False, 0.0, 1.2),
            ("right", False, 0.3, 1.8),
            ("pre_brake_release", False, 0.0, 1.2),
            ("brake", True, 0.0, 1.5),
            ("final_release", False, 0.0, 1.2),
        ]
        stages = {
            name: verifier.run_stage(name, brake, steer, duration)
            for name, brake, steer, duration in ordered_stages
        }
    finally:
        verifier.close()

    passed, failures = evaluate(stages)
    result = {"passed": passed, "failures": failures, "stages": stages}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
