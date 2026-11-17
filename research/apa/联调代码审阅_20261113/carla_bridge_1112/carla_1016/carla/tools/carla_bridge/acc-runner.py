#!/usr/bin/env python3
"""Standalone ACC runner for phase-1 CARLA bridge closed-loop tests."""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any, Dict, Optional

import zmq


EGO_TOPIC = "gaasd.carla.ego_state.v1"
LEAD_TOPIC = "gaasd.carla.lead_vehicle.v1"
CONTROL_TOPIC = "gaasd.carla.control_cmd.v1"


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


def make_header(sequence: int) -> Dict[str, Any]:
    now_ms = int(time.time() * 1000.0)
    return {
        "protocol": "gaasd_carla_bridge",
        "protocol_version": "0.3.0",
        "message_type": CONTROL_TOPIC,
        "frame_id": "gaasd_map",
        "sequence": sequence,
        "sim_time_sec": 0.0,
        "delta_time_sec": 0.0,
        "timestamp_unix_ms": now_ms,
        "source": "acc_runner",
        "map_name": "",
        "ego_role_name": "hero",
        "coordinate_frame": "gaasd_map",
    }


def extract_payload(message: Dict[str, Any]) -> Dict[str, Any]:
    payload = message.get("payload", {})
    if isinstance(payload, dict):
        return payload
    return {}


def compute_target_speed(
    ego: Optional[Dict[str, Any]],
    lead: Optional[Dict[str, Any]],
    cruise_speed: float,
    min_distance: float,
    time_gap: float,
    distance_gain: float,
    relative_speed_gain: float,
) -> Dict[str, float]:
    ego_speed = 0.0
    if ego is not None:
        ego_speed = finite(ego.get("velocity", {}).get("speed_mps"), 0.0)

    lead_valid = False
    distance = 1.0e6
    relative_speed = 0.0
    lead_speed = 0.0
    if lead is not None:
        lead_valid = bool(lead.get("valid", False))
        distance = finite(lead.get("clearance_m"), 1.0e6)
        relative_speed = finite(lead.get("relative_speed_mps"), 0.0)
        lead_speed = finite(lead.get("lead_speed_mps"), 0.0)

    target_speed = cruise_speed
    desired_distance = min_distance + time_gap * ego_speed
    if lead_valid:
        distance_error = distance - desired_distance
        target_speed = lead_speed + distance_gain * distance_error + relative_speed_gain * relative_speed
        target_speed = min(target_speed, cruise_speed)

    target_speed = clamp(target_speed, 0.0, cruise_speed)
    return {
        "target_speed": target_speed,
        "ego_speed": ego_speed,
        "lead_valid": 1.0 if lead_valid else 0.0,
        "lead_speed": lead_speed,
        "distance": distance,
        "desired_distance": desired_distance,
        "relative_speed": relative_speed,
    }


def make_control_message(sequence: int, target_speed: float, max_speed: float, enable: bool) -> Dict[str, Any]:
    return {
        "header": make_header(sequence),
        "payload": {
            "command_id": sequence,
            "enable": enable,
            "target": {
                "target_speed_mps": target_speed,
                "target_accel_mps2": 0.0,
                "steer_rad": 0.0,
            },
            "safety": {
                "max_speed_mps": max_speed,
                "max_abs_steer_rad": 0.6,
            },
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a simple standalone ACC controller against CARLA Bridge.")
    parser.add_argument("--input-endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--output-endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--cruise-speed-mps", type=float, default=5.0)
    parser.add_argument("--min-distance-m", type=float, default=8.0)
    parser.add_argument("--time-gap-sec", type=float, default=1.5)
    parser.add_argument("--distance-gain", type=float, default=0.35)
    parser.add_argument("--relative-speed-gain", type=float, default=0.6)
    parser.add_argument("--period-sec", type=float, default=0.05)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--max-stale-sec", type=float, default=1.0)
    parser.add_argument("--print-period-sec", type=float, default=1.0)
    args = parser.parse_args()

    context = zmq.Context.instance()
    sub = context.socket(zmq.SUB)
    sub.setsockopt_string(zmq.SUBSCRIBE, EGO_TOPIC)
    sub.setsockopt_string(zmq.SUBSCRIBE, LEAD_TOPIC)
    sub.connect(args.input_endpoint)

    pub = context.socket(zmq.PUB)
    pub.connect(args.output_endpoint)
    time.sleep(0.3)

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    ego_payload: Optional[Dict[str, Any]] = None
    lead_payload: Optional[Dict[str, Any]] = None
    last_ego_time = 0.0
    sequence = 1
    next_send = time.monotonic()
    next_print = time.monotonic()
    deadline = time.monotonic() + args.duration_sec

    print(
        "[ACC] started "
        f"cruise={args.cruise_speed_mps:.2f}m/s "
        f"min_distance={args.min_distance_m:.2f}m "
        f"time_gap={args.time_gap_sec:.2f}s"
    )

    try:
        while time.monotonic() < deadline:
            events = dict(poller.poll(1))
            if sub in events:
                topic = sub.recv_string()
                msg = sub.recv_json()
                payload = extract_payload(msg)
                if topic == EGO_TOPIC:
                    ego_payload = payload
                    last_ego_time = time.monotonic()
                elif topic == LEAD_TOPIC:
                    lead_payload = payload

            now = time.monotonic()
            if now >= next_send:
                stale = now - last_ego_time if last_ego_time > 0.0 else 1.0e6
                enable = stale <= args.max_stale_sec
                state = compute_target_speed(
                    ego_payload,
                    lead_payload,
                    args.cruise_speed_mps,
                    args.min_distance_m,
                    args.time_gap_sec,
                    args.distance_gain,
                    args.relative_speed_gain,
                )
                if not enable:
                    state["target_speed"] = 0.0

                msg = make_control_message(
                    sequence,
                    state["target_speed"],
                    args.cruise_speed_mps,
                    enable,
                )
                pub.send_multipart(
                    [
                        CONTROL_TOPIC.encode("utf-8"),
                        json.dumps(msg, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                    ]
                )

                if now >= next_print:
                    print(
                        "[ACC] "
                        f"cmd={sequence} enable={int(enable)} "
                        f"ego={state['ego_speed']:.2f} "
                        f"target={state['target_speed']:.2f} "
                        f"lead_valid={int(state['lead_valid'])} "
                        f"lead={state['lead_speed']:.2f} "
                        f"distance={state['distance']:.2f} "
                        f"desired={state['desired_distance']:.2f} "
                        f"rel={state['relative_speed']:.2f}"
                    )
                    next_print = now + args.print_period_sec

                sequence = sequence + 1
                next_send = next_send + args.period_sec
    finally:
        pub.close(0)
        sub.close(0)
        context.term()

    print(f"[ACC] finished commands={sequence - 1}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
