#!/usr/bin/env python3
"""Mock CARLA bridge for ACC Pangu closed-loop tests.

The mock keeps the same ZMQ direction as the real CARLA bridge:
  - PUB 5701: ego_state, lead_vehicle, driver_command
  - SUB 5702: control_cmd, driver_command
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass

import zmq


EGO_TOPIC = "gaasd.carla.ego_state.v1"
LEAD_TOPIC = "gaasd.carla.lead_vehicle.v1"
CONTROL_TOPIC = "gaasd.carla.control_cmd.v1"
DRIVER_TOPIC = "gaasd.carla.driver_command.v1"


@dataclass
class DriverCommand:
    command_type: int = 0
    mode: str = "pulse"
    active: bool = False
    event_id: int = 0
    description: str = ""
    publish_once: bool = False


def unix_ms() -> int:
    return int(time.time() * 1000)


def make_message(topic: str, sequence: int, sim_time_sec: float, payload: dict) -> dict:
    return {
        "header": {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": topic,
            "frame_id": "mock_map",
            "sequence": sequence,
            "sim_time_sec": sim_time_sec,
            "delta_time_sec": 0.0,
            "timestamp_unix_ms": unix_ms(),
            "source": "pangu_mock_carla_bridge",
            "map_name": "mock_straight",
            "ego_role_name": "hero",
            "coordinate_frame": "gaasd_map",
        },
        "payload": payload,
    }


def send_json(pub: zmq.Socket, topic: str, message: dict) -> None:
    text = json.dumps(message, ensure_ascii=False, separators=(",", ":"))
    pub.send_multipart([topic.encode("utf-8"), text.encode("utf-8")])


def parse_payload(raw: bytes) -> dict:
    message = json.loads(raw.decode("utf-8"))
    payload = message.get("payload", message)
    if not isinstance(payload, dict):
        return {}
    return payload


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def run(args: argparse.Namespace) -> None:
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    sub = ctx.socket(zmq.SUB)
    pub.bind(args.pub_endpoint)
    sub.bind(args.sub_endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, CONTROL_TOPIC)
    sub.setsockopt_string(zmq.SUBSCRIBE, DRIVER_TOPIC)

    ego_speed = args.ego_speed
    lead_speed = args.lead_speed
    distance = args.distance
    target_speed = 0.0
    control_enable = False
    command = DriverCommand()
    sequence = 1
    start = time.monotonic()
    next_tick = start
    dt = args.period

    print(f"[MockBridge] PUB {args.pub_endpoint}")
    print(f"[MockBridge] SUB {args.sub_endpoint}")
    print("[MockBridge] Ctrl+C to stop")

    while True:
        now = time.monotonic()
        while True:
            try:
                topic_raw, payload_raw = sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            topic = topic_raw.decode("utf-8")
            payload = parse_payload(payload_raw)
            if topic == CONTROL_TOPIC:
                target = payload.get("target", {})
                target_speed = float(target.get("target_speed_mps", target_speed))
                control_enable = bool(payload.get("enable", control_enable))
                print(
                    f"[MockBridge] control target={target_speed:.4f} "
                    f"enable={int(control_enable)}",
                    flush=True,
                )
            elif topic == DRIVER_TOPIC:
                command.command_type = int(payload.get("command_type", 0))
                command.mode = str(payload.get("mode", "pulse"))
                command.active = bool(payload.get("active", command.command_type != 0))
                command.event_id = int(payload.get("event_id", command.event_id + 1))
                command.description = str(payload.get("description", ""))
                command.publish_once = True
                print(
                    f"[MockBridge] driver cmd={command.command_type} "
                    f"mode={command.mode} active={int(command.active)}",
                    flush=True,
                )

        if now < next_tick:
            time.sleep(min(0.005, next_tick - now))
            continue

        sim_time = now - start
        next_tick = next_tick + dt
        if control_enable:
            ego_speed += clamp(target_speed - ego_speed, -args.max_accel * dt,
                               args.max_accel * dt)
        else:
            ego_speed = max(0.0, ego_speed - args.coast_decel * dt)
        distance = max(0.0, distance + (lead_speed - ego_speed) * dt)

        ego_payload = {
            "role_name": "hero",
            "ego_id": 1,
            "vehicle": {"length_m": 4.5, "width_m": 1.8, "height_m": 1.5},
            "velocity": {"speed_mps": ego_speed, "longitudinal_mps": ego_speed},
            "acceleration": {"longitudinal_mps2": 0.0},
        }
        lead_payload = {
            "valid": True,
            "object_id": 2,
            "type": "vehicle",
            "selection_rule": "same_lane_nearest_front",
            "ego_speed_mps": ego_speed,
            "lead_speed_mps": lead_speed,
            "clearance_m": distance,
            "longitudinal_distance_m": distance,
            "relative_speed_mps": lead_speed - ego_speed,
            "time_gap_sec": distance / max(ego_speed, 0.1),
            "ttc_sec": 1000000.0 if ego_speed <= lead_speed
            else distance / max(ego_speed - lead_speed, 0.1),
        }

        send_json(pub, EGO_TOPIC, make_message(EGO_TOPIC, sequence, sim_time, ego_payload))
        send_json(pub, LEAD_TOPIC, make_message(LEAD_TOPIC, sequence, sim_time, lead_payload))
        if command.publish_once or (command.mode == "level" and command.active):
            payload = {
                "command_type": command.command_type if command.active else 0,
                "mode": command.mode,
                "active": command.active,
                "event_id": command.event_id,
                "description": command.description,
                "source": "mock_keyboard",
            }
            send_json(pub, DRIVER_TOPIC, make_message(DRIVER_TOPIC, sequence, sim_time, payload))
            command.publish_once = False
        sequence += 1

        if sequence % int(max(1, round(1.0 / dt))) == 0:
            print(
                f"[MockBridge] ego={ego_speed:.2f} lead={lead_speed:.2f} "
                f"distance={distance:.2f} target={target_speed:.2f} "
                f"enable={int(control_enable)} cmd={command.command_type if command.active else 0}"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pub-endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--sub-endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--ego-speed", type=float, default=0.0)
    parser.add_argument("--lead-speed", type=float, default=2.0)
    parser.add_argument("--distance", type=float, default=25.0)
    parser.add_argument("--max-accel", type=float, default=1.5)
    parser.add_argument("--coast-decel", type=float, default=0.4)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
