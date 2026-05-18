#!/usr/bin/env python3
"""Send a GAASD CARLA control command to the bridge."""

from __future__ import annotations

import argparse
import json
import time

import zmq


def make_message(command_id: int, target_speed_mps: float, steer_rad: float) -> dict:
    now_ms = int(time.time() * 1000.0)
    return {
        "header": {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": "gaasd.carla.control_cmd.v1",
            "frame_id": "gaasd_map",
            "sequence": command_id,
            "sim_time_sec": 0.0,
            "delta_time_sec": 0.0,
            "timestamp_unix_ms": now_ms,
            "source": "gaasd_control_test",
            "map_name": "",
            "ego_role_name": "hero",
            "coordinate_frame": "gaasd_map",
        },
        "payload": {
            "command_id": command_id,
            "enable": True,
            "target": {
                "target_speed_mps": target_speed_mps,
                "target_accel_mps2": 0.0,
                "steer_rad": steer_rad,
            },
            "safety": {
                "max_speed_mps": 8.0,
                "max_abs_steer_rad": 0.6,
            },
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Send bridge control command.")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--topic", default="gaasd.carla.control_cmd.v1")
    parser.add_argument("--command-id", type=int, default=1)
    parser.add_argument("--target-speed-mps", type=float, default=0.0)
    parser.add_argument("--steer-rad", type=float, default=0.0)
    parser.add_argument("--repeat", type=int, default=20)
    parser.add_argument("--period-sec", type=float, default=0.05)
    args = parser.parse_args()

    context = zmq.Context.instance()
    sock = context.socket(zmq.PUB)
    sock.connect(args.endpoint)
    time.sleep(0.3)

    for offset in range(args.repeat):
        command_id = args.command_id + offset
        msg = make_message(command_id, args.target_speed_mps, args.steer_rad)
        sock.send_multipart(
            [
                args.topic.encode("utf-8"),
                json.dumps(msg, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            ]
        )
        time.sleep(args.period_sec)

    sock.close(0)
    context.term()
    print(f"[Control] sent {args.repeat} commands to {args.endpoint}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
