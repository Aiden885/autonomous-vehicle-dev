#!/usr/bin/env python3
"""Reliably publish LKS driver takeover inputs to the CARLA Bridge."""

import argparse
import json
import time

import zmq


TOPIC = "gaasd.carla.driver_state_cmd.v1"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5702")
    parser.add_argument("--brake", type=int, choices=(0, 1), default=0)
    parser.add_argument("--steer", type=float, default=0.0)
    parser.add_argument("--repeat-count", type=int, default=6)
    parser.add_argument("--interval-ms", type=float, default=50.0)
    parser.add_argument("--settle-ms", type=float, default=200.0)
    args = parser.parse_args()

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(args.endpoint)
    time.sleep(max(args.settle_ms, 0.0) / 1000.0)
    payload = {
        "header": {
            "protocol": "gaasd_carla_bridge",
            "protocol_version": "0.3.0",
            "message_type": TOPIC,
            "timestamp_unix_ms": int(time.time() * 1000),
            "source": "gaasd_scenario_panel",
        },
        "payload": {
            "brake_pressed": bool(args.brake),
            "driver_steer_norm": max(-1.0, min(1.0, args.steer)),
        },
    }
    frames = [TOPIC.encode("utf-8"), json.dumps(payload, separators=(",", ":")).encode("utf-8")]
    repeat_count = max(args.repeat_count, 1)
    for index in range(repeat_count):
        socket.send_multipart(frames)
        if index + 1 < repeat_count:
            time.sleep(max(args.interval_ms, 0.0) / 1000.0)
    print(
        "driver_state_cmd brake={} steer={:.3f} sent={}".format(
            int(payload["payload"]["brake_pressed"]),
            payload["payload"]["driver_steer_norm"],
            repeat_count,
        )
    )
    socket.close(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
