#!/usr/bin/env python3
"""Probe GAASD CARLA bridge PUB topics."""

from __future__ import annotations

import argparse
import json
import time

import zmq


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe bridge PUB endpoint.")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--topic-prefix", default="gaasd.carla")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-messages", type=int, default=1)
    parser.add_argument("--max-print", type=int, default=20)
    parser.add_argument("--show-payload", action="store_true")
    args = parser.parse_args()

    context = zmq.Context.instance()
    sock = context.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, args.topic_prefix)
    sock.connect(args.endpoint)

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    deadline = time.time() + args.duration
    count = 0

    while time.time() < deadline:
        timeout_ms = max(1, int((deadline - time.time()) * 1000.0))
        events = dict(poller.poll(timeout_ms))
        if sock not in events:
            continue

        topic = sock.recv_string()
        payload = sock.recv_json()
        header = payload.get("header", {})
        body = payload.get("payload", {})
        keys = sorted(body.keys())
        if count < args.max_print:
            summary = {
                "topic": topic,
                "sequence": header.get("sequence"),
                "sim_time_sec": header.get("sim_time_sec"),
                "payload_keys": keys,
            }
            if args.show_payload:
                summary["payload"] = body
            print(json.dumps(summary, ensure_ascii=False))
        count = count + 1

    sock.close(0)
    context.term()

    if count < args.min_messages:
        print(f"[Probe] received {count} messages, expected at least {args.min_messages}")
        return 1

    print(f"[Probe] received {count} messages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
