#!/usr/bin/env python3
"""Record ACC bridge signals for Pangu-CARLA visual closed-loop tests."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import zmq


LEAD_TOPIC = "gaasd.carla.lead_vehicle.v1"
CHASSIS_TOPIC = "gaasd.carla.chassis_feedback.v1"
DRIVER_TOPIC = "gaasd.carla.driver_command.v1"
BRIDGE_STATUS_TOPIC = "gaasd.carla.bridge_status.v1"


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def parse_payload(raw: bytes) -> Dict[str, Any]:
    message = json.loads(raw.decode("utf-8"))
    payload = message.get("payload", message)
    return payload if isinstance(payload, dict) else {}


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def summarize(samples: List[Dict[str, Any]], started_at: float, ended_at: float) -> Dict[str, Any]:
    def values(name: str) -> List[float]:
        return [finite(sample.get(name), 0.0) for sample in samples if sample.get(name) is not None]

    clearance = values("clearance_m")
    ego_speed = values("ego_speed_mps")
    lead_speed = values("lead_speed_mps")
    throttle = values("throttle")
    brake = values("brake")
    steer = values("steer_norm")
    valid_count = sum(1 for sample in samples if bool(sample.get("lead_valid")))
    route_count = sum(
        1 for sample in samples
        if str(sample.get("selection_rule", "")).startswith("preferred_role_route")
    )

    return {
        "started_at_unix": started_at,
        "ended_at_unix": ended_at,
        "duration_sec": ended_at - started_at,
        "sample_count": len(samples),
        "lead_valid_ratio": valid_count / max(len(samples), 1),
        "route_distance_ratio": route_count / max(len(samples), 1),
        "clearance_min_m": min(clearance) if clearance else None,
        "clearance_max_m": max(clearance) if clearance else None,
        "clearance_last_m": clearance[-1] if clearance else None,
        "ego_speed_max_mps": max(ego_speed) if ego_speed else None,
        "ego_speed_last_mps": ego_speed[-1] if ego_speed else None,
        "lead_speed_mean_mps": sum(lead_speed) / len(lead_speed) if lead_speed else None,
        "throttle_max": max(throttle) if throttle else None,
        "brake_max": max(brake) if brake else None,
        "steer_max_abs": max(abs(value) for value in steer) if steer else None,
        "last_sample": samples[-1] if samples else None,
    }


def run(args: argparse.Namespace) -> int:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_path = output_dir / "acc_samples_latest.jsonl"

    context = zmq.Context.instance()
    sock = context.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "gaasd.carla.")
    sock.connect(args.endpoint)
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    started_at = time.time()
    deadline = time.monotonic() + args.duration
    latest_lead: Optional[Dict[str, Any]] = None
    latest_chassis: Optional[Dict[str, Any]] = None
    latest_driver: Optional[Dict[str, Any]] = None
    samples: List[Dict[str, Any]] = []

    with samples_path.open("w", encoding="utf-8") as sample_file:
        while time.monotonic() < deadline:
            events = dict(poller.poll(250))
            if sock not in events:
                continue
            topic = sock.recv_string()
            payload = parse_payload(sock.recv())
            if topic == LEAD_TOPIC:
                latest_lead = payload
            elif topic == CHASSIS_TOPIC:
                latest_chassis = payload
            elif topic == DRIVER_TOPIC:
                latest_driver = payload
            elif topic == BRIDGE_STATUS_TOPIC:
                pass

            if latest_lead is None or latest_chassis is None:
                continue

            sample = {
                "wall_time_sec": time.time() - started_at,
                "lead_valid": bool(latest_lead.get("valid", False)),
                "selection_rule": str(latest_lead.get("selection_rule", "")),
                "clearance_m": finite(latest_lead.get("clearance_m"), 1000000.0),
                "longitudinal_distance_m": finite(
                    latest_lead.get("longitudinal_distance_m"), 1000000.0
                ),
                "lateral_distance_m": finite(latest_lead.get("lateral_distance_m"), 0.0),
                "relative_speed_mps": finite(latest_lead.get("relative_speed_mps"), 0.0),
                "lead_speed_mps": finite(latest_lead.get("lead_speed_mps"), 0.0),
                "ego_speed_mps": finite(latest_chassis.get("speed_mps"), 0.0),
                "throttle": finite(latest_chassis.get("throttle"), 0.0),
                "brake": finite(latest_chassis.get("brake"), 0.0),
                "steer_norm": finite(latest_chassis.get("steer_norm"), 0.0),
                "last_command_age_sec": finite(
                    latest_chassis.get("last_command_age_sec"), 1000000.0
                ),
                "driver_command": int((latest_driver or {}).get("command_type", 0)),
            }
            samples.append(sample)
            sample_file.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")

    ended_at = time.time()
    summary = summarize(samples, started_at, ended_at)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(ended_at))
    write_json(output_dir / f"acc_run_{stamp}.json", summary)
    write_json(output_dir / "latest_summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    sock.close(0)
    return 0 if len(samples) >= args.min_samples else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--output-dir", default="/tmp/newaccpro3-pangu-carla/results")
    parser.add_argument("--min-samples", type=int, default=20)
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
