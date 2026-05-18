#!/usr/bin/env python3
"""Verify that the CARLA lead vehicle is valid and moving.

The script reads Bridge PUB messages only. It does not control CARLA and does
not depend on GAASD UI output, so it can be used before running a GAASD dynamic
lead test.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from typing import Any, Dict, List, Optional, Tuple

import zmq


LEAD_TOPIC = "gaasd.carla.lead_vehicle.v1"
OBJECT_LIST_TOPIC = "gaasd.carla.object_list.v1"


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if math.isfinite(number):
        return number
    return default


def object_pose(payload: Dict[str, Any], role_name: str, object_id: int) -> Optional[Tuple[float, float, float]]:
    for obj in payload.get("objects", []):
        obj_role = str(obj.get("role_name", ""))
        obj_id = int(obj.get("object_id", 0))
        role_matches = bool(role_name) and obj_role == role_name
        id_matches = object_id > 0 and obj_id == object_id
        if not role_matches and not id_matches:
            continue

        pose = obj.get("pose", {})
        velocity = obj.get("velocity", {})
        return (
            finite(pose.get("x_m")),
            finite(pose.get("y_m")),
            finite(velocity.get("speed_mps")),
        )
    return None


def summarize_values(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"min": 0.0, "max": 0.0, "mean": 0.0}
    return {
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify lead vehicle motion from Bridge PUB topics.")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5701")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--lead-role-name", default="gaasd_lead")
    parser.add_argument("--min-valid-ratio", type=float, default=0.8)
    parser.add_argument("--min-speed-mps", type=float, default=0.2)
    parser.add_argument("--min-moving-samples", type=int, default=5)
    parser.add_argument("--min-displacement-m", type=float, default=0.5)
    parser.add_argument("--json", action="store_true", help="Print machine-readable summary only.")
    args = parser.parse_args()

    context = zmq.Context.instance()
    sock = context.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "gaasd.carla.")
    sock.connect(args.endpoint)

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    deadline = time.time() + args.duration_sec
    lead_samples: List[Dict[str, Any]] = []
    object_samples: List[Tuple[float, float, float]] = []
    latest_object_id = 0

    while time.time() < deadline:
        timeout_ms = max(1, int((deadline - time.time()) * 1000.0))
        events = dict(poller.poll(timeout_ms))
        if sock not in events:
            continue

        topic = sock.recv_string()
        message = sock.recv_json()
        payload = message.get("payload", {})

        if topic == LEAD_TOPIC:
            object_id = int(payload.get("object_id", 0))
            if object_id > 0:
                latest_object_id = object_id
            lead_samples.append(
                {
                    "valid": bool(payload.get("valid", False)),
                    "speed": finite(payload.get("lead_speed_mps")),
                    "distance": finite(payload.get("clearance_m"), 1.0e6),
                    "rule": str(payload.get("selection_rule", "")),
                    "object_id": object_id,
                }
            )
        elif topic == OBJECT_LIST_TOPIC:
            pose = object_pose(payload, args.lead_role_name, latest_object_id)
            if pose is not None:
                object_samples.append(pose)

    sock.close(0)
    context.term()

    valid_count = sum(1 for item in lead_samples if item["valid"])
    valid_ratio = valid_count / max(len(lead_samples), 1)
    speeds = [item["speed"] for item in lead_samples if item["valid"]]
    distances = [item["distance"] for item in lead_samples if item["valid"]]
    moving_samples = sum(1 for speed in speeds if speed >= args.min_speed_mps)
    speed_stats = summarize_values(speeds)
    distance_stats = summarize_values(distances)

    displacement = 0.0
    object_speed_stats = {"min": 0.0, "max": 0.0, "mean": 0.0}
    if object_samples:
        first = object_samples[0]
        last = object_samples[-1]
        displacement = math.hypot(last[0] - first[0], last[1] - first[1])
        object_speed_stats = summarize_values([item[2] for item in object_samples])

    result = {
        "duration_sec": args.duration_sec,
        "lead_samples": len(lead_samples),
        "valid_count": valid_count,
        "valid_ratio": valid_ratio,
        "lead_speed_mps": speed_stats,
        "moving_samples": moving_samples,
        "distance_m": distance_stats,
        "object_samples": len(object_samples),
        "object_speed_mps": object_speed_stats,
        "object_displacement_m": displacement,
        "passed": False,
    }

    valid_ok = valid_ratio >= args.min_valid_ratio
    speed_ok = moving_samples >= args.min_moving_samples
    displacement_ok = displacement >= args.min_displacement_m
    result["passed"] = bool(valid_ok and speed_ok and displacement_ok)

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print("[LeadMotion] samples:", result["lead_samples"])
        print(f"[LeadMotion] valid: {valid_count}/{len(lead_samples)} ratio={valid_ratio:.3f}")
        print(
            "[LeadMotion] lead_speed_mps: "
            f"min={speed_stats['min']:.3f} max={speed_stats['max']:.3f} mean={speed_stats['mean']:.3f} "
            f"moving_samples(>={args.min_speed_mps:.3f})={moving_samples}"
        )
        print(
            "[LeadMotion] distance_m: "
            f"min={distance_stats['min']:.3f} max={distance_stats['max']:.3f} mean={distance_stats['mean']:.3f}"
        )
        print(
            "[LeadMotion] object_motion: "
            f"samples={len(object_samples)} displacement={displacement:.3f}m "
            f"speed_max={object_speed_stats['max']:.3f}m/s"
        )
        print(
            "[LeadMotion] checks: "
            f"valid_ok={valid_ok} speed_ok={speed_ok} displacement_ok={displacement_ok}"
        )
        print("[LeadMotion] RESULT:", "PASS" if result["passed"] else "FAIL")

    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
