#!/usr/bin/env python3
"""Temporarily push CARLA ego vehicle above ACC vMin for visual demos."""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time
from typing import Any


def add_carla_paths(carla_root: str) -> None:
    candidates = [os.path.join(carla_root, "PythonAPI", "carla")]
    candidates.extend(
        glob.glob(os.path.join(carla_root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg"))
    )
    for path in reversed(candidates):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def find_ego(world: Any, role_name: str) -> Any:
    vehicles = world.get_actors().filter("vehicle.*")
    for actor in vehicles:
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def velocity_from_yaw(carla: Any, yaw_deg: float, speed_mps: float) -> Any:
    yaw_rad = math.radians(yaw_deg)
    return carla.Vector3D(
        x=math.cos(yaw_rad) * speed_mps,
        y=math.sin(yaw_rad) * speed_mps,
        z=0.0,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--speed-mps", type=float, default=2.0)
    parser.add_argument("--duration-sec", type=float, default=2.0)
    parser.add_argument("--period-sec", type=float, default=0.05)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    args = parser.parse_args()

    add_carla_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    deadline = time.monotonic() + args.timeout_sec
    ego = None
    while time.monotonic() < deadline:
        ego = find_ego(world, args.ego_role_name)
        if ego is not None:
            break
        time.sleep(0.1)
    if ego is None:
        print(f"[BoostEgo] ego role_name={args.ego_role_name} not found", file=sys.stderr)
        return 1

    start = time.monotonic()
    count = 0
    while time.monotonic() - start < args.duration_sec:
        transform = ego.get_transform()
        velocity = velocity_from_yaw(carla, transform.rotation.yaw, args.speed_mps)
        ego.set_target_velocity(velocity)
        ego.apply_control(carla.VehicleControl(throttle=0.35, steer=0.0, brake=0.0))
        count += 1
        time.sleep(max(args.period_sec, 0.01))

    print(
        f"[BoostEgo] applied speed={args.speed_mps:.2f}m/s "
        f"duration={args.duration_sec:.2f}s updates={count}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
