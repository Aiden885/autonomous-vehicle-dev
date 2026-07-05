#!/usr/bin/env python3
"""Keep CARLA spectator behind the ego vehicle for visual ACC tests."""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time
from typing import Any, List, Optional


def add_carla_python_paths(root: str) -> None:
    paths: List[str] = [
        os.path.join(root, "PythonAPI", "carla"),
    ]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def role_name(actor: Any) -> str:
    return str(dict(actor.attributes).get("role_name", ""))


def find_vehicle_by_role(world: Any, target_role_name: str) -> Optional[Any]:
    result = None
    for actor in world.get_actors().filter("vehicle.*"):
        if role_name(actor) == target_role_name:
            result = actor
            break
    return result


def make_follow_transform(carla: Any, target_transform: Any, back_m: float, up_m: float, pitch_deg: float) -> Any:
    yaw_rad = math.radians(target_transform.rotation.yaw)
    location = target_transform.location + carla.Location(
        x=-back_m * math.cos(yaw_rad),
        y=-back_m * math.sin(yaw_rad),
        z=up_m,
    )
    rotation = carla.Rotation(
        pitch=pitch_deg,
        yaw=target_transform.rotation.yaw,
        roll=0.0,
    )
    return carla.Transform(location, rotation)


def fallback_transform_from_spawn(world: Any, spawn_index: int, z_offset_m: float) -> Optional[Any]:
    if spawn_index < 0:
        return None

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        return None
    if spawn_index >= len(spawn_points):
        return None

    transform = spawn_points[spawn_index]
    transform.location.z = transform.location.z + z_offset_m
    return transform


def apply_follow_or_fallback(
    carla: Any,
    world: Any,
    spectator: Any,
    ego_role_name: str,
    back_m: float,
    up_m: float,
    pitch_deg: float,
    fallback_spawn_index: int,
    fallback_z_offset_m: float,
) -> bool:
    ego = find_vehicle_by_role(world, ego_role_name)
    target_transform = ego.get_transform() if ego is not None else fallback_transform_from_spawn(
        world,
        fallback_spawn_index,
        fallback_z_offset_m,
    )
    if target_transform is None:
        return False

    spectator.set_transform(make_follow_transform(carla, target_transform, back_m, up_m, pitch_deg))
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Follow ego with CARLA spectator camera.")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--back-m", type=float, default=8.0)
    parser.add_argument("--up-m", type=float, default=6.0)
    parser.add_argument("--pitch-deg", type=float, default=-25.0)
    parser.add_argument("--period-sec", type=float, default=0.1)
    parser.add_argument("--fallback-spawn-index", type=int, default=-1)
    parser.add_argument("--fallback-z-offset-m", type=float, default=0.5)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout_sec)
    world = client.get_world()
    spectator = world.get_spectator()

    period_sec = max(args.period_sec, 0.02)
    if args.once:
        deadline = time.monotonic() + max(args.timeout_sec, period_sec)
        while True:
            updated = apply_follow_or_fallback(
                carla,
                world,
                spectator,
                args.ego_role_name,
                args.back_m,
                args.up_m,
                args.pitch_deg,
                args.fallback_spawn_index,
                args.fallback_z_offset_m,
            )
            if updated:
                break

            if time.monotonic() >= deadline:
                raise SystemExit(f"[Spectator] ego role_name={args.ego_role_name} not found")
            time.sleep(period_sec)
        return 0

    while True:
        apply_follow_or_fallback(
            carla,
            world,
            spectator,
            args.ego_role_name,
            args.back_m,
            args.up_m,
            args.pitch_deg,
            args.fallback_spawn_index,
            args.fallback_z_offset_m,
        )

        time.sleep(period_sec)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
