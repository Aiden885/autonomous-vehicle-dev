#!/usr/bin/env python3.8
"""Reset ego to the historical official-map LKS start on Town05."""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from typing import Any, Optional


def add_carla_python_paths(root: str) -> None:
    paths = [os.path.join(root, "PythonAPI", "carla")]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def find_ego(world: Any, role_name: str, timeout_sec: float) -> Optional[Any]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == role_name:
                return actor
        time.sleep(0.1)
    return None


def stop_vehicle(carla: Any, actor: Any) -> None:
    try:
        actor.disable_constant_velocity()
    except RuntimeError:
        pass
    actor.set_autopilot(False)
    actor.set_target_velocity(carla.Vector3D())
    actor.set_target_angular_velocity(carla.Vector3D())
    actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--expected-map", default="Town05")
    parser.add_argument("--reference-x", type=float, default=0.663731)
    parser.add_argument("--reference-y", type=float, default=-203.651886)
    parser.add_argument("--ego-x", type=float, default=0.663731)
    parser.add_argument("--ego-y", type=float, default=-203.651886)
    parser.add_argument("--ego-z", type=float, default=0.5)
    parser.add_argument("--lateral-offset-m", type=float, default=0.0)
    args = parser.parse_args()

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    map_name = world.get_map().name
    if not map_name.endswith(args.expected_map):
        raise SystemExit(f"[ResetLKSReference] expected {args.expected_map}, got {map_name}")
    ego = find_ego(world, args.ego_role_name, 15.0)
    if ego is None:
        raise SystemExit("[ResetLKSReference] hero ego not found")

    reference_location = carla.Location(x=args.reference_x, y=args.reference_y, z=args.ego_z)
    waypoint = world.get_map().get_waypoint(
        reference_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        raise SystemExit("[ResetLKSReference] reference waypoint not found")

    transform = carla.Transform(
        carla.Location(x=args.ego_x, y=args.ego_y, z=args.ego_z),
        waypoint.transform.rotation,
    )
    if abs(args.lateral_offset_m) > 1.0e-9:
        right = transform.get_right_vector()
        transform.location.x += right.x * args.lateral_offset_m
        transform.location.y += right.y * args.lateral_offset_m

    for actor in world.get_actors().filter("vehicle.*"):
        if actor.id != ego.id and actor.attributes.get("role_name") == "gaasd_lead":
            actor.destroy()
    stop_vehicle(carla, ego)
    ego.set_transform(transform)
    stop_vehicle(carla, ego)
    actual_waypoint = world.get_map().get_waypoint(transform.location, project_to_road=True)
    print(
        "[ResetLKSReference] ego id={} map={} loc=({:.3f},{:.3f},{:.3f}) "
        "yaw={:.3f} offset={:.3f} road={} lane={}".format(
            ego.id,
            map_name,
            transform.location.x,
            transform.location.y,
            transform.location.z,
            transform.rotation.yaw,
            args.lateral_offset_m,
            actual_waypoint.road_id if actual_waypoint else 0,
            actual_waypoint.lane_id if actual_waypoint else 0,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
