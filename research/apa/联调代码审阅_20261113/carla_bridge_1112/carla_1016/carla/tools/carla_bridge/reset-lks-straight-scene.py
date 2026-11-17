#!/usr/bin/env python3.8
"""Reset CARLA to a repeatable single-ego LKS validation scene."""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time
from typing import Any, List, Optional


def add_carla_python_paths(root: str) -> None:
    paths: List[str] = [os.path.join(root, "PythonAPI", "carla")]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def actor_role_name(actor: Any) -> str:
    return str(dict(actor.attributes).get("role_name", ""))


def find_vehicle_by_role(world: Any, role_name: str) -> Optional[Any]:
    for actor in world.get_actors().filter("vehicle.*"):
        if actor_role_name(actor) == role_name:
            return actor
    return None


def wait_for_vehicle_by_role(world: Any, role_name: str, timeout_sec: float) -> Optional[Any]:
    deadline = time.monotonic() + timeout_sec
    actor = find_vehicle_by_role(world, role_name)
    while actor is None and time.monotonic() < deadline:
        time.sleep(0.1)
        actor = find_vehicle_by_role(world, role_name)
    return actor


def destroy_vehicles_by_role(world: Any, role_name: str) -> int:
    count = 0
    for actor in world.get_actors().filter("vehicle.*"):
        if actor_role_name(actor) == role_name:
            try:
                actor.disable_constant_velocity()
            except RuntimeError:
                pass
            actor.destroy()
            count = count + 1
    return count


def stop_vehicle(carla: Any, actor: Any) -> None:
    try:
        actor.disable_constant_velocity()
    except RuntimeError:
        pass
    try:
        actor.set_autopilot(False)
    except RuntimeError:
        pass
    actor.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    actor.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    actor.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))


def main() -> int:
    parser = argparse.ArgumentParser(description="Reset a single-ego straight-road LKS scene.")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--wait-ego-sec", type=float, default=15.0)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--lead-role-name", default="gaasd_lead")
    parser.add_argument("--ego-spawn-index", type=int, default=198)
    parser.add_argument("--lateral-offset-m", type=float, default=0.8)
    parser.add_argument("--heading-error-deg", type=float, default=5.0)
    parser.add_argument("--spawn-z-offset-m", type=float, default=0.5)
    args = parser.parse_args()

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout_sec)
    world = client.get_world()
    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    if not spawn_points:
        raise SystemExit("[ResetLKS] no spawn points in current map")

    ego = wait_for_vehicle_by_role(world, args.ego_role_name, args.wait_ego_sec)
    if ego is None:
        raise SystemExit(f"[ResetLKS] ego role_name={args.ego_role_name} not found")

    destroyed = destroy_vehicles_by_role(world, args.lead_role_name)
    index = args.ego_spawn_index % len(spawn_points)
    base_transform = spawn_points[index]
    waypoint = carla_map.get_waypoint(
        base_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        raise SystemExit("[ResetLKS] selected spawn point is not on a driving lane")

    forward = waypoint.transform.get_forward_vector()
    right_x = -forward.y
    right_y = forward.x
    right_norm = math.hypot(right_x, right_y)
    if right_norm <= 1.0e-6:
        raise SystemExit("[ResetLKS] cannot calculate lane right direction")
    right_x = right_x / right_norm
    right_y = right_y / right_norm

    transform = waypoint.transform
    transform.location.x = transform.location.x - (right_x * args.lateral_offset_m)
    transform.location.y = transform.location.y - (right_y * args.lateral_offset_m)
    transform.location.z = transform.location.z + args.spawn_z_offset_m
    transform.rotation.yaw = transform.rotation.yaw - args.heading_error_deg

    stop_vehicle(carla, ego)
    ego.set_transform(transform)
    stop_vehicle(carla, ego)

    print(
        "[ResetLKS] ego "
        f"id={ego.id} spawn_index={index} lateral_offset={args.lateral_offset_m:.2f}m "
        f"heading_error={args.heading_error_deg:.2f}deg "
        f"loc=({transform.location.x:.2f},{transform.location.y:.2f},{transform.location.z:.2f}) "
        f"yaw={transform.rotation.yaw:.2f} road={waypoint.road_id} lane={waypoint.lane_id}"
    )
    print(f"[ResetLKS] removed lead vehicles: {destroyed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
