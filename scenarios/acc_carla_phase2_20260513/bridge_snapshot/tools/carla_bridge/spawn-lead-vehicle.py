#!/usr/bin/env python3
"""Spawn a simple lead vehicle in front of the CARLA ego vehicle."""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time
from typing import Any, Dict, List


def add_carla_python_paths(root: str) -> None:
    paths: List[str] = [
        os.path.join(root, "PythonAPI", "carla"),
    ]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def get_actor_by_role(world: Any, role_name: str):
    actors = world.get_actors().filter("vehicle.*")
    for actor in actors:
        if str(dict(actor.attributes).get("role_name", "")) == role_name:
            return actor
    return None


def list_vehicle_roles(world: Any) -> List[str]:
    roles: List[str] = []
    actors = world.get_actors().filter("vehicle.*")
    for actor in actors:
        role_name = str(dict(actor.attributes).get("role_name", ""))
        roles.append(f"{actor.id}:{actor.type_id}:{role_name}")
    return roles


def wait_actor_by_role(client: Any, role_name: str, timeout_sec: float):
    deadline = time.monotonic() + timeout_sec
    actor = None
    world = client.get_world()
    while actor is None and time.monotonic() < deadline:
        world = client.get_world()
        try:
            world.wait_for_tick(1.0)
        except RuntimeError:
            pass
        actor = get_actor_by_role(world, role_name)
        if actor is None:
            time.sleep(0.1)
    return actor, world


def make_ego_forward_transform(carla: Any, ego_transform: Any, distance_m: float):
    forward = ego_transform.get_forward_vector()
    location = ego_transform.location + carla.Location(
        x=forward.x * distance_m,
        y=forward.y * distance_m,
        z=0.5,
    )
    return carla.Transform(location, ego_transform.rotation)


def make_lane_waypoint_transform(carla: Any, world: Any, ego_transform: Any, distance_m: float):
    waypoint = world.get_map().get_waypoint(
        ego_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    candidates = waypoint.next(distance_m)
    if not candidates:
        return None
    transform = candidates[0].transform
    transform.location.z = transform.location.z + 0.5
    return transform


def destroy_old_leads(world: Any, role_name: str) -> int:
    count = 0
    for actor in world.get_actors().filter("vehicle.*"):
        if str(dict(actor.attributes).get("role_name", "")) == role_name:
            try:
                actor.disable_constant_velocity()
            except RuntimeError:
                pass
            actor.destroy()
            count = count + 1
    return count


def configure_constant_velocity(carla: Any, actor: Any, transform: Any, speed_mps: float) -> None:
    yaw_rad = math.radians(transform.rotation.yaw)
    velocity = carla.Vector3D(
        x=speed_mps * math.cos(yaw_rad),
        y=speed_mps * math.sin(yaw_rad),
        z=0.0,
    )
    actor.enable_constant_velocity(velocity)


def configure_traffic_manager(client: Any, world: Any, actor: Any, speed_mps: float, tm_port: int) -> str:
    traffic_manager = client.get_trafficmanager(tm_port)
    settings = world.get_settings()
    try:
        traffic_manager.set_synchronous_mode(bool(settings.synchronous_mode))
    except RuntimeError:
        pass

    actor.set_autopilot(True, tm_port)

    for method_name, value in (
        ("auto_lane_change", False),
        ("ignore_lights_percentage", 100.0),
        ("ignore_signs_percentage", 100.0),
    ):
        method = getattr(traffic_manager, method_name, None)
        if method is not None:
            method(actor, value)

    speed_kmh = speed_mps * 3.6
    speed_limit_kmh = max(float(actor.get_speed_limit()), 1.0)
    percent_slower = max(-100.0, min(100.0, 100.0 - speed_kmh / speed_limit_kmh * 100.0))
    traffic_manager.vehicle_percentage_speed_difference(actor, percent_slower)
    return f"traffic_manager speed_limit={speed_limit_kmh:.1f}km/h target={speed_kmh:.1f}km/h pct_diff={percent_slower:.1f}%"


def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn a CARLA lead vehicle ahead of ego.")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--lead-role-name", default="gaasd_lead")
    parser.add_argument("--wait-ego-sec", type=float, default=5.0)
    parser.add_argument("--distance-m", type=float, default=25.0)
    parser.add_argument("--speed-mps", type=float, default=2.0)
    parser.add_argument("--placement", choices=["ego_forward", "lane_waypoint"], default="ego_forward")
    parser.add_argument("--behavior", choices=["traffic_manager", "constant_velocity"], default="traffic_manager")
    parser.add_argument("--traffic-manager-port", type=int, default=8000)
    parser.add_argument("--blueprint-filter", default="vehicle.*model3*")
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout_sec)
    ego, world = wait_actor_by_role(client, args.ego_role_name, args.wait_ego_sec)
    if ego is None:
        roles = ", ".join(list_vehicle_roles(world))
        raise SystemExit(f"[Lead] ego role_name={args.ego_role_name} not found; vehicles=[{roles}]")

    if args.replace:
        count = destroy_old_leads(world, args.lead_role_name)
        print(f"[Lead] destroyed old lead vehicles: {count}")

    ego_transform = ego.get_transform()
    if args.placement == "ego_forward":
        spawn_transform = make_ego_forward_transform(carla, ego_transform, args.distance_m)
    else:
        spawn_transform = make_lane_waypoint_transform(carla, world, ego_transform, args.distance_m)
        if spawn_transform is None:
            raise SystemExit("[Lead] no waypoint ahead of ego")

    blueprints = world.get_blueprint_library().filter(args.blueprint_filter)
    if not blueprints:
        raise SystemExit(f"[Lead] no blueprint matched {args.blueprint_filter}")

    blueprint = blueprints[0]
    blueprint.set_attribute("role_name", args.lead_role_name)
    actor = world.try_spawn_actor(blueprint, spawn_transform)
    if actor is None:
        raise SystemExit("[Lead] failed to spawn lead vehicle")

    behavior_desc = ""
    if args.behavior == "traffic_manager":
        behavior_desc = configure_traffic_manager(client, world, actor, args.speed_mps, args.traffic_manager_port)
    else:
        configure_constant_velocity(carla, actor, spawn_transform, args.speed_mps)
        behavior_desc = "constant_velocity"
    time.sleep(0.1)

    print(
        "[Lead] spawned "
        f"id={actor.id} role_name={args.lead_role_name} "
        f"placement={args.placement} behavior={behavior_desc} "
        f"distance={args.distance_m:.2f}m speed={args.speed_mps:.2f}m/s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
