#!/usr/bin/env python3.8
"""Reset CARLA to a repeatable straight-road ACC scene.

This is a test helper for GAASD-CARLA P0 validation. It assumes CARLA and the
Bridge are already running, then:
1. Moves ego to a known long straight segment in Town01.
2. Removes old GAASD lead vehicles.
3. Spawns a constant-velocity lead vehicle ahead of ego.
4. Places the CARLA spectator behind ego.
"""

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


def wait_for_world_frame(world: Any, timeout_sec: float) -> None:
    try:
        world.wait_for_tick(timeout_sec)
    except RuntimeError:
        # If another client is ticking the world, actor state may still be usable.
        pass


def find_vehicle_by_role(world: Any, role_name: str) -> Optional[Any]:
    result = None
    for actor in world.get_actors().filter("vehicle.*"):
        if actor_role_name(actor) == role_name:
            result = actor
            break
    return result


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


def constant_velocity_vector(carla: Any, transform: Any, speed_mps: float) -> Any:
    yaw_rad = math.radians(transform.rotation.yaw)
    return carla.Vector3D(
        x=speed_mps * math.cos(yaw_rad),
        y=speed_mps * math.sin(yaw_rad),
        z=0.0,
    )


def make_spectator_transform(carla: Any, target_transform: Any, back_m: float, up_m: float, pitch_deg: float) -> Any:
    yaw_rad = math.radians(target_transform.rotation.yaw)
    location = target_transform.location + carla.Location(
        x=-back_m * math.cos(yaw_rad),
        y=-back_m * math.sin(yaw_rad),
        z=up_m,
    )
    rotation = carla.Rotation(pitch=pitch_deg, yaw=target_transform.rotation.yaw, roll=0.0)
    return carla.Transform(location, rotation)


def main() -> int:
    parser = argparse.ArgumentParser(description="Reset a straight-road ACC scene for GAASD-CARLA tests.")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--ego-role-name", default="hero")
    parser.add_argument("--lead-role-name", default="gaasd_lead")
    parser.add_argument("--ego-spawn-index", type=int, default=198)
    parser.add_argument("--lead-distance-m", type=float, default=25.0)
    parser.add_argument("--lead-speed-mps", type=float, default=2.0)
    parser.add_argument("--spawn-z-offset-m", type=float, default=0.5)
    parser.add_argument("--blueprint-filter", default="vehicle.*model3*")
    parser.add_argument("--spectator-back-m", type=float, default=14.0)
    parser.add_argument("--spectator-up-m", type=float, default=8.0)
    parser.add_argument("--spectator-pitch-deg", type=float, default=-28.0)
    args = parser.parse_args()

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout_sec)
    world = client.get_world()
    wait_for_world_frame(world, args.timeout_sec)

    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    if not spawn_points:
        raise SystemExit("[ResetScene] no spawn points in current map")
    if args.ego_spawn_index < 0 or args.ego_spawn_index >= len(spawn_points):
        raise SystemExit(
            f"[ResetScene] ego spawn index {args.ego_spawn_index} out of range 0..{len(spawn_points) - 1}"
        )

    ego = find_vehicle_by_role(world, args.ego_role_name)
    if ego is None:
        raise SystemExit(f"[ResetScene] ego role_name={args.ego_role_name} not found")

    destroyed = destroy_vehicles_by_role(world, args.lead_role_name)
    wait_for_world_frame(world, args.timeout_sec)

    ego_transform = spawn_points[args.ego_spawn_index]
    ego_transform.location.z = ego_transform.location.z + args.spawn_z_offset_m
    stop_vehicle(carla, ego)
    ego.set_transform(ego_transform)
    stop_vehicle(carla, ego)
    wait_for_world_frame(world, args.timeout_sec)

    ego_waypoint = carla_map.get_waypoint(
        ego_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if ego_waypoint is None:
        raise SystemExit("[ResetScene] ego spawn point is not on a driving lane")

    lead_candidates = ego_waypoint.next(args.lead_distance_m)
    if not lead_candidates:
        raise SystemExit("[ResetScene] no waypoint ahead for lead vehicle")
    lead_transform = lead_candidates[0].transform
    lead_transform.location.z = lead_transform.location.z + args.spawn_z_offset_m

    blueprints = world.get_blueprint_library().filter(args.blueprint_filter)
    if not blueprints:
        raise SystemExit(f"[ResetScene] no blueprint matched {args.blueprint_filter}")
    lead_blueprint = blueprints[0]
    lead_blueprint.set_attribute("role_name", args.lead_role_name)

    lead = world.try_spawn_actor(lead_blueprint, lead_transform)
    if lead is None:
        raise SystemExit("[ResetScene] failed to spawn lead vehicle")
    lead.enable_constant_velocity(constant_velocity_vector(carla, lead_transform, args.lead_speed_mps))

    spectator = world.get_spectator()
    spectator.set_transform(
        make_spectator_transform(
            carla,
            ego_transform,
            args.spectator_back_m,
            args.spectator_up_m,
            args.spectator_pitch_deg,
        )
    )

    ego_wp = carla_map.get_waypoint(ego_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    lead_wp = carla_map.get_waypoint(lead_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    time.sleep(0.1)

    print(
        "[ResetScene] ego "
        f"id={ego.id} spawn_index={args.ego_spawn_index} "
        f"loc=({ego_transform.location.x:.2f},{ego_transform.location.y:.2f},{ego_transform.location.z:.2f}) "
        f"yaw={ego_transform.rotation.yaw:.2f} "
        f"road={ego_wp.road_id if ego_wp else 'unknown'} lane={ego_wp.lane_id if ego_wp else 'unknown'}"
    )
    print(
        "[ResetScene] lead "
        f"id={lead.id} destroyed_old={destroyed} "
        f"distance={args.lead_distance_m:.2f}m speed={args.lead_speed_mps:.2f}m/s "
        f"loc=({lead_transform.location.x:.2f},{lead_transform.location.y:.2f},{lead_transform.location.z:.2f}) "
        f"yaw={lead_transform.rotation.yaw:.2f} "
        f"road={lead_wp.road_id if lead_wp else 'unknown'} lane={lead_wp.lane_id if lead_wp else 'unknown'}"
    )
    print("[ResetScene] spectator updated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
