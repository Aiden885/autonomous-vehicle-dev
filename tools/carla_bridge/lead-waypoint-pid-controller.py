#!/usr/bin/env python3.8
"""Control the ACC lead vehicle with deterministic waypoint + speed PID."""

from __future__ import annotations

import argparse
import glob
import math
import os
import signal
import sys
import time
from typing import Any, List, Optional


_SHOULD_STOP = False


def handle_signal(signum: int, frame: object) -> None:
    global _SHOULD_STOP
    _SHOULD_STOP = True


def add_carla_python_paths(root: str) -> None:
    paths: List[str] = [os.path.join(root, "PythonAPI", "carla")]
    paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def actor_role_name(actor: Any) -> str:
    return str(dict(actor.attributes).get("role_name", ""))


def find_vehicle_by_role(world: Any, role_name: str) -> Optional[Any]:
    result = None
    for actor in world.get_actors().filter("vehicle.*"):
        if actor_role_name(actor) == role_name:
            result = actor
            break
    return result


def wait_vehicle_by_role(world: Any, role_name: str, timeout_sec: float) -> Optional[Any]:
    deadline = time.monotonic() + max(timeout_sec, 0.1)
    result = find_vehicle_by_role(world, role_name)
    while result is None and time.monotonic() < deadline and not _SHOULD_STOP:
        wait_for_world_frame(world, 0.5)
        time.sleep(0.1)
        result = find_vehicle_by_role(world, role_name)
    return result


def wait_for_world_frame(world: Any, timeout_sec: float) -> None:
    try:
        world.wait_for_tick(timeout_sec)
    except RuntimeError:
        time.sleep(min(timeout_sec, 0.05))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle_rad(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad = angle_rad - 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad = angle_rad + 2.0 * math.pi
    return angle_rad


def speed_mps(actor: Any) -> float:
    velocity = actor.get_velocity()
    return math.sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y) + (velocity.z * velocity.z))


def choose_forward_waypoint(candidates: List[Any], current_yaw_deg: float) -> Optional[Any]:
    if not candidates:
        return None
    current_yaw_rad = math.radians(current_yaw_deg)
    best = candidates[0]
    best_error = float("inf")
    for candidate in candidates:
        candidate_yaw_rad = math.radians(candidate.transform.rotation.yaw)
        error = abs(normalize_angle_rad(candidate_yaw_rad - current_yaw_rad))
        if error < best_error:
            best = candidate
            best_error = error
    return best


def compute_steer(carla: Any, actor: Any, world_map: Any, lookahead_m: float, steer_kp: float, steer_kd: float,
                  max_steer: float, previous_error: float, dt: float) -> tuple[float, float, Optional[Any]]:
    transform = actor.get_transform()
    waypoint = world_map.get_waypoint(
        transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        return 0.0, previous_error, None

    candidates = waypoint.next(max(lookahead_m, 0.5))
    target_waypoint = choose_forward_waypoint(candidates, transform.rotation.yaw)
    if target_waypoint is None:
        return 0.0, previous_error, waypoint

    target_location = target_waypoint.transform.location
    dx = target_location.x - transform.location.x
    dy = target_location.y - transform.location.y
    target_angle = math.atan2(dy, dx)
    yaw_rad = math.radians(transform.rotation.yaw)
    heading_error = normalize_angle_rad(target_angle - yaw_rad)
    error_rate = (heading_error - previous_error) / max(dt, 1e-3)
    steer = (steer_kp * heading_error) + (steer_kd * error_rate)
    steer = clamp(steer, -max_steer, max_steer)
    return steer, heading_error, target_waypoint


def compute_longitudinal_control(current_speed: float, target_speed: float, dt: float,
                                 integral: float, previous_error: float, args: argparse.Namespace) -> tuple[float, float, float, float]:
    error = target_speed - current_speed
    integral = clamp(integral + error * dt, -args.speed_integral_limit, args.speed_integral_limit)
    error_rate = (error - previous_error) / max(dt, 1e-3)

    if error >= -args.speed_deadband_mps:
        command = (
            args.cruise_throttle
            + args.speed_kp * error
            + args.speed_ki * integral
            + args.speed_kd * error_rate
        )
        throttle = clamp(command, 0.0, args.max_throttle)
        brake = 0.0
    else:
        throttle = 0.0
        brake = clamp(args.brake_kp * (-error), 0.0, args.max_brake)

    return throttle, brake, integral, error


def disable_external_control(carla: Any, actor: Any) -> None:
    try:
        actor.disable_constant_velocity()
    except RuntimeError:
        pass
    try:
        actor.set_autopilot(False)
    except RuntimeError:
        pass
    try:
        actor.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    except RuntimeError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Waypoint PID controller for the ACC lead vehicle.")
    parser.add_argument("--carla-root", default="/home/aiden/snap/code/app/carla-package")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--lead-role-name", default="gaasd_lead")
    parser.add_argument("--target-speed-mps", type=float, default=2.0)
    parser.add_argument("--lookahead-m", type=float, default=6.0)
    parser.add_argument("--steer-kp", type=float, default=1.8)
    parser.add_argument("--steer-kd", type=float, default=0.08)
    parser.add_argument("--max-steer", type=float, default=0.45)
    parser.add_argument("--speed-kp", type=float, default=0.22)
    parser.add_argument("--speed-ki", type=float, default=0.03)
    parser.add_argument("--speed-kd", type=float, default=0.02)
    parser.add_argument("--speed-integral-limit", type=float, default=3.0)
    parser.add_argument("--speed-deadband-mps", type=float, default=0.05)
    parser.add_argument("--brake-kp", type=float, default=0.35)
    parser.add_argument("--cruise-throttle", type=float, default=0.12)
    parser.add_argument("--max-throttle", type=float, default=0.35)
    parser.add_argument("--max-brake", type=float, default=0.45)
    parser.add_argument("--duration-sec", type=float, default=0.0, help="0 means run until killed.")
    parser.add_argument("--status-interval-sec", type=float, default=2.0)
    args = parser.parse_args()

    if args.target_speed_mps < 0.0:
        raise SystemExit("[LeadWaypointPID] target speed must be non-negative")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    add_carla_python_paths(args.carla_root)
    import carla  # type: ignore

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout_sec)
    world = client.get_world()
    world_map = world.get_map()

    lead = wait_vehicle_by_role(world, args.lead_role_name, args.timeout_sec)
    if lead is None:
        raise SystemExit(f"[LeadWaypointPID] lead role_name={args.lead_role_name} not found")

    disable_external_control(carla, lead)
    print(
        "[LeadWaypointPID] started "
        f"lead_id={lead.id} target_speed={args.target_speed_mps:.2f}m/s "
        f"lookahead={args.lookahead_m:.2f}m"
    )
    sys.stdout.flush()

    start_time = time.monotonic()
    previous_time = start_time
    previous_steer_error = 0.0
    previous_speed_error = 0.0
    speed_integral = 0.0
    next_status_time = start_time

    while not _SHOULD_STOP:
        now = time.monotonic()
        if args.duration_sec > 0.0 and now - start_time >= args.duration_sec:
            break

        wait_for_world_frame(world, 1.0)
        now = time.monotonic()
        dt = clamp(now - previous_time, 0.01, 0.2)
        previous_time = now

        if not lead.is_alive:
            lead = wait_vehicle_by_role(world, args.lead_role_name, args.timeout_sec)
            if lead is None:
                raise SystemExit(f"[LeadWaypointPID] lead role_name={args.lead_role_name} disappeared")
            disable_external_control(carla, lead)

        current_speed = speed_mps(lead)
        steer, previous_steer_error, target_waypoint = compute_steer(
            carla,
            lead,
            world_map,
            args.lookahead_m,
            args.steer_kp,
            args.steer_kd,
            args.max_steer,
            previous_steer_error,
            dt,
        )
        throttle, brake, speed_integral, previous_speed_error = compute_longitudinal_control(
            current_speed,
            args.target_speed_mps,
            dt,
            speed_integral,
            previous_speed_error,
            args,
        )

        control = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
        lead.apply_control(control)

        if now >= next_status_time:
            waypoint_info = "unknown"
            if target_waypoint is not None:
                waypoint_info = f"road={target_waypoint.road_id} lane={target_waypoint.lane_id}"
            print(
                "[LeadWaypointPID] "
                f"speed={current_speed:.2f}/{args.target_speed_mps:.2f}m/s "
                f"throttle={throttle:.3f} brake={brake:.3f} steer={steer:.3f} "
                f"{waypoint_info}"
            )
            sys.stdout.flush()
            next_status_time = now + max(args.status_interval_sec, 0.5)

    lead.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    print("[LeadWaypointPID] stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
