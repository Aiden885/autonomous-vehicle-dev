#!/usr/bin/env python3
"""APA-specific CARLA <-> Pangu ZMQ bridge."""

from __future__ import annotations

import argparse
import copy
import glob
import hashlib
import io
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import weakref
import uuid
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import zmq
from PIL import Image

from apa_parking_map import (PARKING_MAP_PRESETS, OpenDriveParkingMap,
                             convex_polygons_overlap, parking_map_preset)
from apa_obstacle_tiles import StaticObstacleIndex
from apa_scene_compaction import StaticSceneCompactor
from apa_scene_diagnostics import SceneDiagnostics, scene_contract
from apa_scene_limits import SCENE_MAX_OBSTACLES
from apa_boundary_policy import (build_boundary_inputs, classified_map_lines,
                                 line_intersects_bounds, offline_collision_parts)
from apa_slot_geometry import validate_slot_geometry
from apa_parkin_goal import GoalBlocked, centered_goal, parallel_centered_goal, fit_goal, goal_tolerance, parking_footprint
from apa_offline_map import OfflineParkingMap, read_map, inventory_digest, body_geometry_digest, parking_region_guards, tile_cells
from apa_direct_actuation import validate_actuator
from apa_steering_observation import SteeringObservation, empty_steering_observation
from apa_steering_geometry import validate_steering_geometry, equivalent_to_carla, carla_to_equivalent

PROTOCOL = "gaasd_carla_bridge"
PROTOCOL_VERSION = "0.13.0"
SCENE_RADIUS_M = 30.0
OBSTACLE_PREDICTION_SEC = 1.5
SCENE_SURFACE_LABEL_NAMES = ("Roads", "RoadLines", "Sidewalks", "Ground", "Terrain", "Sky", "Water", "RailTrack")
SCENE_RAISED_SURFACE_LABEL_NAMES = ("Sidewalks", "Terrain")
TOPIC_PREFIX = "gaasd.carla"
APA_PLAN_REQUEST_TOPIC = f"{TOPIC_PREFIX}.apa_plan_request.v1"
APA_PLAN_RESULT_TOPIC = f"{TOPIC_PREFIX}.apa_plan_result.v1"
EGO_STATE_TOPIC = f"{TOPIC_PREFIX}.ego_state.v1"
CONTROL_TOPIC = f"{TOPIC_PREFIX}.control_cmd.v1"
APA_COMMAND_TOPIC = f"{TOPIC_PREFIX}.apa_command.v1"
APA_STATUS_TOPIC = f"{TOPIC_PREFIX}.apa_status.v1"
DRIVER_VEHICLE_TOPIC = f"{TOPIC_PREFIX}.driver_vehicle_cmd.v1"
APA_PHASES = ("manual", "search", "park", "complete")
CALIBRATION_BRIDGE_DIR = Path(__file__).resolve().parent
DEFAULT_VEHICLE_PROFILE_PATH = (CALIBRATION_BRIDGE_DIR.parents[3] /
    "pangu_gaasd_base/modules/apa/apa/conf/apa_vehicle_profile.json")
REAR_AXLE_REFERENCE_POINT = "rear_axle_center"
LOW_OBSTACLE_PROFILE_VERSION = 10
LOW_OBSTACLE_TILE_SIZE_M = 32.0


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def read_vehicle_profile(path: Path) -> Dict[str, Any]:
    raw = path.read_bytes()
    profile = json.loads(raw)
    for key in ("width_m", "wheelbase_m", "rear_to_front_m", "rear_to_back_m",
                "rear_axle_offset_m", "max_steer_rad", "planning_min_turn_radius_m",
                "parkin_margin_m", "parkin_physical_margin_m", "parkin_back_margin_m",
                "goal_position_tolerance_m", "goal_yaw_tolerance_rad"):
        value = profile.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"vehicle profile {key} must be finite and positive")
    if profile["parkin_back_margin_m"] > profile["parkin_margin_m"]:
        raise ValueError("parkin_back_margin_m must not exceed parkin_margin_m")
    if profile["parkin_physical_margin_m"] < profile["parkin_margin_m"]:
        raise ValueError("parkin_physical_margin_m must be at least parkin_margin_m")
    for key in ("front_chamfer_lon_m", "front_chamfer_lat_m", "rear_chamfer_lon_m", "rear_chamfer_lat_m"):
        value = profile.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f"vehicle profile {key} must be finite and non-negative")
    if (profile["front_chamfer_lon_m"] >= profile["rear_to_front_m"] - profile["wheelbase_m"] or
            profile["rear_chamfer_lon_m"] >= profile["rear_to_back_m"] or
            profile["front_chamfer_lat_m"] >= profile["width_m"] * 0.45 or
            profile["rear_chamfer_lat_m"] >= profile["width_m"] * 0.45):
        raise ValueError("invalid vehicle corner chamfer dimensions")
    for key in ("blueprint", "carla_version", "dimension_source"):
        if not isinstance(profile.get(key), str) or not profile[key]:
            raise ValueError(f"vehicle profile {key} must be a nonempty string")
    if profile["rear_to_front_m"] <= profile["wheelbase_m"] or profile["max_steer_rad"] >= math.pi / 2:
        raise ValueError("invalid vehicle front overhang or steering lock")
    validate_steering_geometry(profile)
    profile["length_m"] = round(profile["rear_to_front_m"] + profile["rear_to_back_m"], 8)
    profile["front_overhang_m"] = round(profile["rear_to_front_m"] - profile["wheelbase_m"], 8)
    profile["profile_id"] = hashlib.sha256(raw).hexdigest()
    return profile


def default_config(profile_path: Optional[Path] = None) -> Dict[str, Any]:
    profile_path = Path(profile_path or DEFAULT_VEHICLE_PROFILE_PATH).resolve()
    profile = read_vehicle_profile(profile_path)
    return {
        "carla": {
            "host": "127.0.0.1",
            "port": 2000,
            "timeout_sec": 30.0,
            "runtime_rpc_timeout_sec": 2.0,
            "root": "/opt/carla",
            "python_api_paths": ["/opt/carla/PythonAPI/carla"],
            "sync_mode": True,
            "fixed_delta_seconds": 0.02,
            "realtime_pacing": True,
            "map_name": "Town04",
            "ego_role_name": "hero",
            "auto_spawn_ego": True,
            "ego_blueprint_filter": profile["blueprint"],
        },
        "timing": {
            "control_period_sec": 0.02,
            "control_wait_timeout_sec": 1.0,
            "feedback_retry_sec": 0.1,
        },
        "safety": {
            "standstill_speed_mps": 0.02,
            "standstill_dwell_sec": 0.20,
            "brake_timeout_sec": 3.0,
            "rpc_timeout_sec": 0.5,
        },
        "zmq": {
            "publish_bind": "tcp://127.0.0.1:5701",
            "control_connect": "tcp://127.0.0.1:5702",
            "command_bind": "tcp://127.0.0.1:5703",
        },
        "apa": {
            "slot_file": "",
            "slot_id": 1,
            "plan_request_period_sec": 0.2,
            "vehicle_profile_path": str(profile_path),
            "vehicle_profile_id": profile["profile_id"],
            "rear_axle_offset_m": profile["rear_axle_offset_m"],
            "vehicle_geometry": dict(profile),
            "draw_debug": False,
            "spawn_occupied_vehicles": False,
            "parking_query_radius_m": 24.0,
            "max_parked_vehicles": 12,
            "approach_distance_m": 4.5,
            "follow_spectator": True,
            "spectator_back_m": 8.0,
            "spectator_up_m": 6.0,
            "spectator_pitch_deg": -28.0,
            "drivetrain_clutch_strength": None,
            "parking_action": 1,
            "camera_enabled": True,
            "camera_http_host": "127.0.0.1",
            "camera_http_port": 5704,
            "camera_width": 480,
            "camera_height": 270,
            "camera_fov": 100.0,
            "camera_fps": 8.0,
            "camera_x": -2.45,
            "camera_z": 1.05,
            "camera_pitch_deg": -16.0,
            "overhead_camera_enabled": True,
            "overhead_camera_fov": 90.0,
            "overhead_camera_z": 18.0,
            "trajectory_draw_hz": 5.0,
            "spectator_hz": 8.0,
        },
        "control": {
            "actuation": "c_pedals_v1",
            "max_accel_mps2": 1.5,
            "max_abs_target_speed_mps": 1.1,
            "max_abs_steer_rad": profile["max_steer_rad"],
            "manual_pedal_slew_per_sec": 1.6,
            "calibration_record_dir": "",
        },
    }


def add_carla_python_paths(cfg: Dict[str, Any]) -> None:
    paths = list(cfg["carla"].get("python_api_paths", []))
    root = str(cfg["carla"].get("root", ""))
    if root:
        paths.append(os.path.join(root, "PythonAPI", "carla"))
        paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
    for path in reversed(paths):
        if path and os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def import_carla(cfg: Dict[str, Any]):
    add_carla_python_paths(cfg)
    try:
        import carla  # type: ignore
    except ImportError as exc:
        raise SystemExit("Cannot import CARLA Python API. Check carla.root/python_api_paths.") from exc
    return carla


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if math.isfinite(number):
        return number
    return default


def required_number(value: Any, name: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} 必须是有限数值") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数值")
    return number


def convex_hull_xy(points):
    points = sorted(set(points))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

    def chain(items):
        result = []
        for point in items:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 1e-8:
                result.pop()
            result.append(point)
        return result

    return chain(points)[:-1] + chain(reversed(points))[:-1]


def obstacle_from_vertices(identifier: str, vertices: List[Any], vx: float = 0.0, vy: float = 0.0) -> Dict[str, Any]:
    """Keep the 3D box until the consumer's vehicle height is known."""
    xyz = [(required_number(p.x, "obstacle.x"), required_number(p.y, "obstacle.y"),
            required_number(p.z, "obstacle.z")) for p in vertices]
    if not xyz:
        raise ValueError(f"{identifier} 障碍包络为空")
    hull = convex_hull_xy((x, y) for x, y, _ in xyz)
    if len(hull) < 3:

        pad = 0.01
        left, right = min(p[0] for p in xyz)-pad, max(p[0] for p in xyz)+pad
        bottom, top = min(p[1] for p in xyz)-pad, max(p[1] for p in xyz)+pad
        hull = [(left, bottom), (right, bottom), (right, top), (left, top)]
    return {"id": identifier, "footprint": [{"x": x, "y": y} for x, y in hull],
            "velocity_x": required_number(vx, "obstacle.velocity_x"),
            "velocity_y": required_number(vy, "obstacle.velocity_y"),
            "z_min": min(z for _, _, z in xyz), "z_max": max(z for _, _, z in xyz),
            "_xy_bounds": (min(x for x, _ in hull), min(y for _, y in hull),
                           max(x for x, _ in hull), max(y for _, y in hull)),
            "_vertices": tuple(xyz)}


def environment_obstacle_sources(world, carla):
    """World-space static boxes, with bodies kept separate from sampled map geometry."""
    labels = carla.CityObjectLabel
    raised_labels = {getattr(labels, name) for name in SCENE_RAISED_SURFACE_LABEL_NAMES}
    surface_labels = {getattr(labels, name) for name in SCENE_SURFACE_LABEL_NAMES} - raised_labels
    body_labels = {labels.Car, labels.Bus, labels.Truck, labels.Motorcycle,
                   labels.Bicycle, labels.Rider, labels.Pedestrians, labels.Train}
    identity = carla.Transform()

    obstacles = []
    for obj in world.get_environment_objects(labels.Any):
        if obj.type in surface_labels:
            continue

        obstacle = obstacle_from_vertices(f"map:{obj.id}", obj.bounding_box.get_world_vertices(identity))
        obstacle["_semantic_labels"] = frozenset((str(obj.type),))
        obstacle["_profile_eligible"] = obj.type not in body_labels
        obstacle["_raised_surface"] = obj.type in raised_labels
        if obj.type in body_labels:
            obstacle['_asset_name'] = str(obj.name)
            obstacle['_body_geometry_id'] = body_geometry_digest(obstacle)
        obstacles.append(obstacle)
    return obstacles


def obstacle_needs_profile(obstacle, vehicle_z_max):
    if not obstacle.get("_profile_eligible", False):
        return False
    left, bottom, right, top = obstacle["_xy_bounds"]

    return (obstacle.get("_raised_surface", False) or
            obstacle["z_max"] > vehicle_z_max+0.25 or
            max(right-left, top-bottom) > LOW_OBSTACLE_TILE_SIZE_M)


def vehicle_obstacle_height_band(vertices):
    """World-Z occupancy of the actual body, including pitch, roll and ground height."""
    heights = [required_number(point.z, "vehicle.vertex.z") for point in vertices]
    if not heights or min(heights) >= max(heights):
        raise ValueError("车辆三维包络高度无效，不能生成障碍高度带")
    z_min, z_max = min(heights), max(heights)

    return z_min, z_max, (math.floor(z_min*4)/4-0.25, math.ceil(z_max*4)/4+0.25)


def clip_xy_polygon(points, bounds, max_vertices=8):
    """Intersect a convex footprint with an axis-aligned map region."""
    for axis, edge, sign in (("x", bounds[0], 1), ("x", bounds[2], -1),
                             ("y", bounds[1], 1), ("y", bounds[3], -1)):
        points = clip_xy_half_plane(points, axis, edge, sign, max_vertices)
    return points


def clip_xy_half_plane(points, axis, edge, sign, max_vertices=8):
    if not points or all(sign*(p[axis]-edge) <= 0 for p in points):
        return []
    if all(sign*(p[axis]-edge) >= 0 for p in points):
        return points
    result = []
    for a, b in zip(points, points[1:]+points[:1]):
        da, db = sign*(a[axis]-edge), sign*(b[axis]-edge)
        if da >= 0:
            result.append(a)
        if (da < 0 < db) or (db < 0 < da):
            t = da/(da-db)
            result.append({key: a[key]+t*(b[key]-a[key]) for key in ("x", "y")})
    hull = convex_hull_xy((p["x"], p["y"]) for p in result)
    if len(hull) < 3:
        return []
    if max_vertices is not None and len(hull) > max_vertices:

        xs, ys = [p[0] for p in hull], [p[1] for p in hull]
        hull = [(min(xs), min(ys)), (max(xs), min(ys)),
                (max(xs), max(ys)), (min(xs), max(ys))]
    return [{"x": x, "y": y} for x, y in hull]


def obstacle_at_height(obstacle: Dict[str, Any], z_min: float, z_max: float) -> List[Dict[str, Any]]:
    """Intersect a live obstacle's 3D box with the requested world-Z slab."""
    if obstacle["z_max"] < z_min or obstacle["z_min"] > z_max:
        return []
    clean = {key: value for key, value in obstacle.items() if not key.startswith("_")}
    xyz = obstacle.get("_vertices", ())
    if not xyz:
        return [clean]
    points = [(x, y) for x, y, z in xyz if z_min <= z <= z_max]

    for i, a in enumerate(xyz):
        for b in xyz[i+1:]:
            if abs(b[2]-a[2]) < 1e-10:
                continue
            for z in (z_min, z_max):
                t = (z-a[2])/(b[2]-a[2])
                if 0.0 <= t <= 1.0:
                    points.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
    hull = convex_hull_xy(points)

    if 3 <= len(hull) <= 8:
        clean["footprint"] = [{"x": x, "y": y} for x, y in hull]
    return [clean]


class InvalidEgoFeedback(ValueError):
    """Measured feedback failed validation after braking was already latched."""


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def offset_reference_xy(x: float, y: float, yaw_rad: float, forward_offset_m: float) -> Tuple[float, float]:
    """Shift a CARLA pose along heading; actor to rear axle uses negative rear_axle_offset_m."""
    return x + forward_offset_m * math.cos(yaw_rad), y + forward_offset_m * math.sin(yaw_rad)


def carla_steer_command(steer_rad: float, max_abs_steer_rad: float,
                       vehicle_geometry: Optional[Dict[str, Any]] = None) -> float:
    """Equivalent bicycle radians to CARLA normalized inner-wheel command."""
    limit = max(finite(max_abs_steer_rad, math.radians(45.0)), 1.0e-6)
    return equivalent_to_carla(finite(steer_rad), limit, vehicle_geometry or {})


def read_front_wheel_rad(payload: Dict[str, Any]) -> float:
    """Read the equivalent bicycle angle in rad; steering_wheel_angle input uses degrees."""
    target = payload.get("target", {})
    if not isinstance(target, dict):
        target = {}
    for source in (payload, target):
        if any(key in source for key in ("steering_wheel_angle", "SteeringWheelAngle")):
            return math.radians(read_number(source, "steering_wheel_angle", "SteeringWheelAngle", default=0.0))
    return read_number(
        payload,
        "FrontWheelAngle",
        "front_wheel_angle",
        default=read_number(target, "steer_rad", default=0.0),
    )


def manual_steer_command(payload: Dict[str, Any], max_abs_steer_rad: float,
                         vehicle_geometry: Optional[Dict[str, Any]] = None) -> float:
    if any(key in payload for key in ("steer_rad", "front_wheel_angle", "FrontWheelAngle")):
        steer_rad = read_number(payload, "steer_rad", "front_wheel_angle", "FrontWheelAngle", default=0.0)
        return carla_steer_command(steer_rad, max_abs_steer_rad, vehicle_geometry)
    return clamp(finite(payload.get("steer", 0.0)), -1.0, 1.0)


def yaw_rad_from_carla_deg(yaw_deg: float) -> float:
    return math.radians(float(yaw_deg))


def signed_longitudinal_speed(velocity: Any, yaw_rad: float) -> float:
    return finite(velocity.x) * math.cos(yaw_rad) + finite(velocity.y) * math.sin(yaw_rad)


def yaw_deg_from_vector(x: float, y: float) -> float:
    return math.degrees(math.atan2(y, x))


def wrap_yaw_deg(yaw: float) -> float:
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw


def slot_kind_code(kind: str) -> int:
    normalized = str(kind or "").lower()
    if normalized in ("vertical", "perpendicular"):
        return 1
    if normalized == "parallel":
        return 2
    if normalized in ("diagonal", "oblique", "slanted"):
        return 3
    if normalized in ("vh", "vertical_head", "vertical_head_in"):
        return 4
    return 0


def apa_slot_kind_code(slot: Dict[str, Any]) -> int:
    explicit = slot.get("apa_slot_kind", slot.get("slot_kind", ""))
    return slot_kind_code(str(explicit))


def physical_slot_id(slot: Dict[str, Any]) -> int:
    return int(slot.get("physical_slot_id", slot["slot_id"]))


def slot_choice_identity(slot: Dict[str, Any]) -> Dict[str, Any]:
    kind = apa_slot_kind_code(slot)
    return {"slot_id": int(slot["slot_id"]), "physical_slot_id": physical_slot_id(slot),
            "kind": str(slot.get("slot_kind", "")),
            "parking_direction": "head_in" if kind == 4 else "reverse_in" if kind in (1, 3) else "parallel"}


def apa_default_parking_type(slot: Dict[str, Any]) -> int:
    if "parking_type" in slot:
        return int(slot.get("parking_type", 0))

    return 3


def apa_default_surrounding_type(slot: Dict[str, Any]) -> int:
    if "surrounding_type" in slot:
        return int(slot.get("surrounding_type", 0))
    if apa_slot_kind_code(slot) == 2:

        return 4

    return 3


def read_number(payload: Dict[str, Any], *keys: str, default: float = 0.0) -> float:
    for key in keys:
        if key in payload:
            return finite(payload.get(key), default)
    return default


def read_bool(payload: Dict[str, Any], *keys: str, default: bool = False) -> bool:
    for key in keys:
        if key in payload:
            value = payload.get(key)
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)):
                return value != 0
            if isinstance(value, str):
                return value.lower() in ("true", "1", "yes", "on")
    return default


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else 0.0
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


class ParkingSlots:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.data = load_json(str(path))
        validate_slot_geometry(self.data)
        rows = self.data.get("slots", self.data)
        self.slots = {int(row["slot_id"]): row for row in rows}
        for slot in self.slots.values():
            if "physical_slot_id" not in slot:
                continue
            physical = slot["physical_slot_id"]
            base = self.slots.get(physical) if type(physical) is int else None
            base_kind = apa_slot_kind_code(base) if base is not None else 0
            variant_kind = apa_slot_kind_code(slot)
            opposite_direction = ((base_kind in (1, 3) and variant_kind == 4) or
                                  (base_kind == 4 and variant_kind in (1, 3)))
            fields = [f"slot_pt{i}_{axis}" for i in range(1, 5) for axis in ("x", "y")]
            fields += ["slot_kind", "slot_entry", "slot_entry_pt_x", "slot_entry_pt_y",
                       "slot_vertex_ids", "slot_edge_ids", "native_parking",
                       "apa_start_rear_x", "apa_start_rear_y", "apa_start_rear_yaw_rad"]
            if (base is None or "physical_slot_id" in base or not opposite_direction or
                    any(slot.get(key) != base.get(key) for key in fields)):
                raise ValueError("方向目标须引用同一实体车位的边界、入口和起点，且泊入方向相反")
        self.obstacles = self.data.get("obstacles", {}) if isinstance(self.data, dict) else {}
        classified_map_lines(self.obstacles)
        self.native_map = None
        self.occupancy = {}
        self.map_summary = {"source": "calibration", "native_records": 0, "matched_slots": 0}
        if not self.slots:
            raise RuntimeError(f"no parking slots in {path}")

    def physical_slots(self):
        return (slot for slot in self.slots.values() if "physical_slot_id" not in slot)

    def get(self, slot_id: int) -> Dict[str, Any]:
        if slot_id in self.slots:
            return self.slots[slot_id]
        for row in self.slots.values():
            if not bool(row.get("slot_occupied", False)):
                return row
        return next(iter(self.slots.values()))

    def bind_map(self, carla_map):
        """Bind the server XML; native parking tasks require an entrance and approach pose."""
        expected = str(self.data.get("map", "")).rsplit("/", 1)[-1]
        if expected and expected not in carla_map.name:
            raise ValueError("车位文件与当前加载地图不一致")
        self.native_map = OpenDriveParkingMap(carla_map.to_opendrive())
        validate_slot_geometry(self.data, self.native_map.sha256)
        matched = 0
        for slot in self.slots.values():

            if slot.get("geometry_source") == "surveyed_shared":
                vertices = self.data["slot_geometry"]["vertices"]
                slot["_surveyed_ground_z"] = sum(vertices[v]["z"] for v in slot["slot_vertex_ids"])/4
            record = self.native_map.match(slot)
            if record is None:
                if slot.get("geometry_source") == "opendrive" or slot.get("native_parking"):
                    raise ValueError(f'车位 {slot["slot_id"]} 没有唯一的原生地图来源')
                continue
            calibrated = all(f"slot_pt{i}_{axis}" in slot for i in range(1, 5) for axis in ("x", "y"))
            if calibrated:
                x = sum(required_number(slot[f"slot_pt{i}_x"], "slot.x") for i in range(1, 5))/4
                y = sum(required_number(slot[f"slot_pt{i}_y"], "slot.y") for i in range(1, 5))/4
                difference = math.hypot(record["x"]-x, record["y"]-y)
                if difference > 0.75:
                    raise ValueError(f'车位 {slot["slot_id"]} 原生位置与标定相差过大')
            else:
                difference = None
            slot["_native_geometry"] = record
            slot["_native_centre_difference_m"] = difference
            if slot.get("geometry_source") == "opendrive":
                if not record["rectangle_supported"]:
                    raise ValueError("原生车位存在未支持的轮廓或横断面，须使用标定边界")
                if (type(slot.get("slot_entry")) is not int or slot["slot_entry"] not in (1, 2, 3, 4)
                        or not slot.get("slot_kind") or not all(key in slot for key in
                            ("apa_start_rear_x", "apa_start_rear_y", "apa_start_rear_yaw_rad"))):
                    raise ValueError("原生车位须配置入口边、车位类型和初始后轴位姿")
                for index, corner in enumerate(record["corners"], 1):
                    slot.update({f"slot_pt{index}_x": corner["x"], f"slot_pt{index}_y": corner["y"]})
                if slot.get("goal_reference_point") == REAR_AXLE_REFERENCE_POINT and not all(
                        key in slot for key in ("goal_x", "goal_y", "goal_yaw")):
                    raise ValueError("显式后轴目标须提供完整位置和航向")
                slot.setdefault("goal_x", record["x"])
                slot.setdefault("goal_y", record["y"])
                slot.setdefault("goal_yaw", math.degrees(record["object_yaw_rad"]))
                edge = slot["slot_entry"]-1
                first, second = record["corners"][edge], record["corners"][(edge+1) % 4]
                slot.update(slot_entry_pt_x=(first["x"]+second["x"])/2,
                            slot_entry_pt_y=(first["y"]+second["y"])/2)
            elif not calibrated:
                raise ValueError("标定车位缺少完整边界")
            matched += 1
        self.occupancy.clear()
        self.map_summary = {"source": "opendrive_with_calibration" if matched else "calibration",
                            "map": carla_map.name, "opendrive_sha256": self.native_map.sha256,
                            "native_records": self.native_map.raw_count,
                            "native_rectangles": len(self.native_map.records),
                            "rejected_records": len(self.native_map.rejected), "matched_slots": matched}

    def availability(self, slot):
        state = self.occupancy.get(int(slot["slot_id"]), {"state": "unknown", "frame": None})
        native = slot.get("_native_geometry")
        return {"occupied": True if state["state"] == "occupied" else False if state["state"] == "free" else None,
                "occupancy_state": state["state"], "occupancy_frame": state["frame"],
                "occupancy_reason": state.get("reason", "not_checked"),
                "occupancy_obstacle_id": state.get("obstacle_id", ""),
                "occupancy_edge_state": state.get("edge_state", "free"),
                "occupancy_edge_obstacle_id": state.get("edge_obstacle_id", ""),
                "occupancy_static_frame": state.get("static_frame"),
                "configured_occupied": bool(slot.get("slot_occupied", False)),
                "geometry_source": slot.get("geometry_source", "calibrated"),
                "native_source": {"road_id": native["road_id"], "object_ids": native["object_ids"],
                                  "centre_difference_m": slot.get("_native_centre_difference_m")} if native else None}


def encode_camera_frame(width: int, height: int, bgra: bytes) -> Tuple[bytes, str]:
    if width <= 0 or height <= 0 or len(bgra) < width * height * 4:
        raise ValueError("camera frame is shorter than width*height*4")
    buffer = io.BytesIO()
    Image.frombytes("RGB", (width, height), bgra, "raw", "BGRX").save(
        buffer, format="JPEG", quality=60
    )
    return buffer.getvalue(), "image/jpeg"


class RearCameraHandler(BaseHTTPRequestHandler):
    bridge: Optional["ApaCarlaBridge"] = None

    def log_message(self, format: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        owner = self.bridge
        view = "overhead" if path == "/overhead.jpg" else "rear"
        if path not in ("/", "/rear", "/rear.jpg", "/rear.png", "/stream", "/overhead.jpg"):
            self.send_error(404)
            return
        payload, mime, received_at, revision = owner.latest_camera_frame(view) if owner is not None else (b"", "image/jpeg", 0.0, 0)
        if not payload:
            self.send_error(503, "Camera frame not ready")
            return
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Camera-Session", owner.session_id)
        self.send_header("X-Camera-Revision", str(revision))
        self.send_header("X-Camera-Age-Sec", str(max(0.0, time.monotonic() - received_at)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass


def extract_plan_path_xy(plan: Dict[str, Any], max_points: int = 80) -> List[Tuple[float, float]]:
    if not isinstance(plan, dict):
        return []
    points: List[Tuple[float, float]] = []
    raw_path = plan.get("path")
    if isinstance(raw_path, list):
        for item in raw_path:
            if isinstance(item, dict):
                points.append((finite(item.get("x", item.get("X"))), finite(item.get("y", item.get("Y")))))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                points.append((finite(item[0]), finite(item[1])))
    if not points:
        xs = plan.get("path_point_x", plan.get("PthPtX", []))
        ys = plan.get("path_point_y", plan.get("PthPtY", []))
        if isinstance(xs, list) and isinstance(ys, list):
            count = min(len(xs), len(ys))
            points = [(finite(xs[index]), finite(ys[index])) for index in range(count)]
    if max_points > 1 and len(points) > max_points:
        last = len(points) - 1
        points = [points[int(round(index * last / (max_points - 1)))] for index in range(max_points)]
    return points


class SafetyStop:
    """A latched stop; only distinct, continuous physics frames prove standstill."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.lock = threading.RLock()
        self.speed_limit = float(cfg["standstill_speed_mps"])
        self.dwell = float(cfg["standstill_dwell_sec"])
        self.timeout = float(cfg["brake_timeout_sec"])
        self.rpc_timeout = float(cfg["rpc_timeout_sec"])
        if not all(math.isfinite(v) and v > 0.0 for v in (
                self.speed_limit, self.dwell, self.timeout, self.rpc_timeout)):
            raise ValueError("APA safety values must be finite and positive")
        if self.speed_limit > 0.05 or self.dwell < 0.20 or self.rpc_timeout > 1.0 or self.timeout > 4.0:
            raise ValueError("APA safety requires speed <=0.05, dwell >=0.20, RPC <=1 and braking <=4 seconds")
        if self.timeout <= self.dwell or self.rpc_timeout >= self.timeout:
            raise ValueError("APA braking budget must exceed dwell and RPC timeout")
        self.event_id = 0
        self.revision = 0
        self.reason = ""
        self.state = "none"
        self.detail = ""
        self.standstill_sec = 0.0
        self.deadline = 0.0
        self._last_frame = None
        self._last_time = None
        self._stopped_since = None

    def _transition(self, state: str, detail: str = "") -> None:
        if self.state != state or self.detail != detail:
            self.state, self.detail = state, detail
            print(f"[APA safety] event={self.event_id} state={state} reason={self.reason} detail={detail}", flush=True)

    def latch(self, reason: str) -> bool:
        with self.lock:
            self.revision += 1
            if self.reason:

                if self.reason in ("cancelled", "shutdown") and reason not in ("cancelled", "shutdown"):
                    self.reason = str(reason)[:512]
                    self.event_id += 1
                    if self.state == "stopped":
                        self.deadline = time.monotonic() + self.timeout
                        self._stopped_since = None
                        self.standstill_sec = 0.0
                        self._transition("braking")
                    return True
                return False
            self.event_id += 1
            self.reason = str(reason)[:512]
            self.deadline = time.monotonic() + self.timeout
            self._last_frame = self._last_time = self._stopped_since = None
            self.standstill_sec = 0.0
            self._transition("braking")
            return True

    def unconfirmed(self, detail: str) -> None:
        with self.lock:
            self.standstill_sec = 0.0
            self._last_frame = self._last_time = self._stopped_since = None
            self._transition("unconfirmed", detail)

    def observe(self, frame: int, sim_time: float, speed: float, step: float) -> None:
        with self.lock:
            if not self.reason:
                return
            if (type(frame) is not int or frame <= 0 or
                    not all(math.isfinite(v) for v in (sim_time, speed, step)) or
                    sim_time < 0.0 or speed < 0.0 or step <= 0.0):
                self.unconfirmed("invalid physical feedback")
                return
            if self.state == "braking" and time.monotonic() >= self.deadline:
                self.unconfirmed("braking deadline exceeded")
                return
            if self._last_frame is not None:
                if frame == self._last_frame and sim_time == self._last_time:
                    return
                if frame != self._last_frame + 1 or abs(sim_time - self._last_time - step) > 1.0e-5:
                    self.unconfirmed("physical feedback frame/time discontinuity")
                    return
            self._last_frame, self._last_time = frame, sim_time
            if speed <= self.speed_limit:
                if self._stopped_since is None:
                    self._stopped_since = sim_time
                self.standstill_sec = sim_time - self._stopped_since
                if self.standstill_sec + 1.0e-6 >= self.dwell:
                    self._transition("stopped")
            else:
                self.standstill_sec = 0.0
                self._stopped_since = None
                if self.state == "stopped":
                    self.deadline = time.monotonic() + self.timeout
                    self._transition("braking", "standstill lost")

    def payload(self) -> Dict[str, Any]:
        with self.lock:
            return {"event_id": self.event_id, "reason": self.reason,
                    "state": self.state, "standstill_sec": self.standstill_sec}

    def clear(self, expected_revision: Optional[int] = None) -> bool:
        with self.lock:
            if ((self.reason and self.state != "stopped") or
                    (expected_revision is not None and expected_revision != self.revision)):
                return False
            self.reason = ""
            self.standstill_sec = 0.0
            self._last_frame = self._last_time = self._stopped_since = None
            self._transition("none")
            return True


class ApaCarlaBridge:
    def __init__(self, cfg: Dict[str, Any], dry_run: bool = False) -> None:
        self.cfg = cfg
        profile_path = Path(cfg["apa"]["vehicle_profile_path"])
        if not profile_path.is_absolute():
            profile_path = Path(cfg.get("_config_dir", ".")) / profile_path
        profile = read_vehicle_profile(profile_path)
        cfg["apa"].update(vehicle_profile_path=str(profile_path.resolve()),
            vehicle_profile_id=profile["profile_id"], vehicle_geometry=profile,
            rear_axle_offset_m=profile["rear_axle_offset_m"])
        self.vehicle_profile_id = profile["profile_id"]
        self.validate_timing()
        self.validate_control_config()
        bridge_dir = Path(__file__).resolve().parent
        self.scene_diagnostics = SceneDiagnostics(
            bridge_dir.parents[1] / ".cache" / "apa_diagnostics", copy.deepcopy(cfg),
            [bridge_dir / name for name in ("carla_bridge_apa.py", "apa_offline_map.py",
                                           "apa_map_interior.py",
                                           "apa_scene_compaction.py", "apa_scene_diagnostics.py",
                                           "apa_boundary_policy.py", "apa_slot_geometry.py")])
        self.safety_stop = SafetyStop(cfg["safety"])
        self._recovery_config = dict(cfg["safety"])
        self.recovery_stop = SafetyStop(self._recovery_config)
        self._recovery_id = self._path_version = 0
        self._active_actuation_limits = None
        self._safety_tx_failed = False
        self.dry_run = dry_run
        self.session_id = uuid.uuid4().hex
        self.task_id = 0
        self._sync_task_key = None
        self._task_parkin_goal = None
        self._sync_fault = ""
        self._pending_control = None
        self._control_ack = False
        self._feedback_cache = None
        self._last_plan_sim_time = None
        self.running = True
        self.carla = None
        self.client = None
        self.world = None
        self.spectator = None
        self._map_name = ""
        self.ego = None
        self._ego_bounding_box = None
        self._ego_bounding_box_actor_id = None
        self.original_settings: Optional[Dict[str, Any]] = None
        self.environment_obstacles: List[Dict[str, Any]] = []
        self._obstacle_profile_summary = {}
        self._offline_parking_map = None
        self._slot_occupancy_cache = {}
        self._slot_occupancy_explicit = None
        self.calibration_recorder = None
        self._calibration_record_error = ""
        self._calibration_recording_path = ""
        self.sequence: Dict[str, int] = {}
        self._last_control_log_time = 0.0
        self._last_control_log_state = None
        self.last_algorithm_rx = 0.0
        self.last_control_id = 0
        self.current_apa_gear_pos = 1
        self.ignore_control_until = 0.0
        self.last_plan_result: Optional[Dict[str, Any]] = None
        self.parking_action = int(self.cfg["apa"].get("parking_action", 1))
        if self.parking_action not in (1, 2):
            self.parking_action = 1
        self.spawned_parked: List[Any] = []
        self.collision_sensor = None
        self.parking_enabled = False
        self.planning_only = False
        self.apa_phase = "manual"
        self.manual_gear = "D"
        self.manual_hand_brake = False
        self.manual_ignition = "ON"
        self.manual_throttle = 0.0
        self.manual_brake = 0.0
        self.manual_steer = 0.0
        self.applied_manual_throttle = 0.0
        self.applied_manual_brake = 0.0
        self.applied_manual_steer = 0.0
        self.manual_last_input = 0.0
        self.last_control_payload: Optional[Dict[str, Any]] = None
        self.last_mapped_throttle = 0.0
        self.last_mapped_brake = 0.0
        self.last_applied_gear = "D"
        self.actual_path_xy: deque = deque(maxlen=4096)
        self.accepted_plan_history: deque = deque(maxlen=8)
        self.last_accepted_plan_result: Optional[Dict[str, Any]] = None
        self.tracking_error_summary = {}
        self._tracking_error_frame = -1
        self._tracking_error_square_sum = 0.0
        self.rear_camera = None
        self.overhead_camera = None
        self.camera_client = None
        self.camera_lock = threading.Lock()
        self.camera_bytes = {view: b"" for view in ("rear", "overhead")}
        self.camera_mime = {view: "image/jpeg" for view in ("rear", "overhead")}
        self.camera_received_at = {view: 0.0 for view in ("rear", "overhead")}
        self.camera_http = None
        self.camera_http_url = ""
        self._camera_pending = {}
        self._camera_worker_started = set()
        self._camera_revision = 0
        self._camera_min_frame = -1
        self._overhead_camera_error = ""
        self.last_trajectory_draw = 0.0
        self.last_status_publish = 0.0

        self.context = zmq.Context.instance()
        self.pub = self.context.socket(zmq.PUB)
        self.sub = self.context.socket(zmq.SUB)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.command_sub = self.context.socket(zmq.SUB)
        self.command_sub.setsockopt_string(zmq.SUBSCRIBE, APA_COMMAND_TOPIC)
        self.command_sub.setsockopt_string(zmq.SUBSCRIBE, DRIVER_VEHICLE_TOPIC)

        slot_file = self.resolve_slot_file()
        self.slot_provider = ParkingSlots(slot_file)
        self.target_slot = self.slot_provider.get(int(self.cfg["apa"].get("slot_id", 1)))
        self.initial_pose = None
        self.initial_pose_mode = "slot"
        self.parkout_return_pose: Optional[Dict[str, Any]] = None
        self.history_revision = 0
        self.last_task_command: Dict[str, Any] = {}
        self._task_command_results: Dict[str, Dict[str, Any]] = {}
        self.pending_map_config = None
        self.pending_map_command = None
        self.map_bootstrap_command = None
        self.map_transition: Dict[str, Any] = {}
        self._map_providers: Dict[str, ParkingSlots] = {}
        self.map_choices = self.build_map_choices()

    def build_map_choices(self) -> List[Dict[str, Any]]:
        current = str(self.cfg["carla"].get("map_name", "")).rsplit("/", 1)[-1]
        presets = list(PARKING_MAP_PRESETS)
        if current and all(name != current for name, _, _ in presets):
            presets.append((current, self.slot_provider.path.name, int(self.target_slot["slot_id"])))
        choices = []
        for name, filename, default_id in presets:
            choice = {"map_name": name, "available": False, "reason": "车位数据不可用", "slot_choices": []}
            try:
                provider = self.slot_provider if name == current else ParkingSlots(self.slot_provider.path.parent / filename)
                self._map_providers[name] = provider
                choice.update(available=True, reason="", default_slot_id=int(provider.get(default_id)["slot_id"]),
                    slot_choices=[{**slot_choice_identity(slot),
                                   **provider.availability(slot),
                                   "default_pose": self.approach_pose(slot)}
                                  for slot in sorted(provider.slots.values(), key=lambda row: int(row["slot_id"]))])
            except (OSError, ValueError, TypeError, KeyError, RuntimeError) as exc:
                print(f"[APA maps] {name}: {exc}", flush=True)
            choices.append(choice)
        return choices

    def resolve_scene_setup(self, payload: Dict[str, Any], provider: ParkingSlots):
        slot_id = payload.get("slot_id")
        if type(slot_id) is not int or slot_id not in provider.slots:
            raise ValueError("目标车位不存在，请重新选择")
        slot = provider.slots[slot_id]
        if provider.availability(slot)["occupancy_state"] == "occupied":
            raise ValueError("目标停车位置被车辆或障碍阻挡，请选择其他车位")
        mode = payload.get("start_mode", "slot")
        if mode == "slot":
            pose = self.approach_pose(slot)
        elif mode == "custom":
            pose = payload.get("initial_pose")
            if not isinstance(pose, dict) or any(
                    type(pose.get(key)) not in (int, float) or not math.isfinite(pose[key])
                    for key in ("x", "y", "yaw_deg")):
                raise ValueError("初始后轴 X、Y 和航向必须是有限数值")
            pose = {key: float(pose[key]) for key in ("x", "y", "yaw_deg")}
            pose["yaw_deg"] = math.remainder(pose["yaw_deg"], 360.0)
        else:
            raise ValueError("初始位置模式无效")
        radius = self.parking_query_radius(provider)
        if any(abs(slot[f"slot_pt{i}_x"]-pose["x"]) >= radius-1 or
               abs(slot[f"slot_pt{i}_y"]-pose["y"]) >= radius-1 for i in range(1,5)):
            raise ValueError("起点距离目标过远，车位超出本次规划查询范围")
        return slot, mode, pose

    def request_map_change(self, payload: Dict[str, Any]) -> None:
        name = payload.get("map_name")
        choice = next((row for row in self.map_choices if row["map_name"] == name), None)
        if choice is None or not choice["available"]:
            raise ValueError("所选地图不可用，请检查地图安装和车位数据")
        if self.parking_enabled:
            raise ValueError("请先退出 APA 并停稳，再切换地图")
        velocity = self.ego.get_velocity()
        if math.hypot(velocity.x, velocity.y, velocity.z) > self.safety_stop.speed_limit:
            raise ValueError("请停稳车辆后再切换地图")
        if self.safety_stop.reason and self.safety_stop.state != "stopped":
            raise ValueError("请等待车辆确认停稳后再切换地图")
        provider = self._map_providers[name]
        slot, _, _ = self.resolve_scene_setup(payload, provider)
        config = copy.deepcopy(self.cfg)
        config["carla"]["map_name"] = name
        config["apa"].update(slot_file=str(provider.path.resolve()), slot_id=int(slot["slot_id"]), parking_action=1)
        self.stop_parking_task(clear_history=True)
        self.pending_map_config = config
        self.pending_map_command = dict(payload)
        self.running = False

    def complete_map_change(self) -> None:
        command = self.map_bootstrap_command
        if command.get("start_mode", "slot") == "custom":
            self.reset_parking_task(command)
        else:
            self.stop_parking_task(clear_history=True)
            self.apply_manual_hold()
            self.tick_world()
        result = dict(command, ok=True, state="applied", action="configure",
                      task_id=self.task_id, source_task_id=command["task_id"],
                      source_session_id=command["session_id"],
                      message=f"已切换到 {self.cfg['carla']['map_name']}，车辆保持驻车")
        self.map_transition = self.last_task_command = result
        self._task_command_results[result["event_id"]] = result
        self.last_status_publish = 0.0
        self.publish_status_if_needed(self.world.get_snapshot())
        self.map_bootstrap_command = None

    def restart_carla_for_map(self) -> None:
        """Optional trusted host adapter for servers that cannot hot-load maps."""
        command = self.cfg["carla"].get("map_reload_command")
        if not command:
            return
        name = str(self.cfg["carla"]["map_name"])
        args = [arg.replace("{map_name}", name) for arg in command]
        print(f"[APA maps] restart CARLA for {name}", flush=True)
        subprocess.run(args, check=True, timeout=90)
        api = import_carla(self.cfg)
        deadline = time.monotonic() + 120.0
        last_error = None
        last_notice = 0.0
        while self.running and time.monotonic() < deadline:

            probe = api.Client(str(self.cfg["carla"]["host"]), int(self.cfg["carla"]["port"]))
            probe.set_timeout(min(5.0, max(0.1, deadline - time.monotonic())))
            try:
                world = probe.get_world()

                print(f"[APA maps] fresh CARLA ready: {world.get_map().name}; target={name}", flush=True)
                return
            except RuntimeError as exc:
                last_error = str(exc)
                now = time.monotonic()
                if now - last_notice >= 5.0:
                    print(f"[APA maps] waiting for fresh CARLA: {last_error}", flush=True)
                    last_notice = now
            time.sleep(0.5)
        if not self.running:
            raise RuntimeError("CARLA 重启等待已取消")
        raise RuntimeError(f"等待 CARLA 重启就绪超时：{last_error or '未收到世界状态'}")

    def fail_map_change(self, command: Dict[str, Any], message: str) -> None:
        result = dict(command, ok=False, state="failed", action="configure",
                      source_session_id=command["session_id"],
                      source_task_id=command["task_id"], task_id=self.task_id, message=message)
        self.map_transition = self.last_task_command = result
        try:

            self.publish(APA_STATUS_TOPIC, {"session_id": self.session_id, "task_id": self.task_id,
                "map_name": self.cfg["carla"]["map_name"], "last_task_command": result,
                "map_transition": result, "map_choices": self.map_choices,
                "parking_enabled": False, "initial_pose": None, "control_status": "map_failed",
                "safety_state": self.safety_stop.payload()})
            time.sleep(0.2)
        except Exception as exc:
            print(f"[APA maps] cannot publish failure receipt: {exc}", file=sys.stderr, flush=True)

    def validate_timing(self) -> None:
        timing = self.cfg["timing"]
        step = float(self.cfg["carla"]["fixed_delta_seconds"])
        period = float(timing["control_period_sec"])
        timeout = float(timing["control_wait_timeout_sec"])
        retry = float(timing["feedback_retry_sec"])
        rpc_timeout = float(self.cfg["carla"].get("runtime_rpc_timeout_sec", 2.0))
        if not self.cfg["carla"].get("sync_mode"):
            raise ValueError("APA frame synchronization requires carla.sync_mode=true")
        if not all(math.isfinite(v) and v > 0 for v in (step, period, timeout, retry, rpc_timeout)):
            raise ValueError("APA timing values and runtime RPC timeout must be finite and positive")
        if abs(period - 0.02) > 1.0e-6 or step > period or abs(period / step - round(period / step)) > 1.0e-4:
            raise ValueError("APA control period is 0.02 s; CARLA step must divide this period")
        if retry >= timeout:
            raise ValueError("timing.feedback_retry_sec must be less than control_wait_timeout_sec")

    def sync_metadata(self, snapshot: Any = None) -> Dict[str, Any]:
        if not getattr(self, "session_id", ""):
            self.session_id = uuid.uuid4().hex
        key = (bool(getattr(self, "parking_enabled", False)), self.parking_action)
        if key != getattr(self, "_sync_task_key", None):
            self.task_id = int(getattr(self, "task_id", 0)) + 1
            self._sync_task_key = key
            self._last_plan_sim_time = None
            self._task_parkin_goal = None
            self.reset_recovery()
        return {
            "session_id": self.session_id,
            "task_id": self.task_id,
            "vehicle_profile_id": self.vehicle_profile_id,
            "carla_frame_id": int(getattr(snapshot, "frame", 0)),
            "sim_time_sec": finite(getattr(getattr(snapshot, "timestamp", None), "elapsed_seconds", 0.0)),
        }

    def reset_recovery(self) -> None:
        self.recovery_stop = SafetyStop(self._recovery_config)
        self._recovery_id = self._path_version = 0
        self._active_actuation_limits = None

    def validate_control_config(self) -> None:
        control = self.cfg["control"]
        apa = self.cfg["apa"]
        reload_command = self.cfg["carla"].get("map_reload_command")
        if reload_command is not None and (not isinstance(reload_command, list) or
                not all(isinstance(arg, str) and arg for arg in reload_command)):
            raise ValueError("carla.map_reload_command must be an argument array or null")
        for key in ("use_slot_lines_as_boundaries", "parallel_slot_frame_constraints",
                    "allow_neighbor_slot_borrowing"):
            if key in apa:
                if type(apa[key]) is not bool:
                    raise ValueError(f"apa.{key} must be boolean")
                print(f"[APA Bridge] obsolete apa.{key} ignored: slot entrances stay open, "
                      "confirmed-free neighbor access is frozen per task", flush=True)
        if "slot_boundary_radius_m" in apa:
            print("[APA Bridge] obsolete apa.slot_boundary_radius_m ignored: "
                  "markings use the frozen map query window", flush=True)
        clutch = self.cfg["apa"].get("drivetrain_clutch_strength")
        if clutch is not None and (type(clutch) not in (int, float) or
                                   not math.isfinite(clutch) or not 0 < clutch <= 10):
            raise ValueError("apa.drivetrain_clutch_strength must be null or in (0, 10]")
        unsupported = {"steer_filter_tau_sec", "max_forward_steer", "max_reverse_steer",
                       "forward_steer_sign", "reverse_steer_sign", "speed_kp", "timeout_sec", "timeout_brake"}
        legacy = unsupported.intersection(control)
        if legacy:
            raise ValueError("Unsupported APA control settings: " + ", ".join(sorted(legacy)) +
                             "; steering uses signed front-wheel radians, timing uses timing.*")
        if control.get("actuation") != "c_pedals_v1":
            raise ValueError("APA control.actuation must be c_pedals_v1")
        allowed = {"actuation", "max_abs_target_speed_mps", "max_accel_mps2",
                   "max_abs_steer_rad", "manual_pedal_slew_per_sec", "calibration_record_dir"}
        if set(control)-allowed:
            raise ValueError("Bridge has no parking controller; remove control settings: " +
                             ", ".join(sorted(set(control)-allowed)))
        for key in ("max_abs_target_speed_mps", "max_accel_mps2", "max_abs_steer_rad"):
            value = control[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"control.{key} must be finite and positive")
        if control["max_abs_target_speed_mps"] <= self.cfg["safety"]["standstill_speed_mps"]:
            raise ValueError("control speed cap must exceed the standstill speed")
        if control["max_abs_steer_rad"] >= math.pi / 2:
            raise ValueError("control.max_abs_steer_rad must be below pi/2")
        if control["max_abs_steer_rad"] > apa["vehicle_geometry"]["max_steer_rad"] + 1.e-6:
            raise ValueError("control.max_abs_steer_rad exceeds the shared vehicle steering lock")
        if self.cfg["carla"]["ego_blueprint_filter"] != apa["vehicle_geometry"]["blueprint"]:
            raise ValueError("ego_blueprint_filter must match the shared vehicle profile")
        value = control.get("calibration_record_dir", "")
        if not isinstance(value, str) or (value and not Path(value).is_absolute()):
            raise ValueError("control.calibration_record_dir must be empty or an absolute path")

    def accept_actuation_limits(self, payload: Dict[str, Any]) -> bool:
        if payload.get("vehicle_profile_id") != self.vehicle_profile_id:
            self.fail_sync("APA vehicle profile mismatch; rebuild/install APA with the shared vehicle profile")
            return False
        limits = payload.get("actuation_limits")
        control = self.cfg["control"]
        mapping = (("max_abs_target_speed_mps", "max_abs_target_speed_mps", "TargetSpeed"),
                   ("max_abs_accel_mps2", "max_accel_mps2", "TargetAcceleration"),
                   ("max_abs_steer_rad", "max_abs_steer_rad", "FrontWheelAngle"))
        for field, config_key, command in mapping:
            value = limits.get(field) if isinstance(limits, dict) else None
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                self.fail_sync("missing/invalid APA actuation_limits; install matching APA nodes")
                return False
            if value > control[config_key] + 1.e-6:
                self.fail_sync(f"APA {field} exceeds Bridge control.{config_key}; align configurations")
                return False
            if payload["enable"] and not payload["StopReq"] and abs(payload[command]) > value + 1.e-6:
                self.fail_sync(f"APA {command} exceeds its declared actuation limit")
                return False
        active = getattr(self, "_active_actuation_limits", None)
        if active is not None and any(abs(limits[field]-active[field]) > 1.e-6 for field, _, _ in mapping):
            self.fail_sync("APA actuation limits changed within a task")
            return False
        self._active_actuation_limits = {field: limits[field] for field, _, _ in mapping}
        return True

    def apply_sync_hold(self, stop: Optional[SafetyStop] = None) -> None:
        stop = self.safety_stop if stop is None else stop
        with self.safety_stop.lock:
            if self.ego is not None and self.carla is not None:

                applied = self.ego.get_control()
                gear = int(getattr(applied, "gear", 0))
                self._submit_vehicle_control(self.carla.VehicleControl(
                    throttle=0.0, brake=1.0, steer=applied.steer,
                    manual_gear_shift=True, gear=gear, reverse=gear < 0,
                    hand_brake=stop.state == "stopped"), timeout_sec=self.safety_stop.rpc_timeout)
            self.last_mapped_throttle = 0.0
            self.last_mapped_brake = 1.0

    def latch_safety_stop(self, reason: str) -> None:
        with self.safety_stop.lock:
            self.safety_stop.latch(reason)
            self._pending_control = None
            self._control_ack = False
            if reason not in ("cancelled", "shutdown"):
                self._sync_fault = self.safety_stop.reason
            self.last_status_publish = 0.0
            try:
                self.apply_sync_hold()
            except Exception as exc:
                self.safety_stop.unconfirmed(f"brake submission failed: {exc}")

    def observe_safety_stop(self, snapshot: Any, stop: Optional[SafetyStop] = None) -> None:
        stop = self.safety_stop if stop is None else stop
        if not stop.reason:
            return
        timestamp = snapshot.timestamp
        step = float(self.cfg["carla"]["fixed_delta_seconds"])
        measured_step = float(timestamp.delta_seconds)
        if not math.isfinite(measured_step) or abs(measured_step - step) > 1.0e-5:
            raise RuntimeError("CARLA safety feedback has an unexpected fixed step")
        source = snapshot.find(self.ego.id)
        if source is None:
            raise RuntimeError("ego missing from CARLA safety feedback")
        velocity = source.get_velocity()

        speed = math.sqrt(sum(float(v) ** 2 for v in (velocity.x, velocity.y, velocity.z)))
        stop.observe(int(snapshot.frame), float(timestamp.elapsed_seconds), speed, step)

    def publish_safety_feedback(self, snapshot: Any) -> None:

        try:
            self.publish(EGO_STATE_TOPIC, self.ego_payload(snapshot), snapshot)
            self._safety_tx_failed = False
        except Exception as exc:
            if not getattr(self, "_safety_tx_failed", False):
                print(f"[APA safety] feedback unavailable while braking: {exc}", flush=True)
            self._safety_tx_failed = True

    def advance_safety_stop(self, snapshot: Any, stop: Optional[SafetyStop] = None) -> Any:
        """Advance brake-only physics without accepting an algorithm response."""
        stop = self.safety_stop if stop is None else stop
        try:
            self.apply_sync_hold(stop)
            self.observe_safety_stop(snapshot, stop)
            self.publish_safety_feedback(snapshot)

            snapshot = self.tick_world(self.safety_stop.rpc_timeout)
            self.observe_safety_stop(snapshot, stop)
            self.publish_safety_feedback(snapshot)
        except Exception as exc:
            stop.unconfirmed(f"CARLA stopping failed: {exc}")

            try:
                self.apply_sync_hold(stop)
            except Exception:
                pass
        return snapshot

    def shutdown_safely(self) -> None:
        if self.ego is None or self.carla is None or self.world is None:
            return
        self.latch_safety_stop("shutdown")
        self.parking_enabled = False
        self._sync_task_key = None
        self.sync_metadata()
        try:
            self.observe_safety_stop(self.world.get_snapshot())

            while self.safety_stop.state == "braking":
                self.advance_safety_stop(self.world.get_snapshot())
            if self.safety_stop.state == "stopped":
                self.apply_sync_hold()
            print(f"[APA safety] shutdown stop={self.safety_stop.state} "
                  f"standstill_sec={self.safety_stop.standstill_sec:.3f}", flush=True)
        except Exception as exc:
            self.safety_stop.unconfirmed(f"shutdown stop failed: {exc}")

    def fail_sync(self, reason: str) -> None:
        if not self._sync_fault:
            print(f"[APA Bridge] synchronization stopped: {reason}", flush=True)
        self.latch_safety_stop(reason)

    def accept_control(self, payload: Dict[str, Any]) -> None:
        pending = self._pending_control
        if not getattr(self, "running", True) or not self.parking_enabled or pending is None or self.safety_stop.reason or self._sync_fault:
            return
        identity = self.sync_metadata()
        if any(identity[k] != pending[k] for k in ("session_id", "task_id")):
            return
        if any(payload.get(key) != pending[key] for key in ("session_id", "task_id")):
            return

        validation = payload.get("input_validation")
        if (isinstance(validation, dict) and type(validation.get("error")) is str and
                validation["error"]):
            self.last_control_payload = payload
            self.fail_sync(f"APA input rejected: {validation['error']}")
            return
        online = payload.get("online_safety")
        replan = payload.get("replan")
        if isinstance(replan, dict) and all(type(replan.get(k)) is int for k in ("recovery_id", "path_version")):
            if replan["recovery_id"] < self._recovery_id or replan["path_version"] < self._path_version:
                return
        recovering = (isinstance(replan, dict) and isinstance(online, dict) and
                      online.get("reason") in ("path_blocked", "body_predicted", "planning_clearance",
                                               "clearance_deficit", "clearance_predicted", "motion_blocked",
                                               "clearance_recovery_deviation") and
                      (replan.get("state"), payload.get("control_status")) in
                      (("stopping", "replan_stopping"), ("planning", "planning"), ("waiting_scene", "replan_waiting")) and
                      payload.get("enable") is False and payload.get("StopReq") is True)
        motion_braking = (isinstance(online, dict) and online.get("reason") in ("clearance_predicted", "motion_blocked") and
                            isinstance(replan, dict) and replan.get("state") == "ready" and
                            payload.get("control_status") == "motion_braking" and
                            payload.get("enable") is False and payload.get("StopReq") is True and
                            payload.get("TargetSpeed") == 0.0 and payload.get("TargetAcceleration") == 0.0 and
                            payload.get("GearChangeAllowed") is False)
        if isinstance(online, dict) and online.get("blocked") is True and not (recovering or motion_braking):
            self.last_control_payload = payload
            reason = online.get("reason", "")
            category = "clearance_stop" if reason in (
                "planning_clearance", "clearance_deficit", "clearance_predicted",
                "clearance_recovery_deviation") else "obstacle_stop"
            self.fail_sync(f"{category}: {online.get('obstacle_id', '')} {reason}")
            return
        if self._control_ack or payload.get("carla_frame_id") != pending["carla_frame_id"]:
            return
        if any(type(payload[k]) is not int for k in ("task_id", "carla_frame_id")):
            self.fail_sync("invalid synchronized control identity")
            return
        required = ("sim_time_sec", "control_dt_sec", "control_status", "enable", "StopReq",
                    "TargetSpeed", "TargetAcceleration", "FrontWheelAngle", "TargetGearReq",
                    "DirectionSign", "GearChangeAllowed", "CtrlPhase", "safety_state", "input_validation", "online_safety", "replan", "actuation_limits", "actuator")
        if any(key not in payload for key in required):
            self.fail_sync("incomplete synchronized control response; rebuild/install APA")
            return
        numeric = ("sim_time_sec", "control_dt_sec", "TargetSpeed", "TargetAcceleration", "FrontWheelAngle")
        if any(type(payload[k]) not in (int, float) or not math.isfinite(payload[k]) for k in numeric):
            self.fail_sync("non-finite synchronized control response")
            return
        if abs(payload["sim_time_sec"] - pending["sim_time_sec"]) > 1.0e-6:
            self.fail_sync("control response simulation time differs from feedback")
            return
        if any(type(payload[k]) is not bool for k in ("enable", "StopReq", "GearChangeAllowed")):
            self.fail_sync("invalid synchronized control flags")
            return
        if any(type(payload[key]) is not int or payload[key] not in values
               for key, values in (("TargetGearReq", range(4)), ("DirectionSign", (-1, 0, 1)), ("CtrlPhase", range(10)))):
            self.fail_sync("invalid synchronized control enum")
            return
        if not self.accept_actuation_limits(payload):
            return
        try:
            validate_actuator(payload)
        except ValueError as exc:
            self.fail_sync(str(exc))
            return
        planning_only = bool(getattr(self, "planning_only", False))
        if (payload.get("planning_only", False) is not planning_only or
                (planning_only and (payload["enable"] or not payload["StopReq"] or
                 payload["TargetSpeed"] != 0.0 or payload["TargetAcceleration"] != 0.0 or
                 payload["FrontWheelAngle"] != 0.0))):
            self.fail_sync("planning-only task requires matching C brake-only control; rebuild/install APA")
            return
        validation = payload["input_validation"]
        if (not isinstance(validation, dict) or
                any(type(validation.get(key)) is not bool for key in ("feedback_valid", "request_valid")) or
                type(validation.get("error")) is not str):
            self.fail_sync(f"incomplete APA input validation; verify APA {PROTOCOL_VERSION} protocol/build")
            return
        if validation["error"] or not validation["feedback_valid"]:
            self.last_control_payload = payload
            self.fail_sync(f"APA input rejected: {validation['error'] or 'unvalidated_feedback'}")
            return
        if payload["control_status"] in ("plan_ready", "tracking", "holding", "motion_braking", "complete", "tracking_fault") and not validation["request_valid"]:
            self.fail_sync("APA control has no validated planning request")
            return
        if (not isinstance(online, dict) or type(online.get("blocked")) is not bool or
                type(online.get("scene_version")) is not int or online["scene_version"] != pending["carla_frame_id"]):
            self.fail_sync(f"control has no matching online scene check; verify APA {PROTOCOL_VERSION} protocol/build")
            return
        safety = payload["safety_state"]
        if (not isinstance(safety, dict) or type(safety.get("event_id")) is not int or
                type(safety.get("standstill_sec")) not in (int, float) or
                not math.isfinite(safety["standstill_sec"]) or
                safety != self.safety_stop.payload()):
            self.fail_sync(f"APA safety state differs from feedback; verify APA {PROTOCOL_VERSION} protocol/build")
            return
        self.last_control_payload = payload
        if (isinstance(replan, dict) and replan.get("state") == "failed" and
                payload["control_status"] != "plan_failed"):
            self.fail_sync(f"replan_failed: {replan.get('reason', '')}")
            return
        if payload["control_status"] == "timing_error":
            self.fail_sync(payload["control_status"])
            return
        if payload["control_status"] not in ("manual", "waiting_request", "planning", "plan_ready", "replan_stopping", "replan_waiting", "plan_failed", "tracking", "holding", "motion_braking", "complete", "tracking_fault"):
            self.fail_sync("unsupported synchronized control status")
            return
        expected_dt = self.cfg["timing"]["control_period_sec"] if payload["control_status"] in ("tracking", "complete", "tracking_fault") else 0.0
        if (payload["control_status"] == "tracking_fault" and
                payload.get("diagnostic", {}).get("stop_reason") == 9 and
                payload["control_dt_sec"] == 0.0):

            expected_dt = 0.0
        if payload["control_status"] == "motion_braking":

            expected_dt = self.cfg["timing"]["control_period_sec"] if payload["control_dt_sec"] else 0.0
            if (not isinstance(replan, dict) or replan.get("state") != "ready" or payload["enable"] or not payload["StopReq"] or
                    payload["TargetSpeed"] != 0.0 or payload["TargetAcceleration"] != 0.0 or
                    payload["GearChangeAllowed"]):
                self.fail_sync("motion brake requires a ready path and brake-only control")
                return
        if abs(payload["control_dt_sec"] - expected_dt) > 1.0e-6:
            self.fail_sync("control step differs from configured simulation control period")
            return
        if (not isinstance(replan, dict) or any(type(replan.get(k)) is not int or replan[k] < 0 for k in ("recovery_id", "path_version")) or
                replan.get("state") not in ("idle", "stopping", "planning", "waiting_scene", "ready", "failed")):
            self.fail_sync(f"incomplete APA recovery/path version; verify APA {PROTOCOL_VERSION} protocol/build")
            return
        if payload["control_status"] == "plan_failed":

            if (replan["state"] != "failed" or replan["recovery_id"] != self._recovery_id or
                    not self.recovery_stop.reason or self.recovery_stop.state != "stopped" or
                    payload["enable"] or not payload["StopReq"]):
                self.fail_sync("planning failure without confirmed recovery stop")
                return
            self.apply_control(payload)
            self._control_ack = True
            return
        if replan["state"] == "stopping":
            if replan["recovery_id"] == 0 or payload["enable"] or not payload["StopReq"]:
                self.fail_sync("recovery stop requires identity and brake-only control")
                return
            if replan["recovery_id"] != self._recovery_id or not self.recovery_stop.reason:
                self._recovery_id = replan["recovery_id"]
                self.recovery_stop = SafetyStop(self._recovery_config)
                self.recovery_stop.latch("replan")
                self.last_plan_result = None
            self.apply_control(payload)

            self._control_ack = True
            return
        if replan["state"] in ("planning", "waiting_scene"):
            if (replan["recovery_id"] != self._recovery_id or self.recovery_stop.state != "stopped" or
                    payload["enable"] or not payload["StopReq"]):
                self.fail_sync("planning response without confirmed recovery stop")
                return
            self.apply_control(payload)
            self._control_ack = True
            return
        if replan["state"] == "ready":
            if replan["path_version"] == 0 or replan["recovery_id"] != self._recovery_id:
                self.fail_sync("ready path has no matching recovery/version")
                return
            if self.recovery_stop.reason:
                if (replan["path_version"] <= self._path_version or
                        not self.recovery_stop.clear()):
                    self.fail_sync("new path cannot resume before recovery standstill")
                    return
                self._path_version = replan["path_version"]
            elif replan["path_version"] != self._path_version:
                self.fail_sync("path changed without recovery stop")
                return
        elif payload["enable"] or not payload["StopReq"]:
            self.fail_sync("drive command without a ready path")
            return
        self.apply_control(payload)
        self._control_ack = True

    def publish_status_if_needed(self, snapshot: Any, *, read_vehicle=True) -> None:
        now = time.monotonic()
        if snapshot is not None:
            sim_time = float(snapshot.timestamp.elapsed_seconds)
            previous = getattr(self, "_simulation_rate_sample", None)
            if previous is None or sim_time < previous[1]:
                self._simulation_rate_sample = (now, sim_time)
                self.simulation_rate = None
            elif now - previous[0] >= 1.0:
                self.simulation_rate = max(0.0, (sim_time - previous[1]) / (now - previous[0]))
                self._simulation_rate_sample = (now, sim_time)
        if now - self.last_status_publish >= 0.1:
            self.last_status_publish = now
            self.publish(APA_STATUS_TOPIC, self.status_payload(read_vehicle=read_vehicle), snapshot)

    def wait_for_control(self, snapshot: Any, feedback: Dict[str, Any]) -> bool:
        self._pending_control = {key: feedback[key] for key in ("session_id", "task_id", "carla_frame_id", "sim_time_sec")}
        self._control_ack = False
        timeout = self.cfg["timing"]["control_wait_timeout_sec"]
        retry = self.cfg["timing"]["feedback_retry_sec"]
        deadline = time.monotonic() + timeout
        last_send = 0.0
        try:
            while self.running and self.parking_enabled and not self._sync_fault:
                self.process_zmq_messages()
                if not self.running or not self.parking_enabled or self._pending_control is None:
                    return False
                identity = self.sync_metadata(snapshot)
                if any(identity[k] != self._pending_control[k] for k in ("session_id", "task_id")):
                    return False
                if self._control_ack:
                    return True
                now = time.monotonic()
                if now >= deadline:
                    self.fail_sync(f"control timeout at CARLA frame {snapshot.frame}; verify APA {PROTOCOL_VERSION} protocol/build")
                    return False
                if now - last_send >= retry:
                    self.publish_plan_request_if_needed(snapshot, force=last_send > 0.0)
                    if self.safety_stop.reason or self._pending_control is None:
                        return False
                    self.publish(EGO_STATE_TOPIC, feedback, snapshot)
                    last_send = now
                self.publish_status_if_needed(snapshot)
                self.sub.poll(5)
            return False
        finally:
            self._pending_control = None

    def resolve_slot_file(self) -> Path:
        slot_file = str(self.cfg["apa"].get("slot_file", ""))
        if slot_file:
            path = Path(slot_file)
            if not path.is_absolute():
                path = Path(str(self.cfg.get("_config_dir", "."))) / path
            return path
        _, file_name, _ = parking_map_preset(self.cfg["carla"].get("map_name", "Town04"))
        return Path(str(self.cfg.get("_config_dir", "."))) / file_name

    def setup_zmq(self) -> None:
        self.pub.bind(str(self.cfg["zmq"]["publish_bind"]))
        control_endpoint = self.cfg["zmq"].get(
            "control_connect", self.cfg["zmq"].get("control_bind", "tcp://127.0.0.1:5702")
        )
        self.sub.connect(str(control_endpoint))
        command_endpoint = str(
            self.cfg["zmq"].get("command_bind", "tcp://127.0.0.1:5703")
        )
        self.command_sub.bind(command_endpoint)
        print(f"[APA Bridge] PUB bind {self.cfg['zmq']['publish_bind']}")
        print(f"[APA Bridge] SUB connect {control_endpoint}")
        print(f"[APA Bridge] COMMAND SUB bind {command_endpoint}")

    def connect_carla(self) -> None:
        if self.dry_run:
            self.configure_planning_window()
            print("[APA Bridge] dry-run mode, skip CARLA connection")
            return
        self.carla = import_carla(self.cfg)
        carla_cfg = self.cfg["carla"]
        from calibration.carla_connection import connect_checked
        self.client, connection = connect_checked(self.carla, {
            "host": str(carla_cfg["host"]), "port": int(carla_cfg["port"]),
            "timeout_sec": float(carla_cfg.get("timeout_sec", 30.0))})
        print("[APA Bridge] CARLA RPC: " + json.dumps(connection), flush=True)
        if not connection["version_match"]:
            raise ValueError("CARLA client/server versions differ; match the Python API before starting APA")
        available_maps = {name.rsplit("/", 1)[-1] for name in self.client.get_available_maps()}
        for choice in self.map_choices:
            if choice["available"] and choice["map_name"] not in available_maps:
                choice.update(available=False, reason="地图未安装")
        self.world = self.client.get_world()
        map_name = str(carla_cfg.get("map_name", ""))
        carla_map = self.world.get_map()
        if map_name and map_name not in carla_map.name:
            self.world = self.load_world_with_retry(map_name)
            time.sleep(1.0)
            carla_map = self.world.get_map()
        self._map_name = carla_map.name
        self._loaded_map = carla_map
        self._loaded_map_world_id = self.world.id
        self.spectator = self.world.get_spectator()
        self.slot_provider.bind_map(carla_map)
        self._map_preparation = {"available": False, "state": "unavailable", "region_count": 0}
        print("[APA parking map] " + json.dumps(self.slot_provider.map_summary), flush=True)
        if bool(carla_cfg.get("sync_mode", True)):
            settings = self.world.get_settings()
            self.original_settings = {
                "synchronous_mode": settings.synchronous_mode,
                "fixed_delta_seconds": settings.fixed_delta_seconds,
            }
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = float(carla_cfg.get("fixed_delta_seconds", 0.02))
            self.world.apply_settings(settings)

        for row in self.slot_provider.slots.values():

            native = row.get("_native_geometry")
            try:
                if "_surveyed_ground_z" in row:
                    row["_occupancy_ground_z"] = required_number(row["_surveyed_ground_z"], "slot.surveyed_ground_z")
                else:
                    row["_occupancy_ground_z"] = (required_number(native["z"], "slot.ground_z") if native else
                        self.sample_ground_z(sum(row[f"slot_pt{i}_x"] for i in range(1, 5))/4,
                                             sum(row[f"slot_pt{i}_y"] for i in range(1, 5))/4))
            except (KeyError, TypeError, ValueError):
                row.pop("_occupancy_ground_z", None)
            for i in range(1, 5):
                try:
                    self.sample_ground_z(float(row[f"slot_pt{i}_x"]), float(row[f"slot_pt{i}_y"]))
                except (KeyError, TypeError, ValueError):
                    continue
        self.ego = self.find_or_spawn_ego()
        if self.ego.type_id != self.cfg["apa"]["vehicle_geometry"]["blueprint"]:
            raise RuntimeError("CARLA ego vehicle does not match the shared APA vehicle profile")
        print(
            f"[APA Bridge] vehicle/path reference={REAR_AXLE_REFERENCE_POINT} "
            f"frame=carla_world actor_to_rear_m={self.rear_axle_offset_m():.5f}",
            flush=True,
        )
        if self.ego is not None and hasattr(self.ego, "set_simulate_physics"):
            self.ego.set_simulate_physics(True)
        if hasattr(self.ego, "set_collisions"):

            self.ego.set_collisions(True)
        self.apply_front_wheel_lock()

        self._ego_bounding_box = self.ego.bounding_box
        self._ego_bounding_box_actor_id = self.ego.id
        recording_dir = self.cfg["control"].get("calibration_record_dir")
        if recording_dir:
            from calibration.apa_pedal_measurements import PedalRecorder, vehicle_identity
            version = self.client.get_server_version()
            step = float(self.cfg["carla"]["fixed_delta_seconds"])
            if recording_dir:
                path = Path(recording_dir) / f"apa-pedals-{self.session_id}.csv"
                self.calibration_recorder = PedalRecorder(path, {
                    "vehicle": vehicle_identity(self.ego), "carla_version": version,
                    "fixed_delta_seconds": step, "map_name": self.world.get_map().name,
                    "session_id": self.session_id})
                self._calibration_recording_path = str(path)
                print(f"[APA Bridge] same-frame pedal recording: {path}", flush=True)
        print("[APA Bridge] physics actuation: direct C pedals and geometric steer conversion")
        self.current_apa_gear_pos = 1
        self.setup_collision_sensor()
        self.attach_camera()
        self.start_camera_http()
        self.ignore_control_until = time.monotonic() + 2.0
        print("[APA Bridge] ignore control for 2.0s after ego reset")
        if bool(self.cfg["apa"].get("spawn_occupied_vehicles", True)):
            self.spawn_occupied_vehicles()

        self.apply_sync_hold()
        self.tick_world()
        self.settle_ego_for_geometry()
        self.configure_planning_window()
        self.refresh_environment_obstacles()
        if self.ego is not None:
            x, y, yaw = self.rear_axle_pose(self.ego.get_transform())
            self.initial_pose = {"x": x, "y": y, "yaw_deg": math.degrees(yaw)}

        self.attach_camera("overhead")
        runtime_timeout = float(carla_cfg["runtime_rpc_timeout_sec"])
        self.client.set_timeout(runtime_timeout)
        print(f"[APA Bridge] RPC timeouts: runtime={runtime_timeout:.1f}s "
              f"safety={self.safety_stop.rpc_timeout:.1f}s", flush=True)
        lines, vlines, policy = self.boundary_inputs()
        print(f"[APA Bridge] explicit boundaries={len(lines)} virtual constraints={len(vlines)} "
              f"target_frame_policy={policy['target_frame_policy']} "
              f"neighbor_slot_access={policy['neighbor_slot_access']} "
              f"marking_collision_policy={policy['marking_collision_policy']} "
              f"target_slot={self.target_slot['slot_id']}", flush=True)

    def refresh_environment_obstacles(self, *, refresh_static=False) -> None:
        world_key = (int(self.world.id), self._map_name)
        reused = not refresh_static and getattr(self, "_environment_world_key", None) == world_key
        if not reused:
            self.environment_obstacles = environment_obstacle_sources(self.world, self.carla)
            self._static_obstacle_index = StaticObstacleIndex(self.environment_obstacles)
            self._environment_world_key = world_key
            self._slot_occupancy_cache.clear()
            self._slot_occupancy_explicit = None
        self.slot_provider.occupancy.clear()
        self._last_occupancy_time = None
        self.load_offline_parking_map(force=refresh_static)
        self.refresh_slot_occupancy(self.world.get_snapshot())

    def refresh_slot_occupancy(self, snapshot):
        state = snapshot.find(self.ego.id) if self.ego is not None else None
        if state is None:
            return
        transform = state.get_transform()
        x, y, _ = self.rear_axle_pose(transform)
        low, high, _ = vehicle_obstacle_height_band(self._ego_bounding_box.get_world_vertices(transform))
        self.update_slot_occupancy(snapshot, self.actor_obstacle_sources(snapshot), x, y, low, high, force=True)
        self.last_status_publish = 0.0
        self.publish_status_if_needed(snapshot)

    def actor_obstacle_sources(self, snapshot):
        sources = []
        for actor in self.world.get_actors():
            if actor.id == self.ego.id or not actor.type_id.startswith(("vehicle.", "walker.pedestrian.", "static.prop.")):
                continue
            state = snapshot.find(actor.id)
            if state is None:
                continue
            velocity = state.get_velocity()
            obstacle = obstacle_from_vertices(f"actor:{actor.id}",
                actor.bounding_box.get_world_vertices(state.get_transform()), velocity.x, velocity.y)
            obstacle["_profile_eligible"] = actor.type_id.startswith("static.prop.")
            sources.append(obstacle)
        return sources

    def scene_payload(self, snapshot: Any, force_occupancy=False) -> Dict[str, Any]:
        ego = snapshot.find(self.ego.id)
        if (getattr(self, "_ego_bounding_box_actor_id", None) != self.ego.id or
                getattr(self, "_ego_bounding_box", None) is None):
            self._ego_bounding_box = self.ego.bounding_box
            self._ego_bounding_box_actor_id = self.ego.id
        ego_vertices = self._ego_bounding_box.get_world_vertices(ego.get_transform())
        z_min, z_max, _ = vehicle_obstacle_height_band(ego_vertices)
        x, y, yaw = self.rear_axle_pose(ego.get_transform())
        region = self._planning_query_region
        query_bounds = region["bounds"]
        inside = all(query_bounds[0] < p.x < query_bounds[2] and
                     query_bounds[1] < p.y < query_bounds[3] for p in ego_vertices)
        if not inside and self.parking_enabled:
            raise ValueError("车辆超出本次规划查询范围，需停车并重新应用场景设置")

        atlas = getattr(self, "_offline_parking_map", None)
        cell_bands = atlas.connected_height_bands(query_bounds, (x, y), z_min, z_max) if atlas else {}

        def swept_bounds(obstacle):
            points = obstacle["footprint"]
            dx, dy = obstacle["velocity_x"]*OBSTACLE_PREDICTION_SEC, obstacle["velocity_y"]*OBSTACLE_PREDICTION_SEC
            xs, ys = [p["x"] for p in points], [p["y"] for p in points]

            return [min(xs)+min(dx, 0)-0.4, min(ys)+min(dy, 0)-0.4,
                    max(xs)+max(dx, 0)+0.4, max(ys)+max(dy, 0)+0.4]

        def local_parts(obstacle):
            a, b, c, d = swept_bounds(obstacle)
            bounds = [max(a, query_bounds[0]), max(b, query_bounds[1]),
                      min(c, query_bounds[2]), min(d, query_bounds[3])]
            if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
                return []
            return atlas.query_body(obstacle, bounds, cell_bands) if atlas else []

        index = getattr(self, "_static_obstacle_index", None)
        static = index.query([query_bounds[0]-0.4, query_bounds[1]-0.4,
                              query_bounds[2]+0.4, query_bounds[3]+0.4]) if index else self.environment_obstacles
        bodies = [item for item in static if not item.get("_profile_eligible") and
                  (atlas is None or atlas.intersects_coverage(item["_xy_bounds"]))]
        sources = list(bodies)
        groups = [(item, [dict(part, id="occupancy:"+part["id"])
                         for part in local_parts(item)]) for item in bodies]
        for obstacle in self.actor_obstacle_sources(snapshot):
            sources.append(obstacle)
            groups.append((obstacle, local_parts(obstacle)))
        vehicle = self.cfg['apa']['vehicle_geometry']
        center_offset = (vehicle['rear_to_front_m']-vehicle['rear_to_back_m'])/2.0
        planning_center = (x+center_offset*math.cos(yaw), y+center_offset*math.sin(yaw))
        obstacles, map_query = self.offline_map_query(query_bounds, z_min, z_max,
                                                     planning_center=planning_center, cell_bands=cell_bands)
        obstacles = list(obstacles)
        for _, parts in groups:
            for part in parts:
                if math.hypot(part["velocity_x"], part["velocity_y"]) > 1e-4:
                    obstacles.append(part)
                else:
                    polygon = clip_xy_polygon(part["footprint"], query_bounds, max_vertices=None)
                    if len(polygon) <= 8:
                        if polygon:
                            obstacles.append(dict(part, footprint=polygon))
                    else:

                        for first in range(1, len(polygon)-1, 6):
                            obstacles.append(dict(part, id=f'{part["id"]}:clip:{first}',
                                footprint=[polygon[0]]+polygon[first:first+7]))
        obstacles.extend(parking_region_guards(query_bounds, z_min, z_max))

        seen, unique = set(), []
        for part in obstacles:
            if part["id"].startswith(("map:", "occupancy:map:")) or ":unknown:" in part["id"]:
                signature = (":unknown:" in part["id"], part["velocity_x"], part["velocity_y"],
                             tuple(sorted((p["x"], p["y"]) for p in part["footprint"])))
                if signature in seen:
                    continue
                seen.add(signature)
            unique.append(part)
        obstacles = unique
        if not hasattr(self, "_scene_compactor"):
            self._scene_compactor = StaticSceneCompactor()
        obstacles, self._scene_map_summary = self._scene_compactor.compact(obstacles)
        self._scene_map_summary.update(map_query, static_map_revision=getattr(self, "_offline_map_revision", ""),
            live_obstacle_pieces=sum(p["id"].startswith(("actor:", "occupancy:")) for p in obstacles))
        if len(obstacles) > SCENE_MAX_OBSTACLES:
            raise ValueError(f"实时障碍 {len(obstacles)} 个，超过 {SCENE_MAX_OBSTACLES} 上限，不能截断场景")
        self.update_slot_occupancy(snapshot, sources, x, y, z_min, z_max, force=force_occupancy)
        return {"version": int(snapshot.frame), "obstacles": obstacles, "coverage": region,
                "contract": scene_contract(self._map_name, getattr(self, "_offline_map_revision", ""),
                    self.vehicle_profile_id, query_bounds, (z_min, z_max), obstacles),
                "representation": dict(self._scene_map_summary, kind="offline_map_with_live_occupancy")}

    @staticmethod
    def classify_slot_obstacles(polygon, bounds, low, high, sources):
        result = {"state": "free", "reason": "clear", "obstacle_id": ""}

        for item in sorted(sources, key=lambda item: bool(item.get("_profile_eligible"))):
            if item["z_max"] < low or item["z_min"] > high:
                continue
            left, bottom, right, top = item["_xy_bounds"]
            if bounds[0] > right+0.4 or bounds[2] < left-0.4 or bounds[1] > top+0.4 or bounds[3] < bottom-0.4:
                continue
            if not convex_polygons_overlap(polygon, item["footprint"]):
                continue
            needs_profile = obstacle_needs_profile(item, high)
            if needs_profile:
                result.update(state="unknown", reason="geometry_unprepared", obstacle_id=item["id"])
                continue
            for part in obstacle_at_height(item, low, high):
                if not convex_polygons_overlap(polygon, part["footprint"]):
                    continue
                return {"state": "occupied", "reason": "physical_overlap", "obstacle_id": part["id"]}
        return result

    def update_slot_occupancy(self, snapshot, sources, x, y, z_min, z_max, *, force=False):
        """Classify nominal parked bodies using certified map geometry and same-frame actors, excluding ego."""
        now = snapshot.timestamp.elapsed_seconds
        previous = getattr(self, "_last_occupancy_time", None)
        if not force and previous is not None and 0 <= now-previous < 0.2:
            return
        self._last_occupancy_time = now
        frame = int(snapshot.frame)
        try:
            ego_ground = self.sample_ground_z(x, y, allow_query=False)
        except (ValueError, RuntimeError):
            ego_ground = None
        index = self._static_obstacle_index
        atlas = self._offline_parking_map
        if self._slot_occupancy_explicit is None:
            physical, virtual, _ = build_boundary_inputs(self.slot_provider.obstacles, [])
            self._slot_occupancy_explicit = [dict(id=f'goal:line:{i}',
                footprint=[{'x': line['x1'], 'y': line['y1']}, {'x': line['x2'], 'y': line['y2']}],
                goal_margin_m=margin)
                for margin, lines in ((self.cfg['apa']['vehicle_geometry']['parkin_physical_margin_m'], physical),
                                      (self.cfg['apa']['vehicle_geometry']['parkin_margin_m'], virtual))
                for i, line in enumerate(lines)]
        actors = [item for item in sources if item["id"].startswith("actor:")]
        for slot in self.slot_provider.slots.values():
            slot_id = int(slot["slot_id"])
            state = {"state": "unknown", "frame": frame, "reason": "ground_unconfirmed"}
            self.slot_provider.occupancy[slot_id] = state
            ground = slot.get("_occupancy_ground_z")
            if ego_ground is None or ground is None:
                continue
            entry = self._slot_occupancy_cache.get(slot_id)
            if entry is None:
                try:
                    pose = self.parkin_target(slot, obstacles=())
                except ValueError:
                    state["reason"] = "parking_pose_unconfirmed"
                    continue
                polygon = [{'x': slot[f'slot_pt{i}_x'], 'y': slot[f'slot_pt{i}_y']} for i in range(1, 5)]
                footprint = parking_footprint(self.cfg['apa']['vehicle_geometry'], pose)
                points = polygon+footprint
                bounds = [min(p['x'] for p in points)-0.4, min(p['y'] for p in points)-0.4,
                          max(p['x'] for p in points)+0.4, max(p['y'] for p in points)+0.4]
                geometry = self.cfg['apa']['vehicle_geometry']
                offset = (geometry['rear_to_front_m']-geometry['rear_to_back_m'])/2
                center = (pose[0]+offset*math.cos(pose[2]), pose[1]+offset*math.sin(pose[2]))
                entry = dict(polygon=polygon, bounds=bounds, center=center, key=None)
                self._slot_occupancy_cache[slot_id] = entry
            low, high = z_min+ground-ego_ground, z_max+ground-ego_ground
            query_bounds, center = entry['bounds'], entry['center']
            relevant = [a for a in actors if a['_xy_bounds'][0] <= query_bounds[2]
                        and a['_xy_bounds'][2] >= query_bounds[0] and a['_xy_bounds'][1] <= query_bounds[3]
                        and a['_xy_bounds'][3] >= query_bounds[1]]
            actor_key = tuple((a['id'], a['_vertices'], a['velocity_x'], a['velocity_y']) for a in relevant)
            key = (atlas.height_seed(center, low, high) if atlas else None, actor_key)
            if entry['key'] != key:
                bands = atlas.connected_height_bands(query_bounds, center, low, high) if atlas else {}
                parts, _ = self.offline_map_query(query_bounds, low, high, planning_center=center, cell_bands=bands)
                bodies = [item for item in index.query(query_bounds) if not item.get('_profile_eligible')]
                body_parts = [part for item in relevant+bodies
                              for part in (atlas.query_body(item, query_bounds, bands) if atlas else [])]
                entry['result'] = self._evaluate_slot_occupancy(slot, entry['polygon'], parts, body_parts, frame)
                entry['key'] = key
            state = dict(entry['result'], frame=frame)
            if 'static_frame' in state:
                state['static_frame'] = frame
            self.slot_provider.occupancy[slot_id] = state
        counts = {name: sum(row["state"] == name for row in self.slot_provider.occupancy.values())
                  for name in ("free", "occupied", "unknown")}
        blocked = sum(row["state"] == "occupied" and row["reason"] == "parking_envelope_blocked"
                      for row in self.slot_provider.occupancy.values())
        self._slot_occupancy_summary = dict(counts, blocked=blocked, frame=frame, refresh_period_sec=0.2)
        for choice in self.map_choices:
            if choice["map_name"] == str(self.cfg["carla"]["map_name"]).rsplit("/", 1)[-1]:
                for row in choice["slot_choices"]:
                    row.update(self.slot_provider.availability(self.slot_provider.slots[row["slot_id"]]))

    def _evaluate_slot_occupancy(self, slot, polygon, parts, body_parts, frame):
        state = {'frame': frame}
        try:
            pose = self.parkin_target(slot, obstacles=parts+body_parts+self._slot_occupancy_explicit)
        except GoalBlocked as exc:
            unknown = ':unknown:' in exc.obstacle_id
            return dict(state, state='unknown' if unknown else 'occupied',
                        reason='geometry_unconfirmed' if unknown else 'parking_envelope_blocked',
                        obstacle_id=exc.obstacle_id)
        state['parking_goal'] = list(pose)
        footprint = parking_footprint(self.cfg['apa']['vehicle_geometry'], pose)
        static = {'state': 'free', 'reason': 'offline_static_clear', 'obstacle_id': ''}
        edge = {'state': 'free', 'obstacle_id': ''}
        for part in parts:
            if convex_polygons_overlap(polygon, part['footprint']):
                unknown = ':unknown:' in part['id']
                if edge['state'] != 'occupied':
                    edge.update(state='unknown' if unknown else 'occupied', obstacle_id=part['id'])
            if convex_polygons_overlap(footprint, part['footprint']):
                unknown = ':unknown:' in part['id']
                static.update(state='unknown' if unknown else 'occupied',
                              reason='geometry_unconfirmed' if unknown else 'parking_envelope_blocked',
                              obstacle_id=part['id'])
                if not unknown:
                    break
        dynamic = {'state': 'free', 'reason': 'clear', 'obstacle_id': ''}
        border = {'state': 'free', 'obstacle_id': ''}
        for part in body_parts:
            if convex_polygons_overlap(polygon, part['footprint']):
                border.update(state='occupied', obstacle_id=part['id'])
            if convex_polygons_overlap(footprint, part['footprint']):
                dynamic.update(state='occupied', reason='physical_overlap', obstacle_id=part['id'])
        if border['state'] == 'occupied' or edge['state'] == 'free':
            edge = border
        state.update(dynamic if dynamic['state'] == 'occupied' else static)
        state.update(edge_state=edge['state'], edge_obstacle_id=edge['obstacle_id'], static_frame=frame)
        return state

    def setup_collision_sensor(self) -> None:
        if self.world is None or self.carla is None or self.ego is None:
            return
        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
                self.collision_sensor.destroy()
            except Exception:
                pass
            self.collision_sensor = None
        try:
            bp = self.world.get_blueprint_library().find("sensor.other.collision")
            sensor = self.world.spawn_actor(bp, self.carla.Transform(), attach_to=self.ego)
            self_ref = weakref.ref(self)

            def on_collision(event: Any) -> None:
                bridge = self_ref()
                if bridge is not None:
                    bridge.handle_collision(event)

            sensor.listen(on_collision)
            self.collision_sensor = sensor
            print(f"[APA Bridge] collision sensor attached id={sensor.id}")
        except Exception as exc:
            print(f"[APA Bridge] collision sensor setup failed: {exc}", file=sys.stderr)

    def handle_collision(self, event: Any) -> None:
        other = getattr(event, "other_actor", None)
        other_desc = "unknown"
        try:
            if other is not None:
                other_desc = f"{other.type_id}#{other.id}"
        except Exception:
            pass

        self.latch_safety_stop(f"collision: {other_desc}")
        print(f"[APA Bridge] COLLISION with {other_desc}; stop latched until explicit reset", flush=True)

    def load_world_with_retry(self, map_name: str):
        timeout_sec = max(finite(self.cfg["carla"].get("timeout_sec"), 30.0), 30.0)
        self.client.set_timeout(timeout_sec)
        last_error = None
        for attempt in range(1, 4):
            try:
                print(f"[APA Bridge] loading map {map_name}, attempt {attempt}, timeout={timeout_sec}s", flush=True)
                world = self.client.load_world(map_name)
                print(f"[APA Bridge] loaded map {world.get_map().name}", flush=True)
                return world
            except RuntimeError as exc:
                last_error = exc
                print(f"[APA Bridge] load_world failed: {exc}", file=sys.stderr)
                time.sleep(2.0)
                try:
                    world = self.client.get_world()
                    if map_name in world.get_map().name:
                        print(f"[APA Bridge] map became ready after timeout: {world.get_map().name}")
                        return world
                except RuntimeError:
                    pass
        raise RuntimeError(f"failed to load CARLA map {map_name}: {last_error}")

    def find_or_spawn_ego(self):
        role_name = str(self.cfg["carla"].get("ego_role_name", "hero"))
        blueprint_id = self.cfg["apa"]["vehicle_geometry"]["blueprint"]
        auto_spawn = bool(self.cfg["carla"].get("auto_spawn_ego", True))
        bp = self.world.get_blueprint_library().find(blueprint_id)
        desired = self.approach_transform(self.target_slot)
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == role_name:
                if actor.type_id != blueprint_id:
                    if not auto_spawn:
                        raise RuntimeError("existing ego does not match the shared APA vehicle profile")
                    print(f"[APA Bridge] replace ego id={actor.id} {actor.type_id} with {blueprint_id}")
                    for sensor in self.world.get_actors().filter("sensor.*"):
                        parent = sensor.parent
                        if parent is not None and parent.id == actor.id:
                            sensor.destroy()
                    if not actor.destroy():
                        raise RuntimeError("failed to remove the previous APA ego")
                    break
                print(f"[APA Bridge] found ego id={actor.id}, reset to APA approach pose")
                self.reset_ego_actor(actor, desired)
                self.set_spectator_near(actor)
                return actor
        if not auto_spawn:
            return None
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", role_name)

        actor = None
        used_transform = None
        for transform in self.ego_spawn_candidates(desired):
            actor = self.world.try_spawn_actor(bp, transform)
            if actor is not None:
                used_transform = transform
                break
        if actor is None:
            reused = self.nearest_vehicle(desired, max_distance_m=12.0)
            if reused is not None:
                print(f"[APA Bridge] reuse nearest vehicle id={reused.id} as ego")
                self.reset_ego_actor(reused, desired)
                self.set_spectator_near(reused)
                return reused
            raise RuntimeError("failed to spawn ego near APA target slot")

        actor.apply_control(self.carla.VehicleControl(throttle=0.0, brake=1.0, manual_gear_shift=True, gear=1))
        self.set_spectator_near(actor)
        loc = used_transform.location if used_transform is not None else actor.get_transform().location
        print(f"[APA Bridge] spawned ego id={actor.id} x={loc.x:.2f} y={loc.y:.2f}")
        return actor

    def reset_ego_actor(self, actor: Any, transform: Any) -> None:
        try:
            actor.set_transform(transform)
            actor.set_target_velocity(self.carla.Vector3D(0.0, 0.0, 0.0))
            actor.set_target_angular_velocity(self.carla.Vector3D(0.0, 0.0, 0.0))
            actor.apply_control(
                self.carla.VehicleControl(
                    throttle=0.0,
                    brake=1.0,
                    steer=0.0,
                    hand_brake=True,
                    manual_gear_shift=True,
                    gear=1,
                )
            )
            loc = transform.location
            print(f"[APA Bridge] ego reset x={loc.x:.2f} y={loc.y:.2f} yaw={transform.rotation.yaw:.1f}")
        except Exception as exc:
            print(f"[APA Bridge] ego reset skipped: {exc}", file=sys.stderr)

    def ego_spawn_candidates(self, base_transform: Any) -> List[Any]:
        yaw = math.radians(finite(base_transform.rotation.yaw))
        forward = (math.cos(yaw), math.sin(yaw))
        right = (-math.sin(yaw), math.cos(yaw))
        offsets = [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.4),
            (1.0, 0.0, 0.2),
            (-1.0, 0.0, 0.2),
            (0.0, 1.0, 0.2),
            (0.0, -1.0, 0.2),
            (2.0, 0.0, 0.3),
            (-2.0, 0.0, 0.3),
            (0.0, 2.0, 0.3),
            (0.0, -2.0, 0.3),
            (3.5, 0.0, 0.4),
            (-3.5, 0.0, 0.4),
        ]
        result = []
        loc = base_transform.location
        for forward_m, right_m, up_m in offsets:
            x = loc.x + forward[0] * forward_m + right[0] * right_m
            y = loc.y + forward[1] * forward_m + right[1] * right_m
            z = self.sample_ground_z(x, y) + 0.55 + up_m
            result.append(
                self.carla.Transform(
                    self.carla.Location(x=x, y=y, z=z),
                    self.carla.Rotation(yaw=base_transform.rotation.yaw),
                )
            )
        return result

    def nearest_vehicle(self, transform: Any, max_distance_m: float):
        best = None
        best_distance = float(max_distance_m)
        target = transform.location
        for actor in self.world.get_actors().filter(self.cfg["apa"]["vehicle_geometry"]["blueprint"]):
            try:
                loc = actor.get_transform().location
            except Exception:
                continue
            distance = math.hypot(finite(loc.x) - finite(target.x), finite(loc.y) - finite(target.y))
            if distance < best_distance:
                best = actor
                best_distance = distance
        return best

    def apply_front_wheel_lock(self) -> None:
        """Apply the configured wheel lock and optional simulation drivetrain calibration."""
        if self.ego is None or not hasattr(self.ego, "get_physics_control"):
            return
        geometry = self.cfg["apa"]["vehicle_geometry"]

        max_deg = math.degrees(geometry.get("carla_max_steer_rad", geometry["max_steer_rad"]))
        try:
            physics = self.ego.get_physics_control()
            wheels = list(physics.wheels)
        except Exception as exc:
            print(f"[APA Bridge] front wheel lock skipped: {exc}", flush=True)
            return
        try:
            gears = list(physics.forward_gears)
            torque = list(physics.torque_curve)
            steering = list(physics.steering_curve)
        except Exception as exc:
            print(f"[APA Bridge] front wheel lock skipped, drivetrain kept: {exc}", flush=True)
            return
        if len(gears) < 1 or len(torque) < 2:
            print("[APA Bridge] drivetrain missing, respawn ego before wheel lock", flush=True)
            self._respawn_ego_from_blueprint()
            return
        changed = False
        clutch = self.cfg["apa"].get("drivetrain_clutch_strength")
        if clutch is not None:
            physics.clutch_strength = float(clutch)
            changed = True
        for wheel in wheels:
            current = abs(float(getattr(wheel, "max_steer_angle", 0.0)))
            if current <= 1.0:
                continue

            changed = changed or abs(current - max_deg) >= 0.5
            wheel.max_steer_angle = max_deg
        if not changed:
            return
        physics.wheels = wheels
        physics.forward_gears = gears
        physics.torque_curve = torque
        physics.steering_curve = steering
        try:
            self.ego.apply_physics_control(physics)
        except Exception as exc:
            if clutch is not None:
                raise RuntimeError("configured APA drivetrain calibration failed") from exc
            print(f"[APA Bridge] front wheel lock apply failed: {exc}", flush=True)
            return
        if clutch is not None and abs(float(self.ego.get_physics_control().clutch_strength) - float(clutch)) > 1.e-6:
            raise RuntimeError("CARLA did not apply the configured APA clutch strength")
        print(f"[APA Bridge] front wheel lock ±{max_deg:.1f} deg, clutch_strength={physics.clutch_strength}", flush=True)

    def _respawn_ego_from_blueprint(self) -> None:
        if getattr(self, "_respawning_ego", False) or self.ego is None or self.world is None:
            return
        self._respawning_ego = True
        try:
            transform = self.ego.get_transform()
            role_name = str(self.cfg["carla"].get("ego_role_name", "hero"))
            try:
                self.ego.destroy()
            except Exception:
                pass
            self.ego = None
            bps = self.world.get_blueprint_library().filter(str(self.cfg["carla"]["ego_blueprint_filter"]))
            if not bps:
                bps = self.world.get_blueprint_library().filter("vehicle.*")
            blueprint = bps[0]
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute("role_name", role_name)
            actor = self.world.try_spawn_actor(blueprint, transform)
            if actor is None:
                print("[APA Bridge] ego respawn failed", flush=True)
                return
            if hasattr(actor, "set_simulate_physics"):
                actor.set_simulate_physics(True)
            self.ego = actor
            print(f"[APA Bridge] respawned ego id={actor.id} with blueprint drivetrain", flush=True)
            self.apply_front_wheel_lock()
        finally:
            self._respawning_ego = False

    def set_spectator_near(self, actor: Any) -> None:
        try:
            spectator = self.spectator
            transform = actor.get_transform()
            yaw = math.radians(finite(transform.rotation.yaw))
            back_m = finite(self.cfg["apa"].get("spectator_back_m", 8.0), 8.0)
            up_m = finite(self.cfg["apa"].get("spectator_up_m", 6.0), 6.0)
            loc = transform.location
            spectator.set_transform(
                self.carla.Transform(
                    self.carla.Location(
                        x=loc.x - math.cos(yaw) * back_m,
                        y=loc.y - math.sin(yaw) * back_m,
                        z=loc.z + up_m,
                    ),
                    self.carla.Rotation(
                        pitch=finite(self.cfg["apa"].get("spectator_pitch_deg", -28.0), -28.0),
                        yaw=transform.rotation.yaw,
                    ),
                )
            )
        except Exception as exc:
            print(f"[APA Bridge] set spectator skipped: {exc}")

    def sample_ground_z(self, x: float, y: float, allow_query: bool = True, *, allow_held_query: bool = False) -> float:
        """Resolve a validated road level; uncached task queries require stationary scene setup."""
        x, y = required_number(x, "ground.x"), required_number(y, "ground.y")
        world_id = self.world.id
        if getattr(self, "_ground_world_id", None) != world_id:
            self._ground_map = (self._loaded_map if getattr(self, "_loaded_map_world_id", None) == world_id
                                else self.world.get_map())
            index = {}
            for waypoint in self._ground_map.generate_waypoints(2.0):
                point = waypoint.transform.location
                index.setdefault((math.floor(point.x/10), math.floor(point.y/10)), []).append(point)
            if not index:
                raise ValueError("地图没有可用道路高度，不能生成车辆初始位置")
            self._ground_index = index
            self._ground_world_id = world_id
            self._parking_ground_cache = {}
        bx, by = math.floor(x/10), math.floor(y/10)
        nearby = [point for dx in range(-3, 4) for dy in range(-3, 4)
                  for point in self._ground_index.get((bx+dx, by+dy), ())]
        if not nearby:
            raise ValueError(f"({x:.2f},{y:.2f}) 附近没有道路高度，不能生成车辆初始位置")
        distance = lambda point: math.hypot(point.x-x, point.y-y)
        nearest = min(nearby, key=distance)
        levels = [point for point in nearby if distance(point) <= distance(nearest)+1.0]
        if max(p.z for p in levels)-min(p.z for p in levels) > 2.0:
            ego = getattr(self, "ego", None)
            reference = ego.get_transform().location if ego is not None else None
            if reference is None or math.hypot(reference.x-x, reference.y-y) > 20.0:
                raise ValueError(f"({x:.2f},{y:.2f}) 存在多层道路，需要邻近自车确认路面高度")
            nearest = min(levels, key=lambda p: (abs(p.z-reference.z), distance(p)))
            if abs(nearest.z-reference.z) > 3.0:
                raise ValueError("自车与候选路面高度不匹配，不能选择道路层")
        if distance(nearest) > 20.0:
            key = (round(x, 3), round(y, 3), round(nearest.z))
            if key in self._parking_ground_cache:
                return self._parking_ground_cache[key]
            if not allow_query:
                cached = [(math.hypot(k[0]-x, k[1]-y), z)
                          for k, z in self._parking_ground_cache.items()
                          if abs(k[2]-nearest.z) < 2.0]

                return min(cached)[1] if cached else required_number(nearest.z, "ground.z")
            if getattr(self, "parking_enabled", False):
                if not allow_held_query or not self._scene_setup_is_stationary():
                    raise ValueError("泊车执行中不能查询未缓存的停车场地面")
            surfaces = {self.carla.CityObjectLabel.Roads, self.carla.CityObjectLabel.Ground,
                        self.carla.CityObjectLabel.Terrain}
            hits = self.world.cast_ray(self.carla.Location(x, y, nearest.z+20.0),
                                       self.carla.Location(x, y, nearest.z-5.0))
            heights = [required_number(hit.location.z, "ground.ray_z") for hit in hits
                       if hit.label in surfaces]
            if (not heights or max(heights)-min(heights) > 2.0 or
                    abs(max(heights)-nearest.z) > 3.0):
                raise ValueError(f"({x:.2f},{y:.2f}) 没有唯一可确认的停车场路面")
            self._parking_ground_cache[key] = max(heights)
            return self._parking_ground_cache[key]
        waypoint = self._ground_map.get_waypoint(
            self.carla.Location(x=x, y=y, z=nearest.z), project_to_road=True,
            lane_type=self.carla.LaneType.Any)
        if waypoint is None:
            raise ValueError(f"({x:.2f},{y:.2f}) 道路高度投影失败")
        point = waypoint.transform.location
        if distance(point) > distance(nearest)+2.0 or abs(point.z-nearest.z) > 2.0:
            raise ValueError(f"({x:.2f},{y:.2f}) 道路投影偏离附近道路，不能选择路面高度")
        return required_number(point.z, "ground.z")

    def settle_ego_for_geometry(self):
        """Hold the spawned/reset car on its road before selecting the Z slice."""
        stable, previous = 0, None
        step = float(self.cfg["carla"].get("fixed_delta_seconds", 0.02))
        for _ in range(math.ceil(3.0/step)):
            self.apply_sync_hold()
            snapshot = self.tick_world()
            state = snapshot.find(self.ego.id)
            if state is None:
                raise RuntimeError("车辆初始化没有同帧反馈，不能准备地图障碍")
            z = state.get_transform().location.z
            velocity = state.get_velocity()
            speed = math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
            stable = stable+1 if previous is not None and abs(z-previous) < 0.002 and speed < 0.03 else 0
            if stable >= 5:
                return
            previous = z
        raise RuntimeError("车辆初始化未稳定落到路面，不能准备地图障碍")

    def rear_axle_offset_m(self) -> float:
        """Positive longitudinal distance from the rear axle to Actor origin."""
        value = self.cfg["apa"].get("rear_axle_offset_m")
        if value is None:

            value = read_vehicle_profile(DEFAULT_VEHICLE_PROFILE_PATH)["rear_axle_offset_m"]
        return required_number(value, "rear_axle_offset_m")

    def rear_axle_pose(self, actor_transform: Any) -> Tuple[float, float, float]:
        """Normalize a CARLA Actor pose once at the Bridge/algorithm boundary."""
        if actor_transform is None:
            return 0.0, 0.0, 0.0
        yaw = yaw_rad_from_carla_deg(finite(actor_transform.rotation.yaw))
        x, y = offset_reference_xy(
            finite(actor_transform.location.x), finite(actor_transform.location.y),
            yaw, -self.rear_axle_offset_m(),
        )
        return x, y, yaw

    def parkin_target(self, slot: Dict[str, Any], obstacles=()) -> Tuple[float, float, float]:
        """Generate the rear-axle target supplied to the planner."""
        yaw = (required_number(slot["apa_target_yaw_rad"], "slot.apa_target_yaw_rad")
               if "apa_target_yaw_rad" in slot else math.radians(required_number(slot["goal_yaw"], "slot.goal_yaw")))
        reference = slot.get("goal_reference_point", "slot_center")
        if reference == REAR_AXLE_REFERENCE_POINT:
            return fit_goal(slot, self.cfg["apa"]["vehicle_geometry"],
                            (required_number(slot["goal_x"], "slot.goal_x"),
                             required_number(slot["goal_y"], "slot.goal_y")), yaw, adjust=False)
        if reference != "slot_center":
            raise ValueError("slot.goal_reference_point 必须为 slot_center 或 rear_axle_center")
        points = [(slot[f"slot_pt{i}_x"], slot[f"slot_pt{i}_y"]) for i in range(1, 5)]
        entry = int(slot.get("slot_entry", 1)) - 1
        p1, p2, p3, p4 = [points[(entry + i) % 4] for i in range(4)]
        kind = apa_slot_kind_code(slot)
        if kind == 2:

            if "apa_target_yaw_rad" not in slot:
                heading = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                approach_yaw = slot.get("apa_start_rear_yaw_rad", yaw)
                yaw = heading if math.cos(heading - approach_yaw) >= 0.0 else heading + math.pi
        else:
            outward = math.atan2(p1[1] + p2[1] - p3[1] - p4[1],
                                 p1[0] + p2[0] - p3[0] - p4[0])
            if "apa_target_yaw_rad" not in slot:

                reverse_in = kind in (1, 3)
                yaw = outward if reverse_in else outward + math.pi
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        geometry = self.cfg["apa"]["vehicle_geometry"]
        front, rear, width = [required_number(geometry[key], f"apa.vehicle_geometry.{key}")
                              for key in ("rear_to_front_m", "rear_to_back_m", "width_m")]
        if min(front, rear, width) <= 0.0:
            raise ValueError("APA 车辆尺寸必须为正数")
        if kind == 2:
            return parallel_centered_goal(slot, geometry, yaw, obstacles)
        return centered_goal(slot, geometry, yaw)

    def parkout_target(self, slot: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
        """Resolve one rear-axle exit goal for validation, publication and UI."""
        slot = self.target_slot if slot is None else slot
        for fields, source in (
            (("parkout_target_x", "parkout_target_y", "parkout_target_yaw_rad"), "configured"),
            (("apa_start_rear_x", "apa_start_rear_y", "apa_start_rear_yaw_rad"), "configured_start"),
        ):

            captured = getattr(self, "parkout_return_pose", None)
            if source == "configured_start" and captured and captured["slot_id"] == int(slot.get("slot_id", 0)):
                return {key: captured[key] for key in ("x", "y", "yaw_rad", "source")}
            values = [slot.get(key) for key in fields]
            if all(type(value) in (int, float) and math.isfinite(value) for value in values):
                return {"x": values[0], "y": values[1], "yaw_rad": values[2], "source": source}
        return None

    def approach_pose(self, slot: Dict[str, Any]) -> Dict[str, float]:
        """The configured slot start is a CARLA-world rear-axle pose."""
        if all(key in slot for key in ("apa_start_rear_x", "apa_start_rear_y", "apa_start_rear_yaw_rad")):
            rear_x = finite(slot.get("apa_start_rear_x"))
            rear_y = finite(slot.get("apa_start_rear_y"))
            rear_yaw = finite(slot.get("apa_start_rear_yaw_rad"))
        else:
            gx = finite(slot.get("goal_x"))
            gy = finite(slot.get("goal_y"))
            ex = finite(slot.get("slot_entry_pt_x"), finite(slot.get("slot_pt1_x")))
            ey = finite(slot.get("slot_entry_pt_y"), finite(slot.get("slot_pt1_y")))
            dx = ex - gx
            dy = ey - gy
            norm = math.hypot(dx, dy)
            if norm < 0.2:
                yaw = math.radians(finite(slot.get("goal_yaw")) + 180.0)
                ox, oy = math.cos(yaw), math.sin(yaw)
            else:
                ox, oy = dx / norm, dy / norm
            distance = finite(self.cfg["apa"].get("approach_distance_m", 4.5), 4.5)
            rear_x = ex + ox * distance
            rear_y = ey + oy * distance
            yaw_deg = wrap_yaw_deg(yaw_deg_from_vector(-ox, -oy))
            if str(slot.get("slot_kind", "")).lower() == "parallel":
                yaw_deg = finite(slot.get("goal_yaw"), yaw_deg)
            rear_yaw = math.radians(yaw_deg)

        rear_yaw = finite(slot.get("apa_start_rear_yaw_rad"), rear_yaw)
        return {"x": rear_x, "y": rear_y, "yaw_deg": math.degrees(rear_yaw)}

    def parking_query_radius(self, provider=None) -> float:
        provider = self.slot_provider if provider is None else provider
        radius = required_number(provider.data.get("parking_query_radius_m",
            self.cfg["apa"].get("parking_query_radius_m", 24.0)), "parking_query_radius_m")
        if not 16 <= radius <= 30:
            raise ValueError("泊车查询半径须为 16–30 米")
        return radius

    def configure_planning_window(self):
        """Freeze a query window; map coverage comes only from the offline artifact."""
        self._neighbor_boundary_key = None
        if self.ego is not None:
            x, y, _ = self.rear_axle_pose(self.ego.get_transform())
        else:

            x = self.target_slot.get("apa_start_rear_x", self.target_slot["slot_entry_pt_x"])
            y = self.target_slot.get("apa_start_rear_y", self.target_slot["slot_entry_pt_y"])
        radius = self.parking_query_radius()

        self._planning_query_region = {"name": "离线底图查询窗口", "scope": "offline_map_query",
            "bounds": [x-radius, y-radius, x+radius, y+radius], "radius_m": radius}

    def offline_map_query(self, bounds, low, high, *, planning_center=None, cell_bands=None):
        atlas = getattr(self, "_offline_parking_map", None)
        if atlas is not None:
            if cell_bands is None:
                parts, summary = atlas.query(bounds, low, high, planning_center=planning_center)
            else:
                parts, summary = atlas.query_cells(bounds, low, high, cell_bands, planning_center=planning_center)
            hard, policy = offline_collision_parts(parts)
            return hard, dict(summary, **policy)
        a, b, c, d = bounds
        return [{"id": "offline-map:unknown:unavailable", "velocity_x": 0.0, "velocity_y": 0.0,
            "z_min": low, "z_max": high,
            "footprint": [{"x": x, "y": y} for x, y in ((a,b),(c,b),(c,d),(a,d))]}], {"missing_sources": 0, "unavailable": True}

    def load_offline_parking_map(self, force=False):
        """Read a map-level artifact. No ray sampling or cache mutation here."""
        started = time.monotonic()
        name = self._map_name.rsplit("/", 1)[-1]

        directory = Path(__file__).resolve().parents[2] / "scenarios" / "apa" / "parking_maps"
        path = directory / f"{name}.json.gz"
        try:
            stat = path.stat()
            identity = (str(path), stat.st_mtime_ns, stat.st_size, self._environment_world_key)
            reused = not force and getattr(self, "_offline_map_identity", None) == identity
            if not reused:
                data, revision = read_map(path, name, self.slot_provider.native_map.sha256,
                    inventory_digest(self.environment_obstacles), self.client.get_server_version(),
                    LOW_OBSTACLE_PROFILE_VERSION, compatible_sampling_versions=(9,))
                self._offline_parking_map = OfflineParkingMap(data, obstacle_at_height,
                    clip_xy_polygon, obstacle_needs_profile, self.vehicle_profile_id,
                    hashlib.sha256(self.slot_provider.path.read_bytes()).hexdigest(),
                    vehicle_geometry=self.cfg["apa"]["vehicle_geometry"])
                self._offline_map_revision = revision
                self._offline_map_identity = identity
                self._scene_compactor = StaticSceneCompactor()
                self._slot_occupancy_cache.clear()
                self._slot_occupancy_explicit = None
            atlas = self._offline_parking_map
            self._map_preparation = {"available": True, "current_available": True, "state": "ready",
                "sampling_version": atlas.data["sampling_version"],
                "sampling_geometry_state": ("legacy_dilated_requires_resurvey"
                    if atlas.data["sampling_version"] == 9 else "raw_hit_cells"),
                "scope": "offline_map", "map_name": name, "artifact": path.name,
                "revision": self._offline_map_revision, "coverage_cells": len(atlas.cells),
                "coverage_area_m2": len(atlas.cells)*64.0, "static_sources": len(atlas.sources),
                "coverage_quality": {k: v for k, v in atlas.data.get("coverage_quality", {}).items() if k != "missing"},
                "interior_filter": {"built": atlas.interior.policy is not None,
                    "height_records": len(atlas.interior.bands), "fallback": atlas.interior.reason},
                "active_region": self._planning_query_region, "reused": reused}
            self._obstacle_profile_summary = {"state": "ready", "pending": 0, "cache_scope": "offline_map",
                "cache_persistent": True, "rays": 0, "elapsed_sec": time.monotonic()-started}
            self._geometry_progress = {"state": "ready", "unit": "offline_map_load", "ray_queries": 0}
            print("[APA offline map] " + json.dumps(self._map_preparation, ensure_ascii=False), flush=True)
        except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
            self._offline_parking_map = None
            self._offline_map_identity = None
            self._offline_map_revision = ""
            self._map_preparation = {"available": True, "current_available": True, "state": "unavailable",
                "scope": "offline_map", "map_name": name, "artifact": path.name, "message": str(exc),
                "active_region": self._planning_query_region}
            self._obstacle_profile_summary = {"state": "failed", "pending": 1, "rays": 0, "cache_scope": "offline_map"}
            self._geometry_progress = {"state": "failed"}
            print(f"[APA offline map] unavailable: {exc}", flush=True)

    def pose_transform(self, pose: Dict[str, float], *, allow_held_query: bool = False):
        rear_yaw = math.radians(pose["yaw_deg"])
        x, y = offset_reference_xy(pose["x"], pose["y"], rear_yaw, self.rear_axle_offset_m())
        z = self.sample_ground_z(x, y, allow_held_query=allow_held_query) + 0.45
        return self.carla.Transform(
            self.carla.Location(x=x, y=y, z=z), self.carla.Rotation(yaw=math.degrees(rear_yaw))
        )

    def approach_transform(self, slot: Dict[str, Any]):
        return self.pose_transform(self.approach_pose(slot))

    def spawn_occupied_vehicles(self) -> None:
        bps = self.world.get_blueprint_library().filter("vehicle.*")
        candidates = [bp for bp in bps if "truck" not in bp.id and "bus" not in bp.id]
        if not candidates:
            return
        max_count = int(self.cfg["apa"].get("max_parked_vehicles", 12))
        target_id = physical_slot_id(self.target_slot)
        for slot in self.slot_provider.physical_slots():
            if len(self.spawned_parked) >= max_count:
                break
            if int(slot.get("slot_id", -1)) == target_id or not bool(slot.get("slot_occupied", False)):
                continue
            xs = [finite(slot.get(key)) for key in ("slot_pt1_x", "slot_pt2_x", "slot_pt3_x", "slot_pt4_x")]
            ys = [finite(slot.get(key)) for key in ("slot_pt1_y", "slot_pt2_y", "slot_pt3_y", "slot_pt4_y")]
            transform = self.carla.Transform(
                self.carla.Location(x=sum(xs) / 4.0, y=sum(ys) / 4.0, z=self.sample_ground_z(sum(xs) / 4.0, sum(ys) / 4.0) + 0.5),
                self.carla.Rotation(yaw=finite(slot.get("goal_yaw"))),
            )
            actor = self.world.try_spawn_actor(candidates[len(self.spawned_parked) % len(candidates)], transform)
            if actor is not None:
                actor.apply_control(self.carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                self.spawned_parked.append(actor)
        print(f"[APA Bridge] parked vehicles: {len(self.spawned_parked)}")

    def stop(self, *_args: Any) -> None:
        self.pending_map_config = None
        self.running = False

    def next_sequence(self, topic: str) -> int:
        value = self.sequence.get(topic, 0) + 1
        self.sequence[topic] = value
        return value

    def make_header(self, topic: str, snapshot: Any = None) -> Dict[str, Any]:
        frame = int(getattr(snapshot, "frame", 0)) if snapshot is not None else 0
        dt = float(self.cfg["carla"].get("fixed_delta_seconds", 0.02))
        sim_time = 0.0
        if snapshot is not None and getattr(snapshot, "timestamp", None) is not None:
            sim_time = finite(snapshot.timestamp.elapsed_seconds)
            dt = finite(snapshot.timestamp.delta_seconds, dt)
        return {
            "protocol": PROTOCOL,
            "protocol_version": PROTOCOL_VERSION,
            "message_type": topic,
            "frame_id": frame,
            "sequence": self.next_sequence(topic),
            "sim_time_sec": sim_time,
            "delta_time_sec": dt,
            "timestamp_unix_ms": int(time.time() * 1000),
            "source": "carla_bridge_apa",
            "map_name": getattr(self, "_map_name", "") if self.world is not None else "",
            "ego_role_name": self.cfg["carla"].get("ego_role_name", "hero"),
            "coordinate_frame": "carla_world",
        }

    def publish(self, topic: str, payload: Dict[str, Any], snapshot: Any = None) -> None:
        envelope = {"header": self.make_header(topic, snapshot), "payload": json_safe(payload)}
        data = json.dumps(envelope, ensure_ascii=False, separators=(",", ":"))
        self.pub.send_multipart([topic.encode("utf-8"), data.encode("utf-8")])
        if topic in (EGO_STATE_TOPIC, APA_PLAN_REQUEST_TOPIC):
            self.scene_diagnostics.observe(topic, envelope,
                getattr(self, "_feedback_scene_provenance", {}))

    def tick_world(self, timeout_sec: Optional[float] = None):
        self.update_overhead_camera()
        recorder = getattr(self, "calibration_recorder", None)
        before = self.world.get_snapshot() if recorder is not None else None
        if self.cfg["carla"].get("sync_mode", True):
            timeout = self.cfg["carla"].get("runtime_rpc_timeout_sec", 2.0) if timeout_sec is None else timeout_sec
            frame = self.world.tick(timeout)
            snapshot = self.world.get_snapshot()
            if timeout_sec is not None and snapshot.frame != frame:
                raise RuntimeError("another client advanced CARLA during safety stopping")
        else:
            snapshot = self.world.wait_for_tick()
        if recorder is not None:
            try:

                command = self.ego.get_control()
                recorder.record(before, snapshot, self.ego.id, command,
                                bool(self.safety_stop.reason or self.recovery_stop.reason))
            except (OSError, ValueError) as exc:
                self._calibration_record_error = str(exc)
                print(f"[APA Bridge] calibration recording disabled: {exc}", file=sys.stderr)
                try:
                    recorder.close(complete=False)
                except OSError:
                    pass
                self.calibration_recorder = None
        return snapshot

    def run(self) -> None:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        try:

            self.setup_zmq()
            if self.map_bootstrap_command:
                command = self.map_bootstrap_command
                self.map_transition = dict(command, state="loading", ok=True,
                    source_session_id=command["session_id"],
                    message=f"正在准备 {self.cfg['carla']['map_name']}，车辆保持驻车")
                self.restart_carla_for_map()
            self.connect_carla()
            if self.map_bootstrap_command:
                self.complete_map_change()
            while self.running:
                started = time.monotonic()
                if self.dry_run:
                    self.publish(APA_PLAN_REQUEST_TOPIC, self.plan_request_payload(), None)
                    time.sleep(1.0)
                    continue
                try:
                    self._run_frame()
                except InvalidEgoFeedback:

                    self.publish_status_if_needed(None, read_vehicle=False)
                except RuntimeError as exc:

                    self.latch_safety_stop(f"CARLA RPC failed: {exc}")

                    self.publish_status_if_needed(None, read_vehicle=False)
                self.pace(started)
        except Exception as exc:
            if self.map_bootstrap_command:
                self.fail_map_change(self.map_bootstrap_command,
                    f"地图加载失败：{exc}。请检查环境日志后重新启动环境。")
            raise
        finally:
            if getattr(self, "ego", None) is not None and getattr(self, "carla", None) is not None:
                try:
                    self.shutdown_safely()
                except Exception as exc:
                    self.safety_stop.unconfirmed(f"exit brake failed: {exc}")
            if self.pending_map_config is not None and self.safety_stop.state != "stopped":
                self.fail_map_change(self.pending_map_command,
                    "未确认车辆停稳，已取消地图切换；请检查环境日志后重新启动环境。")
                self.pending_map_config = None
            recorder = getattr(self, "calibration_recorder", None)
            if recorder is not None:
                try:
                    recorder.close()
                except OSError as exc:
                    print(f"[APA Bridge] cannot finalize calibration recording: {exc}", file=sys.stderr)
            self.restore_settings()
            self.pub.close(0)
            self.sub.close(0)
            self.command_sub.close(0)

    def _run_frame(self) -> None:
        self.process_zmq_messages()
        snapshot = self.world.get_snapshot()
        if self.ego is None:
            return
        if self.recovery_stop.reason and not self.safety_stop.reason:
            self.observe_safety_stop(snapshot, self.recovery_stop)
            if self.recovery_stop.state == "unconfirmed":
                self.latch_safety_stop("recovery_standstill_unconfirmed")
        feedback = None if self.safety_stop.reason else self.ego_payload(snapshot)
        ready = not self.parking_enabled
        if self.safety_stop.reason:
            snapshot = self.advance_safety_stop(snapshot)
            ready = False
        elif self.parking_enabled:
            if time.monotonic() < self.ignore_control_until:
                self.apply_sync_hold()
                self.publish(EGO_STATE_TOPIC, feedback, snapshot)
                ready = True
            else:
                ready = self.wait_for_control(snapshot, feedback)
        else:
            self.apply_manual_hold()
            self.publish(EGO_STATE_TOPIC, feedback, snapshot)
        if ready and self.running and not self.safety_stop.reason:
            snapshot = self.tick_world()
            self.ego_payload(snapshot)
            if self.parking_enabled:
                self.record_actual_path()
        self.publish_status_if_needed(snapshot)
        if self.safety_stop.reason:
            return
        self.draw_trajectories()
        if bool(self.cfg["apa"].get("draw_debug", False)):
            self.draw_debug()
        if bool(self.cfg["apa"].get("follow_spectator", True)):
            self.set_spectator_near(self.ego)

    def pace(self, started: float) -> None:
        if not self.cfg["carla"].get("realtime_pacing", True):
            return
        fixed_delta = finite(self.cfg["carla"].get("fixed_delta_seconds"), 0.02)
        remaining = fixed_delta - (time.monotonic() - started)
        if remaining > 0:
            time.sleep(remaining)

    def restore_settings(self) -> None:
        for attribute in ("rear_camera", "overhead_camera"):
            camera = getattr(self, attribute, None)
            if camera is not None:
                try:
                    try:
                        camera.stop()
                    finally:
                        camera.destroy()
                except Exception:
                    pass
                setattr(self, attribute, None)
        if getattr(self, "camera_http", None) is not None:
            try:
                self.camera_http.shutdown()
                self.camera_http.server_close()
            except Exception:
                pass
            self.camera_http = None
        if self.collision_sensor is not None:
            try:
                self.collision_sensor.stop()
                self.collision_sensor.destroy()
            except Exception:
                pass
            self.collision_sensor = None
        if self.world is None:
            return
        if self.safety_stop.reason and self.safety_stop.state != "stopped":
            print("[APA safety] standstill unconfirmed; retain synchronous world settings", flush=True)
            return
        for actor in self.spawned_parked:
            try:
                if not actor.destroy():
                    print(f"[APA Bridge] cannot remove owned parked vehicle {actor.id}", flush=True)
            except RuntimeError as exc:
                print(f"[APA Bridge] cannot remove owned parked vehicle {actor.id}: {exc}", flush=True)
        self.spawned_parked.clear()
        if self.original_settings is None:
            return
        try:
            settings = self.world.get_settings()
            settings.synchronous_mode = bool(self.original_settings["synchronous_mode"])
            settings.fixed_delta_seconds = self.original_settings["fixed_delta_seconds"]
            self.world.apply_settings(settings)
        except Exception as exc:
            print(f"[APA Bridge] restore settings failed: {exc}", file=sys.stderr)

    def process_zmq_messages(self) -> None:
        self.process_apa_commands()
        for _ in range(100):
            try:
                parts = self.sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) != 2:
                if parts and parts[0] == CONTROL_TOPIC.encode() and self.parking_enabled:
                    self.fail_sync("invalid control multipart frame count")
                continue
            topic_b, payload_b = parts
            topic = topic_b.decode("utf-8", errors="replace")
            try:
                message = json.loads(payload_b.decode("utf-8", errors="replace"))
            except json.JSONDecodeError:
                if topic == CONTROL_TOPIC and self.parking_enabled:
                    self.fail_sync("malformed control JSON")
                continue
            if not isinstance(message, dict) or not isinstance(message.get("payload"), dict):
                if topic == CONTROL_TOPIC and self.parking_enabled:
                    self.fail_sync("invalid control JSON object")
                continue
            payload = message["payload"]
            if topic == CONTROL_TOPIC:
                identity = self.sync_metadata()
                if any(payload.get(key) != identity[key] for key in ("session_id", "task_id")):
                    continue
                self.last_algorithm_rx = time.monotonic()
                self.accept_control(payload)
                if self.last_control_payload is payload:
                    self.scene_diagnostics.observe_control(payload)
            elif topic == DRIVER_VEHICLE_TOPIC:
                self.apply_driver_vehicle_message(payload)
            elif topic == APA_PLAN_RESULT_TOPIC:
                identity = self.sync_metadata()
                replan = payload.get("replan")
                if (all(payload.get(k) == identity[k] for k in ("session_id", "task_id")) and
                        isinstance(replan, dict) and all(type(replan.get(k)) is int for k in ("recovery_id", "path_version")) and
                        replan["recovery_id"] >= self._recovery_id and replan["path_version"] >= self._path_version):
                    self.last_algorithm_rx = time.monotonic()
                    self.last_plan_result = payload
                    if payload.get("success") and len(extract_plan_path_xy(payload, 100)) >= 2:
                        self.last_accepted_plan_result = payload
                        history = self.accepted_plan_history
                        if history and history[-1]["replan"]["path_version"] == replan["path_version"]:
                            history[-1] = payload
                        else:
                            history.append(payload)

    def process_apa_commands(self) -> None:
        for _ in range(20):
            try:
                parts = self.command_sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) != 2:
                continue
            topic_b, payload_b = parts
            topic = topic_b.decode("utf-8", errors="replace")
            if topic not in {APA_COMMAND_TOPIC, DRIVER_VEHICLE_TOPIC}:
                continue
            try:
                message = json.loads(payload_b.decode("utf-8", errors="replace"))
            except json.JSONDecodeError:
                continue
            if not isinstance(message, dict):
                continue
            payload = message.get("payload", {})
            if not isinstance(payload, dict):
                continue
            if topic == DRIVER_VEHICLE_TOPIC:
                self.apply_driver_vehicle_message(payload)
            else:
                self.handle_task_command(payload)

    def stop_parking_task(self, clear_history: bool = False) -> None:
        """Invalidate old controls before changing the car or the task geometry."""
        self.parking_enabled = False
        self.apa_phase = "manual"
        self.planning_only = False
        self.task_id += 1
        self._task_parkin_goal = None
        self._sync_task_key = (False, self.parking_action)
        self._last_plan_sim_time = None
        self._pending_control = None
        self.reset_recovery()
        self._control_ack = False
        self._sync_fault = self.safety_stop.reason if self.safety_stop.reason != "cancelled" else ""
        self._feedback_cache = None
        self.last_control_payload = None
        self.last_algorithm_rx = 0.0
        self.last_control_id = 0
        self.last_mapped_throttle = 0.0
        self.last_mapped_brake = 1.0
        self.manual_throttle = self.manual_steer = 0.0
        self.manual_brake = 1.0
        self.manual_gear = self.last_applied_gear = "P"
        self.manual_hand_brake = True
        self.manual_ignition = "ON"
        self.manual_last_input = time.monotonic()
        self.applied_manual_throttle = self.applied_manual_steer = 0.0
        self.applied_manual_brake = 1.0
        self.current_apa_gear_pos = 3
        self.ego.set_autopilot(False)
        self.apply_sync_hold()
        if clear_history:
            self.history_revision += 1
            self.last_plan_result = None
            self.clear_trajectory_history()
            self._speed_sample_time = None
            self._speed_sample = self._accel_filtered = 0.0

    def reset_parking_task(self, payload: Dict[str, Any]) -> None:
        self.publish_task_progress(payload, "confirming_stop", "正在确认车辆停稳")
        with self.safety_stop.lock:
            if self.safety_stop.reason:
                self.observe_safety_stop(self.world.get_snapshot())
                if self.safety_stop.state != "stopped":
                    raise ValueError("保护停车尚未确认停稳，暂不能重置或移动车辆")
            safety_revision = self.safety_stop.revision
        slot = self.target_slot
        pose = self.initial_pose or self.approach_pose(slot)
        mode = self.initial_pose_mode
        if payload.get("action") == "configure":
            slot, mode, pose = self.resolve_scene_setup(payload, self.slot_provider)
        if not self._scene_setup_is_stationary():
            raise ValueError("请先停稳车辆，再应用设置或重置")
        self.apply_sync_hold()
        transform = self.pose_transform(pose, allow_held_query=True)
        self.publish_task_progress(payload, "applying_pose", "正在应用车位和车辆起点")
        self.stop_parking_task(clear_history=True)

        self.ego.set_target_velocity(self.carla.Vector3D())
        self.ego.set_target_angular_velocity(self.carla.Vector3D())
        self.ego.set_transform(transform)
        self.target_slot = slot
        self.cfg["apa"]["slot_id"] = int(slot["slot_id"])
        self.initial_pose = dict(pose)
        self.initial_pose_mode = mode
        self.reset_camera_frames()
        self.parkout_return_pose = None
        self.publish_task_progress(payload, "settling", "正在等待车辆在路面稳定")
        self.settle_ego_for_geometry()
        self.configure_planning_window()
        self.publish_task_progress(payload, "checking_geometry", "正在读取地图缓存并更新场景")
        self.refresh_environment_obstacles()
        self.publish_task_progress(payload, "finalizing", "地图检查结束，正在确认设置完成")
        self.apply_manual_hold()
        self.tick_world()
        with self.safety_stop.lock:
            if not self.safety_stop.clear(safety_revision):
                raise ValueError("重置期间又收到停车事件，车辆继续保持保护停车")
            self._sync_fault = ""
        self._feedback_cache = None
        self.ignore_control_until = time.monotonic() + 0.3
        self.set_spectator_near(self.ego)

    def reload_parking_map(self, command):
        """Reload the offline artifact while the vehicle is held."""
        if self.parking_enabled or abs(self.current_speed()) > self.safety_stop.speed_limit:
            raise ValueError("请先退出 APA 并停稳，再重新加载离线底图")
        if self.safety_stop.reason and self.safety_stop.state != "stopped":
            raise ValueError("请等待车辆确认停稳")
        self.stop_parking_task()
        self.publish_task_progress(command, "loading_map", "正在重新加载离线泊车底图，车辆保持驻车")
        self.refresh_environment_obstacles(refresh_static=True)
        self._feedback_cache = None
        if self._offline_parking_map is None:
            raise ValueError(self._map_preparation.get("message") or "离线底图不可用，请先离线构建")

    def publish_task_progress(self, command, stage, message):
        if not command.get("event_id"):
            return
        self.last_task_command = {
            "event_id": command["event_id"], "action": command.get("action"),
            "map_name": command.get("map_name", self.cfg["carla"]["map_name"]),
            "source_session_id": command.get("session_id"),
            "source_task_id": command.get("task_id"), "task_id": self.task_id,
            "state": "running", "stage": stage, "ok": True, "message": message,
        }
        self.last_status_publish = 0.0
        self.publish_status_if_needed(self.world.get_snapshot())

    def handle_task_command(self, payload: Dict[str, Any]) -> None:
        event_id = str(payload.get("event_id") or "")
        if event_id and event_id in self._task_command_results:
            cached = self._task_command_results[event_id]
            if (cached.get("source_session_id") != payload.get("session_id") or
                    cached.get("source_task_id") != payload.get("task_id") or
                    cached.get("action") != payload.get("action")):
                self.last_task_command = dict(event_id=event_id, ok=False,
                    action=payload.get("action"), source_task_id=payload.get("task_id"),
                    task_id=self.task_id, message="操作事件号与原操作身份不一致")
            else:
                self.last_task_command = cached
            self.last_status_publish = 0.0
            return
        result = {"event_id": event_id, "ok": False, "message": "",
                  "source_session_id": payload.get("session_id")}
        if payload.get("action") == "configure" and "map_name" in payload:
            result.update(map_name=payload["map_name"], source_session_id=payload.get("session_id"))
        try:
            if self.pending_map_config is not None:
                raise ValueError("地图正在切换，请等待新会话就绪")
            if payload.get("session_id") != self.session_id:
                raise ValueError("Bridge 会话已变化，请刷新状态后重试")
            action = payload.get("action")
            prior = self._task_command_results.get(str(payload.get("cancel_pending_event_id", "")), {})
            cancels_transition = (action == "cancel" and prior and
                prior.get("task_id") == self.task_id and
                prior.get("source_task_id") == payload.get("task_id"))
            if payload.get("task_id") != self.task_id and not cancels_transition:
                raise ValueError("车辆任务已变化，请刷新状态后重试")
            if self.ego is None or self.carla is None:
                raise ValueError("车辆尚未就绪")
            if action in ("reset", "configure"):
                requested_map = payload.get("map_name", self.cfg["carla"].get("map_name", ""))
                if action == "configure" and requested_map != self.cfg["carla"].get("map_name", ""):
                    self.request_map_change(payload)
                    result.update(state="loading", map_name=requested_map,
                                  source_session_id=self.session_id,
                                  message=f"正在加载 {requested_map}，车辆保持驻车")
                else:
                    self.reset_parking_task(payload)
                    result["message"] = "已重置任务，车辆回到初始位置并保持驻车"
            elif action == "prepare_map":
                self.reload_parking_map(payload)
                result["message"] = "离线泊车底图已加载，车辆保持驻车"
            elif action == "cancel":
                self.latch_safety_stop("cancelled")
                self.stop_parking_task()
                self.apply_manual_hold()
                result["message"] = ("已退出 APA，正在制动并确认停稳" if self.safety_stop.reason == "cancelled"
                                     else "已退出 APA，保护停车仍锁存；确认停稳后请显式重置")
            elif action in ("plan_only", "park_in", "park_out"):
                if not self.apply_apa_command(payload):
                    raise ValueError("无法开始泊车：请停稳车辆，并确认车位具有对应的泊出目标")
                result["message"] = {"plan_only": "已开始规划，车辆保持驻车",
                                     "park_in": "已开始泊入", "park_out": "已开始泊出"}[action]
            else:
                raise ValueError("不支持的 APA 操作")
            result["ok"] = True
        except Exception as exc:
            result["message"] = str(exc)
            print(f"[APA Bridge] task command failed: {exc}", flush=True)
        self.sync_metadata()
        result.update(action=payload.get("action"), source_task_id=payload.get("task_id"), task_id=self.task_id)
        self.last_task_command = result
        if result.get("state") == "loading" and result["ok"]:

            self.map_transition = dict(result)
        if event_id:
            self._task_command_results[event_id] = result
            while len(self._task_command_results) > 32:
                del self._task_command_results[next(iter(self._task_command_results))]
        self.last_status_publish = 0.0

    def apply_apa_command(self, payload: Dict[str, Any]) -> bool:
        with self.safety_stop.lock:
            if self.safety_stop.reason:
                if self.safety_stop.reason != "cancelled" or self.safety_stop.state != "stopped":
                    raise ValueError("保护停车未解除；请等待停稳并显式重置后再开始")
        action_value = payload.get("parking_action", payload.get("action", 0))
        planning_only = isinstance(action_value, str) and action_value.lower() == "plan_only"
        if isinstance(action_value, str):
            action_value = {"plan_only": 1, "park_in": 1, "park_out": 2}.get(action_value.lower(), 0)
        try:
            action = int(action_value)
        except (TypeError, ValueError):
            action = 0
        if action not in (1, 2):
            print(f"[APA Bridge] reject APA command: invalid action={action_value}", flush=True)
            return False

        if action == 2 and self.parkout_target() is None:
            print(
                "[APA Bridge] reject APA park_out: target slot has no park-out target",
                flush=True,
            )
            return False

        speed = abs(self.current_speed()) if self.ego is not None else 0.0
        max_switch_speed = max(
            finite(self.cfg["apa"].get("action_switch_max_speed_mps"), 0.03), 0.0
        )
        if speed > max_switch_speed:
            print(
                f"[APA Bridge] reject APA command: vehicle moving speed={speed:.3f} "
                f"limit={max_switch_speed:.3f}",
                flush=True,
            )
            return False

        if self.ego is not None:
            bounds = self._planning_query_region["bounds"]
            vertices = self._ego_bounding_box.get_world_vertices(self.ego.get_transform())
            if not all(bounds[0] < p.x < bounds[2] and bounds[1] < p.y < bounds[3] for p in vertices):
                raise ValueError("车辆已超出本次规划查询范围，请重新应用场景设置后开始泊车")

        previous = self.parking_action
        if planning_only and self.parking_enabled and not self.planning_only:
            raise ValueError("请先退出泊车并停稳，再仅规划路径")
        fresh = not bool(getattr(self, "parking_enabled", False)) or bool(getattr(self, "_sync_fault", ""))
        fresh = fresh or planning_only or self.planning_only
        if action == previous and not fresh:
            print(
                f"[APA Bridge] ignore duplicate APA action={action} "
                f"name={'park_in' if action == 1 else 'park_out'}",
                flush=True,
            )
            return True
        if self._offline_parking_map is None:
            raise ValueError("离线泊车底图尚不可用，本次未启动泊车；请检查底图版本并重新加载")
        if self.ego is not None:
            self.scene_payload(self.world.get_snapshot(), force_occupancy=True)
        if action == 1 and self.ego is not None:
            availability = self.slot_provider.availability(self.target_slot)
            if availability["occupancy_state"] == "occupied":
                raise ValueError("目标停车位置与当前实体障碍重叠，本次未启动泊车")
            if availability["occupancy_state"] != "free":
                raise ValueError("目标车位的地图覆盖或高度层尚未确认，不能判定为空车位")
        if action == 1 and not planning_only and self.ego is not None:
            x, y, yaw = self.rear_axle_pose(self.ego.get_transform())
            self.parkout_return_pose = {
                "slot_id": int(self.target_slot.get("slot_id", 0)),
                "x": x, "y": y, "yaw_rad": yaw, "source": "park_in_start",
            }
        with self.safety_stop.lock:
            if self.safety_stop.reason and (
                    self.safety_stop.reason != "cancelled" or not self.safety_stop.clear()):
                raise ValueError("开始任务前出现保护停车，请先重置")
            self.parking_enabled = True
            self.planning_only = planning_only
            self.apa_phase = "park"
            self.parking_action = action
            self.history_revision += 1
            self._sync_fault = ""
            self._sync_task_key = None
        self.begin_algorithm_parking()
        self.clear_trajectory_history()
        self.last_plan_result = None
        print(
            f"[APA Bridge] APA action changed {previous}->{action} "
            f"name={'park_in' if action == 1 else 'park_out'} speed={speed:.3f}",
            flush=True,
        )
        return True

    def apply_control(self, payload: Dict[str, Any]) -> None:
        with self.safety_stop.lock:
            if self.safety_stop.reason:
                self.apply_sync_hold()
                return
            self._apply_algorithm_control(payload)

    def _submit_vehicle_control(self, control: Any, timeout_sec: Optional[float] = None) -> None:

        if timeout_sec is not None:
            self.client.set_timeout(timeout_sec)
        try:
            responses = self.client.apply_batch_sync(
                [self.carla.command.ApplyVehicleControl(self.ego.id, control)], False)
        finally:
            if timeout_sec is not None:
                self.client.set_timeout(float(self.cfg["carla"].get("runtime_rpc_timeout_sec", 2.0)))
        if len(responses) != 1 or responses[0].error:
            raise RuntimeError("CARLA did not acknowledge vehicle control")

    def _apply_algorithm_control(self, payload: Dict[str, Any]) -> None:
        self.last_control_payload = payload
        if hasattr(self, "parking_enabled") and not self.parking_enabled:
            return
        if self.ego is None or self.carla is None:
            return
        if time.monotonic() < self.ignore_control_until:
            control_id = int(read_number(payload, "command_id", "frame_id", default=-1))
            print(f"[APA Bridge] drop warmup control_cmd id={control_id}", flush=True)
            return
        target = payload.get("target", {})
        if not isinstance(target, dict):
            target = {}
        enable = read_bool(payload, "enable", "ControlEnabled", "control_enabled", default=True)
        stop_req = read_bool(payload, "StopReq", "stop_req", default=False)
        accel = read_number(
            payload,
            "TargetAcceleration",
            "target_acceleration",
            default=read_number(target, "target_accel_mps2", "target_acceleration", default=0.0),
        )
        target_speed = read_number(
            payload,
            "TargetSpeed",
            "target_speed",
            default=read_number(target, "target_speed_mps", default=0.0),
        )

        feedback = getattr(self, "_feedback_cache", None)
        same_frame = feedback is not None and int(payload.get("carla_frame_id", -1)) == feedback["frame_id"]
        if same_frame:
            self.record_tracking_error(payload)
        current_speed = feedback["vehicle_speed"] if same_frame else self.current_speed()
        steer_rad = read_front_wheel_rad(payload)
        max_steer = max(finite(self.cfg["control"].get("max_abs_steer_rad"), math.radians(45.0)), 1.0e-6)
        steer = carla_steer_command(steer_rad, max_steer, self.cfg["apa"]["vehicle_geometry"])

        gear_req = int(read_number(payload, "TargetGearReq", "target_gear_req", default=0))
        direction = int(read_number(payload, "DirectionSign", "direction_sign", default=0))
        gear_change_allowed = read_bool(
            payload, "GearChangeAllowed", "gear_change_allowed", default=True
        )

        try:
            actuator = validate_actuator(payload)
        except ValueError as exc:
            self.fail_sync(str(exc))
            return
        self.current_apa_gear_pos = actuator["gear_pos"]
        reverse = self.current_apa_gear_pos == 2
        gear = {0: 0, 1: 1, 2: -1, 3: 0}[self.current_apa_gear_pos]
        ctrl_phase = int(read_number(payload, "CtrlPhase", "ctrl_phase", default=-1))
        throttle, brake = actuator["throttle"], actuator["brake"]
        self.last_mapped_throttle = throttle
        self.last_mapped_brake = brake
        self.last_applied_gear = {1: "D", 2: "R", 3: "P"}.get(self.current_apa_gear_pos, "D")
        self._submit_vehicle_control(
            self.carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=steer,
                manual_gear_shift=True,
                reverse=reverse,
                gear=gear,
                hand_brake=actuator["hand_brake"],
            )
        )
        control_id = int(read_number(payload, "command_id", "frame_id", default=self.last_control_id + 1))
        now = time.monotonic()
        log_state = (getattr(self, "task_id", None), enable, stop_req, gear_req,
                     gear_change_allowed, self.current_apa_gear_pos, ctrl_phase)
        if (now - getattr(self, "_last_control_log_time", 0.0) >= 0.2 or
                log_state != getattr(self, "_last_control_log_state", None)):
            print(
                "[APA Bridge] apply control_cmd "
                f"id={control_id} enable={enable} stop={stop_req} "
                f"gear_req={gear_req} gear_allowed={gear_change_allowed} "
                f"gear_pos={self.current_apa_gear_pos} dir={direction} "
                f"target_speed={target_speed:.3f} current_speed={current_speed:.3f} "
                f"accel={accel:.3f} steer_rad={steer_rad:.3f} "
                f"carla_throttle={throttle:.3f} carla_brake={brake:.3f} "
                f"carla_steer={steer:.3f} carla_gear={gear} reverse={reverse}",
                flush=True,
            )
            self._last_control_log_time = now
            self._last_control_log_state = log_state
        self.last_control_id = control_id

    def measured_longitudinal_accel(self, speed: float, sim_time_sec: float) -> float:
        """Compute once per frame, using elapsed simulation time rather than wall time."""
        previous_time = getattr(self, "_speed_sample_time", None)
        previous_speed = float(getattr(self, "_speed_sample", speed))
        filtered = float(getattr(self, "_accel_filtered", 0.0))
        if previous_time is not None and sim_time_sec <= previous_time:
            return filtered
        self._speed_sample_time = sim_time_sec
        self._speed_sample = speed
        if previous_time is None:
            self._accel_filtered = 0.0
            return 0.0
        dt = sim_time_sec - previous_time
        if dt > 0.5:
            self._accel_filtered = 0.0
            return 0.0
        raw = clamp((speed - previous_speed) / dt, -6.0, 6.0)
        filtered += dt / (0.10 + dt) * (raw - filtered)
        self._accel_filtered = filtered
        return filtered

    def transmitted_path_pose(self, path: List[Tuple[float, float]], vehicle_x: float, vehicle_y: float) -> Tuple[float, float]:
        if not path:
            return vehicle_x, vehicle_y
        nearest = min(path, key=lambda point: (point[0] - vehicle_x) ** 2 + (point[1] - vehicle_y) ** 2)
        return nearest

    def current_speed(self) -> float:
        velocity = self.ego.get_velocity()
        yaw_rad = yaw_rad_from_carla_deg(finite(self.ego.get_transform().rotation.yaw))
        return signed_longitudinal_speed(velocity, yaw_rad)

    def _scene_setup_is_stationary(self) -> bool:
        velocity = self.ego.get_velocity()
        speed = math.sqrt(sum(required_number(v, "setup.velocity") ** 2
                              for v in (velocity.x, velocity.y, velocity.z)))
        return speed <= self.safety_stop.speed_limit

    def ego_payload(self, snapshot: Any) -> Dict[str, Any]:
        frame = int(getattr(snapshot, "frame", 0))
        cached = getattr(self, "_feedback_cache", None)
        if cached is None or cached["frame_id"] != frame:
            try:
                cached = self._sample_ego_payload(snapshot)
            except (ValueError, RuntimeError) as exc:
                self.latch_safety_stop(f"invalid_ego_feedback: {exc}")
                if isinstance(exc, ValueError):
                    raise InvalidEgoFeedback(str(exc)) from exc
                raise
            self._feedback_cache = cached
            compactor = getattr(self, "_scene_compactor", None)
            self._feedback_scene_provenance = compactor.summary.get("sources", {}) if compactor else {}
        payload = dict(cached)
        payload.update(self.sync_metadata(snapshot))
        payload["parking_enabled"] = bool(getattr(self, "parking_enabled", False)) and (
            time.monotonic() >= float(getattr(self, "ignore_control_until", 0.0)))
        payload["parking_action"] = self.parking_action
        payload["planning_only"] = bool(getattr(self, "planning_only", False)) and payload["parking_enabled"]
        payload["safety_state"] = self.safety_stop.payload()
        payload["recovery"] = {"recovery_id": self._recovery_id, "stop": self.recovery_stop.payload()}
        return payload

    def _sample_ego_payload(self, snapshot: Any) -> Dict[str, Any]:
        source = self.ego
        if callable(getattr(snapshot, "find", None)):
            source = snapshot.find(self.ego.id)
            if source is None:
                raise RuntimeError(f"ego {self.ego.id} missing from CARLA frame {snapshot.frame}")
        transform = source.get_transform()
        velocity = source.get_velocity()
        for name, value in (("vehicle_x", transform.location.x), ("vehicle_y", transform.location.y),
                            ("vehicle_yaw_deg", transform.rotation.yaw),
                            ("velocity_x", velocity.x), ("velocity_y", velocity.y), ("velocity_z", velocity.z)):
            required_number(value, name)
        vehicle_x, vehicle_y, yaw_rad = self.rear_axle_pose(transform)
        speed = signed_longitudinal_speed(velocity, yaw_rad)
        sim_time = required_number(getattr(getattr(snapshot, "timestamp", None), "elapsed_seconds", None), "sim_time_sec")
        accel_long = self.measured_longitudinal_accel(speed, sim_time)
        frame = int(getattr(snapshot, "frame", 0))
        cycle_time = required_number(getattr(getattr(snapshot, "timestamp", None), "delta_seconds", None), "cycle_time")

        applied = self.ego.get_control()
        raw_gear = int(applied.gear)
        gear_pos = 2 if raw_gear < 0 else 1 if raw_gear > 0 else (
            3 if self.current_apa_gear_pos == 3 else 0)
        steering = carla_to_equivalent(required_number(applied.steer, "applied.steer"),
            self.cfg["control"]["max_abs_steer_rad"], self.cfg["apa"]["vehicle_geometry"])
        up = transform.get_up_vector()
        gravity_x = 9.80665 * required_number(up.z, "road_normal.z") * required_number(up.x, "road_normal.x")
        gravity_y = 9.80665 * up.z * required_number(up.y, "road_normal.y")
        observer = getattr(self, "_steering_observer", None)
        if observer is None:
            observer = self._steering_observer = SteeringObservation()
        observer.sample((self.session_id, self.task_id, self.ego.id), frame, sim_time,
                        vehicle_x, vehicle_y, yaw_rad, speed,
                        lambda: (self.ego.get_wheel_steer_angle(self.carla.VehicleWheelLocation.FL_Wheel),
                                 self.ego.get_wheel_steer_angle(self.carla.VehicleWheelLocation.FR_Wheel)))
        return {
            "frame_id": frame,
            "vehicle_reference_point": REAR_AXLE_REFERENCE_POINT,
            "carla_actor_x": finite(transform.location.x),
            "carla_actor_y": finite(transform.location.y),
            "vehicle_x": vehicle_x,
            "vehicle_y": vehicle_y,
            "vehicle_yaw": yaw_rad,
            "vehicle_speed": speed,
            "vehicle_motion_speed": math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2),
            "vehicle_acceleration": accel_long,
            "road_gravity_x": gravity_x,
            "road_gravity_y": gravity_y,
            "vehicle_steering_command_rad": steering,
            "gear_pos": gear_pos,
            "gear_feedback_source": "carla_applied_control",
            "cycle_time": cycle_time,
            "scene": self.scene_payload(snapshot),
            "VehicleX": vehicle_x,
            "VehicleY": vehicle_y,
            "VehicleYaw": yaw_rad,
            "VehicleSpeed": speed,
            "VehicleAcceleration": accel_long,
            "GearPos": gear_pos,
            "CycleTime": cycle_time,
        }

    def publish_plan_request_if_needed(self, snapshot: Any, force: bool = False) -> None:
        if self.safety_stop.reason:
            return
        if hasattr(self, "parking_enabled") and not self.parking_enabled:
            return
        period = finite(self.cfg["apa"].get("plan_request_period_sec"), 1.0)
        sim_time = finite(getattr(getattr(snapshot, "timestamp", None), "elapsed_seconds", 0.0))
        previous = getattr(self, "_last_plan_sim_time", None)
        if not force and previous is not None and sim_time - previous < max(period, 0.02) - 1.0e-6:
            return
        self._last_plan_sim_time = sim_time
        try:
            payload = self.plan_request_payload(snapshot)
        except (ValueError, TypeError, KeyError) as exc:
            self.latch_safety_stop(f"invalid_plan_request: {exc}")
            return
        print(
            "[APA Bridge] publish plan_request "
            f"action={payload.get('parking_action')} "
            f"slot_kind={payload.get('target_slot_kind')} "
            f"parking_type={payload.get('parking_type')} "
            f"reference={payload.get('vehicle_reference_point')} "
            f"target_reference={payload.get('target_reference_point')} "
            f"ego=({payload.get('vehicle_x'):.3f},{payload.get('vehicle_y'):.3f},{payload.get('vehicle_yaw'):.3f}) "
            f"target=({payload.get('target_x'):.3f},{payload.get('target_y'):.3f},{payload.get('target_theta'):.3f}) "
            f"entry=({payload.get('entry_point_x'):.3f},{payload.get('entry_point_y'):.3f}) "
            f"boundary={payload.get('boundary_line_num')} "
            f"virtual={payload.get('virtual_boundary_line_num')} "
            f"slot_stopper={payload.get('slot_track_stopper_num')} "
            f"sep={payload.get('boundary_sep_point_num')} "
            f"stopper={payload.get('stopper_point_num')}",
            flush=True,
        )
        self.publish(APA_PLAN_REQUEST_TOPIC, payload, snapshot)

    def plan_request_payload(self, snapshot: Any = None) -> Dict[str, Any]:

        metadata = self.sync_metadata(snapshot)
        slot = self.target_slot
        def slot_number(key: str) -> float:
            return required_number(slot.get(key), f"slot.{key}")

        for i in range(1, 5):
            slot_number(f"slot_pt{i}_x")
            slot_number(f"slot_pt{i}_y")
        slot_number("slot_entry_pt_x")
        slot_number("slot_entry_pt_y")
        for key, default, low, high in (("slot_entry", 1, 1, 4), ("parking_type", 3, 0, 3),
                                        ("surrounding_type", 3, 0, 4)):
            value = slot.get(key, default)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"slot.{key} 枚举无效")
        feedback = getattr(self, "_feedback_cache", None)
        if snapshot is not None and feedback is not None and feedback["frame_id"] == snapshot.frame:
            vehicle_x, vehicle_y, vehicle_yaw = (
                feedback["vehicle_x"], feedback["vehicle_y"], feedback["vehicle_yaw"])
        else:
            ego = self.ego.get_transform() if self.ego is not None else None
            vehicle_x, vehicle_y, vehicle_yaw = self.rear_axle_pose(ego)
        obstacles = self.slot_provider.obstacles
        if not isinstance(obstacles, dict):
            raise ValueError("obstacles 必须是对象")
        lines, vlines, boundary_policy = self.boundary_inputs()
        slot_stoppers = obstacles.get("obs_slot_stoppers", [])
        points = obstacles.get("obs_points", [])
        stoppers = obstacles.get("obs_stoppers", [])
        target_slot_kind = apa_slot_kind_code(slot)
        if target_slot_kind not in (1, 2, 3, 4):
            raise ValueError("slot_kind 无效")
        for name, items, capacity in (("obs_lines", lines, 64), ("obs_vlines", vlines, 64),
                                       ("obs_slot_stoppers", slot_stoppers, 64),
                                       ("obs_points", points, 128), ("obs_stoppers", stoppers, 128)):
            if not isinstance(items, list) or len(items) > capacity:
                raise ValueError(f"{name} 必须是容量不超过 {capacity} 的数组")

        def arr(items: List[Dict[str, Any]], key: str) -> List[float]:
            values = []
            for item in items:
                if not isinstance(item, dict):
                    raise ValueError("障碍数组含非对象元素")
                values.append(required_number(item.get(key), f"obstacle.{key}"))
            return values

        if self.parking_action == 2:

            for fields in (("parkout_target_x", "parkout_target_y", "parkout_target_yaw_rad"),
                           ("apa_start_rear_x", "apa_start_rear_y", "apa_start_rear_yaw_rad")):
                if any(key in slot for key in fields):
                    for key in fields:
                        slot_number(key)
            target = self.parkout_target(slot)
            if target is None:
                raise ValueError("泊出目标不可用，请先完成泊入以保存返回起点")
            target_x, target_y, target_theta = target["x"], target["y"], target["yaw_rad"]
            for key, value in (("x", target_x), ("y", target_y), ("yaw_rad", target_theta)):
                required_number(value, f"parkout_target.{key}")
        else:
            if self._task_parkin_goal is None:
                row = self.slot_provider.occupancy.get(int(slot['slot_id']), {})
                self._task_parkin_goal = (tuple(row['parking_goal']) if 'parking_goal' in row
                                          else self.parkin_target(slot))
            target_x, target_y, target_theta = self._task_parkin_goal

        payload = {
            "frame_id": int(getattr(snapshot, "frame", 0)),
            "parking_action": self.parking_action,
            "target_slot_id": int(slot.get("slot_id", 0)),
            "target_slot_kind": target_slot_kind,
            "vehicle_reference_point": REAR_AXLE_REFERENCE_POINT,
            "vehicle_x": vehicle_x,
            "vehicle_y": vehicle_y,
            "vehicle_yaw": vehicle_yaw,
            "target_reference_point": REAR_AXLE_REFERENCE_POINT,
            "target_x": target_x,
            "target_y": target_y,
            "target_theta": target_theta,
            "r_min": max(self.cfg["apa"]["vehicle_geometry"]["planning_min_turn_radius_m"],
                         self.cfg["apa"]["vehicle_geometry"]["wheelbase_m"] /
                         math.tan(self.cfg["control"]["max_abs_steer_rad"])),
            "entry_point_x": slot_number("slot_entry_pt_x"),
            "entry_point_y": slot_number("slot_entry_pt_y"),
            "slot_point1_x": slot_number("slot_pt1_x"),
            "slot_point1_y": slot_number("slot_pt1_y"),
            "slot_point2_x": slot_number("slot_pt2_x"),
            "slot_point2_y": slot_number("slot_pt2_y"),
            "slot_point3_x": slot_number("slot_pt3_x"),
            "slot_point3_y": slot_number("slot_pt3_y"),
            "slot_point4_x": slot_number("slot_pt4_x"),
            "slot_point4_y": slot_number("slot_pt4_y"),
            "slot_entry": int(slot.get("slot_entry", 1)),
            "parking_type": apa_default_parking_type(slot),
            "surrounding_type": apa_default_surrounding_type(slot),
            "angle_offset": 0.0,
            "boundary_line_num": len(lines),
            "boundary_line_x1": arr(lines, "x1"),
            "boundary_line_y1": arr(lines, "y1"),
            "boundary_line_x2": arr(lines, "x2"),
            "boundary_line_y2": arr(lines, "y2"),
            "virtual_boundary_line_num": len(vlines),
            "virtual_boundary_p1_x": arr(vlines, "x1"),
            "virtual_boundary_p1_y": arr(vlines, "y1"),
            "virtual_boundary_p2_x": arr(vlines, "x2"),
            "virtual_boundary_p2_y": arr(vlines, "y2"),
            "slot_track_stopper_num": len(slot_stoppers),
            "slot_track_stopper_x1": arr(slot_stoppers, "x1"),
            "slot_track_stopper_y1": arr(slot_stoppers, "y1"),
            "slot_track_stopper_x2": arr(slot_stoppers, "x2"),
            "slot_track_stopper_y2": arr(slot_stoppers, "y2"),
            "boundary_sep_point_num": len(points),
            "boundary_sep_point_x": arr(points, "x"),
            "boundary_sep_point_y": arr(points, "y"),
            "stopper_point_num": len(stoppers),
            "stopper_point_x": arr(stoppers, "x"),
            "stopper_point_y": arr(stoppers, "y"),
            "shortcut_switch": False,
            "shortcut_threshold": 0.0,
            "need_hybrid_algorithm_fusion": True,
        }
        payload.update(metadata)

        payload["boundary_policy"] = boundary_policy
        position_tolerance, yaw_tolerance = goal_tolerance(slot, self.cfg["apa"]["vehicle_geometry"])
        payload["goal_tolerance"] = {"position_m": position_tolerance, "yaw_rad": yaw_tolerance}
        payload.update({
            "ParkingAction": payload["parking_action"],
            "TargetSlotId": payload["target_slot_id"],
            "TargetSlotKind": payload["target_slot_kind"],
            "VehicleX": payload["vehicle_x"],
            "VehicleY": payload["vehicle_y"],
            "VehicleYaw": payload["vehicle_yaw"],
            "TarX": payload["target_x"],
            "TarY": payload["target_y"],
            "TarTheta": payload["target_theta"],
            "Rmin": payload["r_min"],
            "EntrPtX": payload["entry_point_x"],
            "EntrPtY": payload["entry_point_y"],
            "SltPt1X": payload["slot_point1_x"],
            "SltPt1Y": payload["slot_point1_y"],
            "SltPt2X": payload["slot_point2_x"],
            "SltPt2Y": payload["slot_point2_y"],
            "SltPt3X": payload["slot_point3_x"],
            "SltPt3Y": payload["slot_point3_y"],
            "SltPt4X": payload["slot_point4_x"],
            "SltPt4Y": payload["slot_point4_y"],
            "SltEntr": payload["slot_entry"],
            "PrkType": payload["parking_type"],
            "SurdType": payload["surrounding_type"],
            "AgrOfst": payload["angle_offset"],
            "BoundaryLineNum": payload["boundary_line_num"],
            "BoundaryLineX1": payload["boundary_line_x1"],
            "BoundaryLineY1": payload["boundary_line_y1"],
            "BoundaryLineX2": payload["boundary_line_x2"],
            "BoundaryLineY2": payload["boundary_line_y2"],
            "VirtualBoundaryLineNum": payload["virtual_boundary_line_num"],
            "VirtualBoundaryP1X": payload["virtual_boundary_p1_x"],
            "VirtualBoundaryP1Y": payload["virtual_boundary_p1_y"],
            "VirtualBoundaryP2X": payload["virtual_boundary_p2_x"],
            "VirtualBoundaryP2Y": payload["virtual_boundary_p2_y"],
            "SlotTrackStopperNum": payload["slot_track_stopper_num"],
            "SlotTrackStopperX1": payload["slot_track_stopper_x1"],
            "SlotTrackStopperY1": payload["slot_track_stopper_y1"],
            "SlotTrackStopperX2": payload["slot_track_stopper_x2"],
            "SlotTrackStopperY2": payload["slot_track_stopper_y2"],
            "BoundarySepPtNum": payload["boundary_sep_point_num"],
            "BoundarySepPtX": payload["boundary_sep_point_x"],
            "BoundarySepPtY": payload["boundary_sep_point_y"],
            "StopperPtNum": payload["stopper_point_num"],
            "StopperPtX": payload["stopper_point_x"],
            "StopperPtY": payload["stopper_point_y"],
            "ShortCutSw": payload["shortcut_switch"],
            "ShortCutThd": payload["shortcut_threshold"],
            "NeedHybridAlgFusion": payload["need_hybrid_algorithm_fusion"],
        })
        return payload

    def boundary_inputs(self):
        frames = self.slot_boundary_lines()
        lines, virtual, policy = build_boundary_inputs(self.slot_provider.obstacles, frames,
                                                       self._planning_query_region["bounds"])
        policy["neighbor_access_snapshot"] = copy.deepcopy(self._neighbor_boundary_access)
        return lines, virtual, policy

    def slot_boundary_lines(self) -> List[Dict[str, Any]]:
        result: List[Dict[str, Any]] = []
        target_physical = physical_slot_id(self.target_slot)

        key = (getattr(self, "session_id", ""), getattr(self, "task_id", 0),
               int(self.target_slot["slot_id"]), getattr(self, "parking_action", 1))
        if key != getattr(self, "_neighbor_boundary_key", None):
            available = []
            occupancy_frame = getattr(self, "_slot_occupancy_summary", {}).get("frame")
            if key[3] in (1, 2):
                for row in self.slot_provider.physical_slots():
                    if int(row["slot_id"]) == target_physical:
                        continue
                    state = getattr(self.slot_provider, "occupancy", {}).get(int(row["slot_id"]), {})
                    frame = state.get("frame")
                    if (state.get("state") == "free" and type(frame) is int and frame >= 0 and
                            frame == occupancy_frame and
                            state.get("static_frame") == frame):
                        available.append({"slot_id": int(row["slot_id"]), "frame": frame})
            self._neighbor_boundary_key = key
            self._neighbor_boundary_access = {"session_id": key[0], "task_id": key[1],
                "parking_action": key[3], "frame": occupancy_frame, "confirmed_free": available}
        borrowable = {row["slot_id"] for row in self._neighbor_boundary_access["confirmed_free"]}

        bounds = self._planning_query_region["bounds"]
        for row in self.slot_provider.physical_slots():
            is_target = int(row["slot_id"]) == target_physical
            if is_target:
                row = self.target_slot
            if not is_target and int(row["slot_id"]) in borrowable:
                continue
            pts = [
                (required_number(row.get("slot_pt1_x"), "slot_pt1_x"), required_number(row.get("slot_pt1_y"), "slot_pt1_y")),
                (required_number(row.get("slot_pt2_x"), "slot_pt2_x"), required_number(row.get("slot_pt2_y"), "slot_pt2_y")),
                (required_number(row.get("slot_pt3_x"), "slot_pt3_x"), required_number(row.get("slot_pt3_y"), "slot_pt3_y")),
                (required_number(row.get("slot_pt4_x"), "slot_pt4_x"), required_number(row.get("slot_pt4_y"), "slot_pt4_y")),
            ]
            entry_edge = row.get("slot_entry", 1)
            if type(entry_edge) is not int or not 1 <= entry_edge <= 4:
                raise ValueError("slot_entry 枚举无效")
            for edge_idx, (first, second) in enumerate(zip(pts, pts[1:] + pts[:1]), start=1):
                if edge_idx == entry_edge:
                    continue
                if not is_target and not line_intersects_bounds(first, second, bounds):
                    continue
                line = {"x1": first[0], "y1": first[1], "x2": second[0], "y2": second[1],
                        "slot_id": int(row["slot_id"]), "edge": edge_idx, "is_target": is_target}
                if "slot_edge_ids" in row:
                    line["shared_edge_id"] = row["slot_edge_ids"][edge_idx-1]
                result.append(line)
        return result

    def ego_slope_deg(self) -> float:
        if self.ego is None:
            return 0.0
        try:
            forward = self.ego.get_transform().get_forward_vector()
            return math.degrees(math.asin(clamp(finite(getattr(forward, "z", 0.0)), -1.0, 1.0)))
        except Exception:
            try:
                return finite(getattr(self.ego.get_transform().rotation, "pitch", 0.0))
            except Exception:
                return 0.0

    def overhead_camera_transform(self):
        """Follow ego position in world space, with the reset heading fixed."""
        location = self.ego.get_transform().location
        return self.carla.Transform(
            self.carla.Location(x=location.x, y=location.y,
                z=location.z + finite(self.cfg["apa"].get("overhead_camera_z"), 18.0)),
            self.carla.Rotation(pitch=-90.0,
                yaw=required_number(self.initial_pose["yaw_deg"], "initial_pose.yaw_deg")),
        )

    def update_overhead_camera(self) -> None:
        camera = getattr(self, "overhead_camera", None)
        if camera is None:
            return
        try:
            camera.set_transform(self.overhead_camera_transform())
            self._overhead_camera_error = ""
        except Exception as exc:

            if not self._overhead_camera_error:
                print(f"[APA Bridge] overhead camera update failed: {exc}", file=sys.stderr)
            self._overhead_camera_error = str(exc) or "camera update failed"

    def reset_camera_frames(self) -> None:
        frame = self.world.get_snapshot().frame
        with self.camera_lock:
            self._camera_revision += 1
            self._camera_min_frame = frame
            self._camera_pending.clear()
            for view in self.camera_bytes:
                self.camera_bytes[view] = b""
                self.camera_received_at[view] = 0.0

    def latest_camera_frame(self, view: str = "rear") -> Tuple[bytes, str, float, int]:
        with self.camera_lock:
            return (self.camera_bytes[view], self.camera_mime[view],
                    self.camera_received_at[view], self._camera_revision)

    def on_camera_image(self, image: Any, view: str = "rear") -> None:
        try:
            pending = (int(image.width), int(image.height), bytes(image.raw_data), time.monotonic())
        except Exception:
            return
        with self.camera_lock:
            if image.frame <= self._camera_min_frame:
                return
            if view == "overhead":
                if self._overhead_camera_error or self.initial_pose is None:
                    return

                right = image.transform.get_right_vector()
                yaw = math.radians(self.initial_pose["yaw_deg"])
                if -right.x * math.sin(yaw) + right.y * math.cos(yaw) < 0.9999:
                    return
            self._camera_pending[view] = (*pending, self._camera_revision)
            if view in self._camera_worker_started:
                return
            self._camera_worker_started.add(view)
        threading.Thread(target=self._encode_camera_loop, args=(view,),
                         name=f"apa-{view}-encode", daemon=True).start()

    def _encode_camera_loop(self, view: str) -> None:
        while self.running:
            with self.camera_lock:
                pending = self._camera_pending.pop(view, None)
            if pending is None:
                time.sleep(0.02)
                continue
            width, height, raw, received_at, revision = pending
            try:
                payload, mime = encode_camera_frame(width, height, raw)
            except Exception as exc:
                print(f"[APA Bridge] {view} camera encode failed: {exc}", file=sys.stderr)
                continue
            with self.camera_lock:
                if revision != self._camera_revision or (view == "overhead" and self._overhead_camera_error):
                    continue
                self.camera_bytes[view] = payload
                self.camera_mime[view] = mime
                self.camera_received_at[view] = received_at

    def attach_camera(self, view: str = "rear") -> None:
        if self.dry_run or self.world is None or self.ego is None or self.carla is None:
            return
        if not bool(self.cfg["apa"].get("camera_enabled", True)):
            return
        overhead = view == "overhead"
        if overhead and not bool(self.cfg["apa"].get("overhead_camera_enabled", True)):
            return
        try:
            carla_cfg = self.cfg["carla"]
            camera_client = self.camera_client or self.carla.Client(str(carla_cfg["host"]), int(carla_cfg["port"]))
            camera_client.set_timeout(float(carla_cfg.get("timeout_sec", 5.0)))
            camera_world = camera_client.get_world()
            ego = camera_world.get_actor(self.ego.id)
            blueprint = camera_world.get_blueprint_library().find("sensor.camera.rgb")
            blueprint.set_attribute("image_size_x", str(int(self.cfg["apa"].get("camera_width", 480))))
            blueprint.set_attribute("image_size_y", str(int(self.cfg["apa"].get("camera_height", 270))))
            fov = self.cfg["apa"].get("overhead_camera_fov" if overhead else "camera_fov")
            blueprint.set_attribute("fov", str(finite(fov, 90.0 if overhead else 100.0)))
            sensor_tick = 1.0 / max(finite(self.cfg["apa"].get("camera_fps"), 8.0), 1.0)
            blueprint.set_attribute("sensor_tick", str(sensor_tick))
            if overhead:
                camera = camera_world.spawn_actor(blueprint, self.overhead_camera_transform())
            else:
                transform = self.carla.Transform(
                    self.carla.Location(x=finite(self.cfg["apa"].get("camera_x"), -2.45),
                                        z=finite(self.cfg["apa"].get("camera_z"), 1.05)),
                    self.carla.Rotation(pitch=finite(self.cfg["apa"].get("camera_pitch_deg"), -16.0), yaw=180.0),
                )
                camera = camera_world.spawn_actor(blueprint, transform, attach_to=ego)
            setattr(self, f"{view}_camera", camera)
            camera.listen(lambda image: self.on_camera_image(image, view))
            self.camera_client = camera_client
            print(f"[APA Bridge] {view} camera on a separate client id={camera.id}", flush=True)
        except Exception as exc:
            print(f"[APA Bridge] {view} camera attach failed: {exc}", file=sys.stderr)

    def start_camera_http(self) -> None:
        if self.dry_run or not bool(self.cfg["apa"].get("camera_enabled", True)):
            return
        host = str(self.cfg["apa"].get("camera_http_host", "127.0.0.1"))
        port = int(self.cfg["apa"].get("camera_http_port", 5704))
        RearCameraHandler.bridge = self
        try:
            server = ThreadingHTTPServer((host, port), RearCameraHandler)
        except OSError as exc:
            print(f"[APA Bridge] rear camera HTTP bind failed {host}:{port}: {exc}", file=sys.stderr)
            return
        self.camera_http = server
        self.camera_http_url = f"http://{host}:{port}"
        thread = threading.Thread(target=server.serve_forever, name="apa-rear-camera", daemon=True)
        thread.start()
        print(f"[APA Bridge] rear camera HTTP {self.camera_http_url}/rear.jpg", flush=True)

    def apply_driver_vehicle_message(self, payload: Dict[str, Any]) -> None:
        requested_phase = str(payload.get("apa_phase") or "").strip().lower()
        if payload.get("session_id") != self.session_id or payload.get("task_id") != self.task_id:
            return
        with self.safety_stop.lock:
            if self.safety_stop.reason:
                if (self.safety_stop.reason == "cancelled" and self.safety_stop.state == "stopped" and
                        requested_phase != "park" and payload.get("gear") in ("D", "R", "N")):
                    self.safety_stop.clear()
                else:
                    return
        if requested_phase == "park":

            return
        if self.parking_enabled and bool(payload.get("apa_exit")):
            self.latch_safety_stop("cancelled")
            self.stop_parking_task()
            return
        if self.parking_enabled and not bool(payload.get("apa_exit")):
            return
        if requested_phase in APA_PHASES:
            self.apa_phase = requested_phase
            if self.parking_enabled:
                self.begin_manual_driving()
            self.parking_enabled = False
        max_steer = max(finite(self.cfg["control"].get("max_abs_steer_rad"), 0.6), 1.0e-6)
        self.manual_throttle = clamp(finite(payload.get("throttle", 0.0)), 0.0, 1.0)
        self.manual_brake = clamp(finite(payload.get("brake", 0.0)), 0.0, 1.0)
        self.manual_steer = manual_steer_command(payload, max_steer, self.cfg["apa"]["vehicle_geometry"])
        if "gear" in payload:
            gear = str(payload.get("gear") or "D").strip().upper()
            if gear in {"P", "R", "N", "D"}:
                self.manual_gear = gear
        if "hand_brake" in payload:
            self.manual_hand_brake = bool(payload.get("hand_brake"))
        if "ignition_switch" in payload:
            ignition = str(payload.get("ignition_switch") or "ON").upper()
            if ignition in {"OFF", "ACC", "ON", "START"}:
                self.manual_ignition = ignition
        self.manual_last_input = time.monotonic()

    def begin_algorithm_parking(self) -> None:
        if self.ego is not None and hasattr(self.ego, "set_simulate_physics"):
            self.ego.set_simulate_physics(True)

    def begin_manual_driving(self) -> None:
        if self.ego is not None and hasattr(self.ego, "set_simulate_physics"):
            self.ego.set_simulate_physics(True)

    def apply_manual_hold(self) -> None:
        with self.safety_stop.lock:
            if self.safety_stop.reason:
                self.apply_sync_hold()
                return
            self._apply_manual_control()

    def _apply_manual_control(self) -> None:
        if self.ego is None or self.carla is None or self.parking_enabled:
            return
        now = time.monotonic()
        stale = self.manual_last_input <= 0.0 or now - self.manual_last_input > 0.45
        throttle = 0.0 if stale else self.manual_throttle
        brake = 1.0 if stale else self.manual_brake
        steer = 0.0 if stale else self.manual_steer
        gear = self.manual_gear
        hand_brake = False if stale else bool(self.manual_hand_brake)
        if self.manual_ignition not in {"ON", "START"}:
            throttle = 0.0
            if self.manual_ignition == "OFF":
                gear = "P"
                hand_brake = True
                brake = 1.0
        if gear == "P" or hand_brake:
            throttle = 0.0
            brake = max(brake, 1.0)
            if gear == "P":
                hand_brake = True
        elif gear == "N":
            throttle = 0.0
        dt = finite(self.cfg["carla"].get("fixed_delta_seconds"), 0.05)
        slew = max(finite(self.cfg["control"].get("manual_pedal_slew_per_sec"), 1.6), 0.05) * dt
        steer_step = max(finite(self.cfg["control"].get("manual_steer_slew_per_sec"), 40.0), 0.1) * dt
        self.applied_manual_throttle = clamp(throttle, self.applied_manual_throttle - slew, self.applied_manual_throttle + slew)
        self.applied_manual_brake = clamp(brake, self.applied_manual_brake - slew, self.applied_manual_brake + slew)
        self.applied_manual_steer = clamp(steer, self.applied_manual_steer - steer_step, self.applied_manual_steer + steer_step)
        if self.applied_manual_brake > 1.0e-3:
            self.applied_manual_throttle = 0.0
        reverse = gear == "R"
        gear_int = -1 if reverse else (0 if gear in {"P", "N"} else 1)
        self.last_applied_gear = gear
        self._submit_vehicle_control(
            self.carla.VehicleControl(
                throttle=self.applied_manual_throttle,
                brake=self.applied_manual_brake,
                steer=self.applied_manual_steer,
                hand_brake=bool(hand_brake),
                reverse=reverse,
                manual_gear_shift=True,
                gear=gear_int,
            )
        )

    def clear_trajectory_history(self) -> None:
        self.actual_path_xy.clear()
        self.accepted_plan_history.clear()
        self.last_accepted_plan_result = None
        self.tracking_error_summary = {}
        self._tracking_error_frame = -1
        self._tracking_error_square_sum = 0.0

    def record_tracking_error(self, payload: Dict[str, Any]) -> None:
        diagnostic = payload.get("diagnostic", {})
        frame = int(payload.get("carla_frame_id", -1))
        if (frame <= self._tracking_error_frame or not isinstance(diagnostic, dict) or
                not diagnostic.get("reference_valid") or payload.get("CtrlPhase") not in (3, 7, 8)):
            return
        lateral = diagnostic.get("cross_track_error_m")
        heading = diagnostic.get("heading_error_rad")
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in (lateral, heading)):
            return
        self._tracking_error_frame = frame
        count = self.tracking_error_summary.get("samples", 0) + 1
        self._tracking_error_square_sum += lateral * lateral
        self.tracking_error_summary = {
            "samples": count,
            "rms_cross_track_error_m": math.sqrt(self._tracking_error_square_sum / count),
            "peak_cross_track_error_m": max(abs(lateral), self.tracking_error_summary.get("peak_cross_track_error_m", 0.0)),
            "peak_heading_error_rad": max(abs(heading), self.tracking_error_summary.get("peak_heading_error_rad", 0.0)),
        }

    def record_actual_path(self) -> None:
        if self.ego is None:
            return
        feedback = getattr(self, "_feedback_cache", None)
        if feedback is not None:
            point = (feedback["vehicle_x"], feedback["vehicle_y"])
        else:
            transform = self.ego.get_transform()
            rear_x, rear_y, _ = self.rear_axle_pose(transform)
            point = (rear_x, rear_y)
        trail = self.actual_path_xy
        if trail:
            last = trail[-1]
            if math.hypot(point[0] - last[0], point[1] - last[1]) < 0.03:
                return
        if len(trail) == trail.maxlen:

            retained = list(trail)[::2]
            trail.clear()
            trail.extend(retained)
        trail.append(point)

    def draw_polyline(self, points: List[Tuple[float, float]], color: Any, thickness: float, ground_z: float) -> None:
        if self.world is None or self.carla is None or len(points) < 2:
            return
        lifted = [
            self.carla.Location(float(x), float(y), ground_z)
            for x, y in points
        ]
        life = max(1.0 / max(finite(self.cfg["apa"].get("trajectory_draw_hz"), 5.0), 1.0) * 1.15, 0.08)
        for first, second in zip(lifted, lifted[1:]):
            self.world.debug.draw_line(first, second, thickness=thickness, color=color, life_time=life)

    def draw_trajectories(self) -> None:
        if self.world is None or self.carla is None or self.ego is None:
            return
        if not bool(self.cfg["apa"].get("draw_trajectories", True)):
            return
        period = 1.0 / max(finite(self.cfg["apa"].get("trajectory_draw_hz"), 5.0), 1.0)
        now = time.monotonic()
        if now - self.last_trajectory_draw < period:
            return
        self.last_trajectory_draw = now
        ground_z = finite(self.ego.get_transform().location.z, 0.4) + 0.08
        plan = self.last_accepted_plan_result or {}
        self.draw_polyline(extract_plan_path_xy(plan, 80), self.carla.Color(176, 78, 24), 0.02, ground_z)
        actual = extract_plan_path_xy({"path": list(self.actual_path_xy)}, 180)
        self.draw_polyline(actual, self.carla.Color(20, 110, 78), 0.02, ground_z + 0.02)

    def status_payload(self, *, read_vehicle=True) -> Dict[str, Any]:
        plan = self.last_plan_result if isinstance(self.last_plan_result, dict) else {}
        display_plan = self.last_accepted_plan_result or plan
        display_path = extract_plan_path_xy(display_plan, 100)
        algo = self.last_control_payload if isinstance(self.last_control_payload, dict) else {}
        path = extract_plan_path_xy(plan, 100) if self.parking_enabled else []
        diagnostic = algo.get("diagnostic", {}) if isinstance(algo.get("diagnostic"), dict) else {}
        actual = list(self.actual_path_xy)
        accel = read_number(algo, "TargetAcceleration", "target_acceleration", default=0.0) if algo else 0.0
        target_speed = read_number(algo, "TargetSpeed", "target_speed", default=0.0) if algo else 0.0
        steer_rad = read_front_wheel_rad(algo) if algo else 0.0
        gear_req = int(read_number(algo, "TargetGearReq", "target_gear_req", default=0)) if algo else 0
        direction = int(read_number(algo, "DirectionSign", "direction_sign", default=0)) if algo else 0
        algo_gear = "R" if gear_req == 2 or direction < 0 else ("P" if gear_req == 3 else "D")
        ego_x = ego_y = ego_yaw = actor_x = actor_y = 0.0
        vehicle_control = None
        if self.ego is not None and read_vehicle:
            transform = self.ego.get_transform()
            ego_x, ego_y, ego_yaw = self.rear_axle_pose(transform)
            actor_x = finite(transform.location.x)
            actor_y = finite(transform.location.y)
            try:
                vehicle_control = self.ego.get_control()
            except Exception:
                vehicle_control = None
        gear_name = self.last_applied_gear if self.last_applied_gear in {"P", "R", "N", "D"} else "D"
        if not self.parking_enabled:
            gear_name = self.manual_gear
        corners = [
            {"x": finite(self.target_slot.get(f"slot_pt{index}_x")), "y": finite(self.target_slot.get(f"slot_pt{index}_y"))}
            for index in (1, 2, 3, 4)
        ]
        speed = self.current_speed() if self.ego is not None and read_vehicle else None
        cached = getattr(self, "_feedback_cache", None)
        if cached is not None:
            ego_x, ego_y, ego_yaw = cached["vehicle_x"], cached["vehicle_y"], cached["vehicle_yaw"]
            actor_x, actor_y = cached["carla_actor_x"], cached["carla_actor_y"]
            speed = cached["vehicle_speed"]
        accel_long = cached["vehicle_acceleration"] if cached is not None else 0.0
        expected_x, expected_y = self.transmitted_path_pose(path, ego_x, ego_y)
        if diagnostic.get("reference_valid"):
            expected_x, expected_y = finite(diagnostic.get("reference_x")), finite(diagnostic.get("reference_y"))
        stop_messages = {3: "输入或参数无效", 4: "活动段参考丢失或偏差过大", 5: "未达到段终点位姿",
                         6: "档位确认超时", 7: "路径进度超时", 8: "终点车身边界不合格",
                         9: "C控制器实测超速保护"}
        linked = time.monotonic() - float(getattr(self, "last_algorithm_rx", 0.0)) < 1.0
        safety = self.safety_stop.payload()
        stop_state_messages = {"braking": "正在制动并确认停稳", "stopped": "已确认停稳，保持制动",
                               "unconfirmed": "尚未确认停稳，保持制动并等待有效停稳反馈"}
        safety_message = (f"保护停车：{safety['reason']}；{stop_state_messages.get(safety['state'], '')}"
                          if safety["reason"] else "")
        replan = algo.get("replan", {}) if isinstance(algo.get("replan"), dict) else {}
        planning_reasons = {
            "planning_clearance": "规划余量不足，保持停车",
            "clearance_deficit": "车身未碰障，正在规划恢复净距的路径",
            "clearance_predicted": "预测制动范围内余量不足，停车复核",
            "motion_blocked": "实际运动预计碰障或压线，停车复核",
            "line_contact": "实际车身已触及车位中心线，保持停车",
            "body_blocked": "实际车身与实体障碍重叠，保持停车",
            "start_contact": "重规划起点已碰障或压线，保持停车",
            "clearance_recovery_no_solution": "未找到可安全恢复规划余量的路径",
            "clearance_recovery_capacity": "恢复路径拼接超出点数容量",
            "clearance_recovery_deviation": "恢复净距未按预期增加，停车重新规划",
            "body_predicted": "动态障碍预计侵入当前车身，保持停车等待",
            "no_solution": "未找到通过规划与安全检查的路径",
            "planning_scene_capacity": "场景约束超出规划容量",
            "planning_refinement_limit": "场景约束细化达到上限",
            "unsafe_candidate": "候选路径未通过当前场景安全检查",
            "planning_timeout": "路径搜索超过单次时间预算",
            "planner_process_failed": "规划进程异常退出",
            "candidate_blocked": "候选路径被当前场景障碍阻挡",
            "stale_plan_start": "规划期间车辆起点发生变化",
            "endpoint_deviation": "段终点位姿超差，正在从停稳位置重新规划",
        }
        planning_reason = planning_reasons.get(replan.get("reason"), str(replan.get("reason", "")))
        planner_error = replan.get("planner_error_type", 0)
        if type(planner_error) is int and planner_error:
            planning_reason += f"；规划错误码 {planner_error}" + ("（未知错误）" if planner_error == 31 else "")
        scene_diagnostic = self.scene_diagnostics.update(
            (self.session_id, self.task_id), algo, safety["reason"], self.last_task_command,
            {"map_preparation": getattr(self, "_map_preparation", {}),
             "target_slot": self.target_slot, "slot_occupancy": self.slot_provider.availability(self.target_slot),
             "task_command": self.last_task_command, "safety_state": safety,
             "accepted_plans": [item for item in self.accepted_plan_history
                 if (item.get("session_id"), item.get("task_id")) == (self.session_id, self.task_id)],
             "actual_path_xy": actual})

        applied_steer_wheel_deg = math.degrees(carla_to_equivalent(
            finite(getattr(vehicle_control, "steer", 0.0)),
            self.cfg["control"]["max_abs_steer_rad"], self.cfg["apa"]["vehicle_geometry"]))
        observer = getattr(self, "_steering_observer", None)
        steering_observation = empty_steering_observation("feedback_unavailable")
        if (read_vehicle and cached is not None and observer is not None and self.ego is not None
                and observer.identity == (self.session_id, self.task_id, self.ego.id)
                and observer.payload["frame_id"] == cached["frame_id"]):
            steering_observation = observer.payload
        return {
            "session_id": self.session_id,
            "task_id": self.task_id,
            "map_name": self.cfg["carla"].get("map_name", ""),
            "parking_map": self.slot_provider.map_summary,
            "geometry": dict(getattr(self, "_obstacle_profile_summary", {}),
                             scene_map=getattr(self, "_scene_map_summary", {}),
                             progress=getattr(self, "_geometry_progress", {}),
                             map_preparation=getattr(self, "_map_preparation", {}),
                             preparing=False),
            "slot_occupancy": dict(getattr(self, "_slot_occupancy_summary", {}),
                                   preparing_geometry=False),
            "map_choices": self.map_choices,
            "map_transition": self.map_transition,
            "history_revision": self.history_revision,
            "camera_revision": self._camera_revision,
            "vehicle_geometry": self.cfg["apa"].get("vehicle_geometry", {}),
            "vehicle_feedback_available": read_vehicle and self.ego is not None,
            "action_switch_max_speed_mps": max(finite(
                self.cfg["apa"].get("action_switch_max_speed_mps"), 0.03), 0.0),
            "initial_pose": self.initial_pose,
            "start_mode": self.initial_pose_mode,
            "last_task_command": self.last_task_command,
            "slot_choices": [
                {**slot_choice_identity(slot),
                 "corners": [{"x": slot[f"slot_pt{index}_x"], "y": slot[f"slot_pt{index}_y"]}
                             for index in range(1, 5)],
                 **self.slot_provider.availability(slot),
                 "default_pose": self.approach_pose(slot),
                 "park_out_available": self.parkout_target(slot) is not None,
                 "park_out_target": self.parkout_target(slot)}
                for slot in sorted(self.slot_provider.slots.values(), key=lambda row: int(row["slot_id"]))
            ],
            "parking_enabled": bool(self.parking_enabled),
            "planning_only": bool(self.planning_only and self.parking_enabled),
            "sync_error": getattr(self, "_sync_fault", ""),
            "control_status": "safety_stop" if safety["reason"] else algo.get("control_status", ""),
            "safety_state": safety,
            "safety_stop_detail": self.safety_stop.detail,
            "control_diagnostic": diagnostic,
            "input_validation": algo.get("input_validation", {}),
            "online_safety": algo.get("online_safety", {}),
            "scene_diagnostic": scene_diagnostic,
            "replan": algo.get("replan", {}),
            "actuation_limits": algo.get("actuation_limits", {}),
            "pedal_calibration": {
                "coverage": None,
                "last_mapping": {"source": "c_pedals_v1",
                                 **algo.get("actuator", {})},
                "recording_path": getattr(self, "_calibration_recording_path", ""),
                "recording_error": getattr(self, "_calibration_record_error", ""),
                "recorded_frames": self.calibration_recorder.rows if getattr(self, "calibration_recorder", None) else 0,
            },
            "recovery": {"recovery_id": self._recovery_id, "stop": self.recovery_stop.payload()},
            "control_phase": int(read_number(algo, "CtrlPhase", default=0)),
            "gear_feedback_source": "carla_applied_control",
            "control_dt_sec": algo.get("control_dt_sec", 0.0),
            "control_step_count": algo.get("control_step_count", 0),
            "carla_frame_id": cached["frame_id"] if cached is not None else 0,
            "phase": self.apa_phase,
            "algorithm_linked": linked,
            "algorithm_has_output": linked,
            "waiting_for_path": bool(self.parking_enabled and len(path) <= 1 and
                                     replan.get("state") != "failed" and not safety["reason"]),
            "slot_id": int(self.target_slot.get("slot_id", 0)),
            "slot_entry": int(self.target_slot.get("slot_entry", 1)),
            "slot_kind": str(self.target_slot.get("slot_kind", self.target_slot.get("apa_slot_kind", ""))),
            "physical_slot_id": physical_slot_id(self.target_slot),
            "parking_direction": slot_choice_identity(self.target_slot)["parking_direction"],
            "parking_action": int(self.parking_action),
            "path_point_count": len(path),
            "gear": gear_name,
            "applied_gear": gear_name,
            "vehicle_speed": speed,
            "actual_speed_mps": speed,
            "actual_accel_mps2": accel_long,
            "throttle": finite(getattr(vehicle_control, "throttle", self.last_mapped_throttle)),
            "brake": finite(getattr(vehicle_control, "brake", self.last_mapped_brake)),
            "steer": finite(getattr(vehicle_control, "steer", self.applied_manual_steer)),
            "hand_brake": bool(getattr(vehicle_control, "hand_brake", self.manual_hand_brake)),
            "ignition_switch": self.manual_ignition,
            "camera_ready": self.rear_camera is not None,
            "camera_http": self.camera_http_url,
            "simulation_rate": getattr(self, "simulation_rate", None),
            "algorithm_target_accel": accel,
            "algorithm_target_speed": target_speed,
            "algorithm_steer_rad": steer_rad,
            "algorithm_gear": algo_gear if algo else "",
            "algorithm_gear_req": gear_req,
            "expected_speed_mps": target_speed,
            "expected_accel_mps2": accel,
            "expected_brake": self.last_mapped_brake,
            "expected_throttle": self.last_mapped_throttle,
            "expected_gear": algo_gear if self.parking_enabled else "",
            "expected_steer": steer_rad,
            "expected_steer_wheel_deg": math.degrees(steer_rad),
            "actual_steer_wheel_deg": applied_steer_wheel_deg,
            "actual_steer_wheel_deg_semantics": "applied_equivalent_bicycle_angle",
            "applied_steer_wheel_deg": applied_steer_wheel_deg,
            "applied_steer_wheel_deg_source": "carla_get_control_ackermann_inverse",
            "steering_observation": steering_observation,
            "expected_x": expected_x,
            "expected_y": expected_y,
            "vehicle_x": ego_x,
            "vehicle_y": ego_y,
            "vehicle_yaw": ego_yaw,
            "vehicle_reference_point": REAR_AXLE_REFERENCE_POINT,
            "path_reference_point": REAR_AXLE_REFERENCE_POINT,
            "carla_actor_x": actor_x,
            "carla_actor_y": actor_y,
            "algorithm_path_x": [point[0] for point in display_path],
            "algorithm_path_y": [point[1] for point in display_path],
            "algorithm_path_yaw": display_plan.get("path_point_yaw", []),
            "algorithm_path_odo": display_plan.get("path_point_odo", []),
            "trajectory": {
                "path_version": display_plan.get("replan", {}).get("path_version", 0),
                "retained": bool(display_path) and (not path or not self.parking_enabled),
                "previous_plans": [
                    {"path_version": item["replan"]["path_version"],
                     "x": item.get("path_point_x", []), "y": item.get("path_point_y", [])}
                    for item in list(self.accepted_plan_history)[:-1]],
                "tracking_error": dict(self.tracking_error_summary),
            },
            "actual_path_x": [point[0] for point in actual],
            "actual_path_y": [point[1] for point in actual],
            "slot_corners": corners,
            "algorithm_message": (
                safety_message if safety_message else
                f"规划失败：{planning_reason}。车辆保持停车，可退出 APA 后调整起点重试"
                if self.parking_enabled and algo.get("control_status") == "plan_failed" else
                "正在停车，停稳后重新规划" if self.parking_enabled and algo.get("control_status") == "replan_stopping" else
                "正在制动，停稳复核后沿原路径低速继续" if self.parking_enabled and algo.get("control_status") == "motion_braking" else
                f"暂未得到可执行路径：{planning_reason}；等待场景变化后重试，车辆保持制动"
                if self.parking_enabled and algo.get("control_status") == "replan_waiting" else
                "正在规划，车辆保持制动" if self.parking_enabled and algo.get("control_status") == "planning" else
                "路径已就绪，车辆保持驻车" if self.planning_only and algo.get("control_status") == "plan_ready" else
                "泊车到位合格" if self.parking_enabled and algo.get("control_status") == "complete" else
                f"跟踪失败，车辆保持制动：{stop_messages.get(diagnostic.get('stop_reason'), '控制保护')}；请重置后重试"
                if self.parking_enabled and algo.get("control_status") == "tracking_fault" else
                "泊车执行中" if self.parking_enabled else "等待开始泊入或泊出"),
            "plan_success": None if not isinstance(plan.get("success", None), bool) and plan.get("success") is None else bool(plan.get("success")),
        }

    def draw_debug(self) -> None:
        if not bool(self.cfg["apa"].get("draw_debug", True)) or self.world is None or self.carla is None:
            return
        target_id = int(self.target_slot.get("slot_id", -1))
        for slot in self.slot_provider.slots.values():
            is_target = int(slot.get("slot_id", -1)) == target_id
            if not is_target:
                continue
            color = self.carla.Color(0, 60, 120)
            pts = [
                self.carla.Location(finite(slot.get("slot_pt1_x")), finite(slot.get("slot_pt1_y")), self.sample_ground_z(finite(slot.get("slot_pt1_x")), finite(slot.get("slot_pt1_y")), allow_query=False) + 0.12),
                self.carla.Location(finite(slot.get("slot_pt2_x")), finite(slot.get("slot_pt2_y")), self.sample_ground_z(finite(slot.get("slot_pt2_x")), finite(slot.get("slot_pt2_y")), allow_query=False) + 0.12),
                self.carla.Location(finite(slot.get("slot_pt3_x")), finite(slot.get("slot_pt3_y")), self.sample_ground_z(finite(slot.get("slot_pt3_x")), finite(slot.get("slot_pt3_y")), allow_query=False) + 0.12),
                self.carla.Location(finite(slot.get("slot_pt4_x")), finite(slot.get("slot_pt4_y")), self.sample_ground_z(finite(slot.get("slot_pt4_x")), finite(slot.get("slot_pt4_y")), allow_query=False) + 0.12),
            ]
            for first, second in zip(pts, pts[1:] + pts[:1]):
                self.world.debug.draw_line(first, second, thickness=0.02, color=color, life_time=0.1)
            center = self.carla.Location(
                sum(point.x for point in pts) / 4.0,
                sum(point.y for point in pts) / 4.0,
                max(point.z for point in pts) + 0.35,
            )
            self.world.debug.draw_string(center, f"APA {target_id}", draw_shadow=False, color=color, life_time=0.1)
        if bool(self.cfg["apa"].get("draw_path_debug", False)) and self.last_plan_result:
            xs = self.last_plan_result.get("path_point_x", self.last_plan_result.get("PthPtX", []))
            ys = self.last_plan_result.get("path_point_y", self.last_plan_result.get("PthPtY", []))
            if isinstance(xs, list) and isinstance(ys, list):
                for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]):
                    p1 = self.carla.Location(finite(x1), finite(y1), self.sample_ground_z(finite(x1), finite(y1), allow_query=False) + 0.25)
                    p2 = self.carla.Location(finite(x2), finite(y2), self.sample_ground_z(finite(x2), finite(y2), allow_query=False) + 0.25)
                    self.world.debug.draw_line(p1, p2, thickness=0.02, color=self.carla.Color(0, 80, 140), life_time=0.1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="APA CARLA <-> Pangu ZMQ bridge")
    parser.add_argument("--config", default="", help="bridge config json")
    parser.add_argument("--dry-run", action="store_true", help="start ZMQ without CARLA")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_dir = os.getcwd()
    override = {}
    if args.config:
        config_dir = str(Path(args.config).resolve().parent)
        override = load_json(args.config)
    if any(key in override.get("apa", {}) for key in ("vehicle_geometry", "rear_axle_offset_m", "vehicle_profile_id")):
        raise SystemExit("Vehicle dimensions and rear axle offset belong in apa.vehicle_profile_path, not bridge_config.json")
    profile_path = Path(override.get("apa", {}).get("vehicle_profile_path", DEFAULT_VEHICLE_PROFILE_PATH))
    if not profile_path.is_absolute():
        profile_path = Path(config_dir) / profile_path
    cfg = deep_merge(default_config(profile_path), override)
    cfg["_config_dir"] = config_dir
    raw_fixed_delta = os.environ.get("CARLA_FIXED_DELTA_SECONDS")
    if raw_fixed_delta:
        try:
            fixed_delta = float(raw_fixed_delta)
        except ValueError as exc:
            raise SystemExit("CARLA_FIXED_DELTA_SECONDS must be a number") from exc
        if not 0.0 < fixed_delta <= 0.1:
            raise SystemExit("CARLA_FIXED_DELTA_SECONDS must be within (0, 0.1]")
        cfg.setdefault("carla", {})["fixed_delta_seconds"] = fixed_delta
    transition = json.loads(os.environ.pop("APA_BRIDGE_MAP_TRANSITION", "null"))
    if transition:
        cfg = transition["config"]
    bridge = ApaCarlaBridge(cfg, dry_run=args.dry_run)
    bridge.map_bootstrap_command = transition["command"] if transition else None
    bridge.run()
    if bridge.pending_map_config is not None:
        if bridge.safety_stop.state != "stopped":
            raise RuntimeError("地图切换取消：车辆未确认停稳")

        env = dict(os.environ)
        env["APA_BRIDGE_MAP_TRANSITION"] = json.dumps({
            "config": bridge.pending_map_config, "command": bridge.pending_map_command})

        for entry in Path("/proc/self/fd").iterdir():
            descriptor = int(entry.name)
            if descriptor > 2:
                try:
                    os.set_inheritable(descriptor, False)
                except OSError:
                    pass
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]], env)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
