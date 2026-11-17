#!/usr/bin/env python3
"""Generate APA vehicle constants from the shared vehicle profile, without CARLA."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def generate(profile_path, output_path):
    raw = Path(profile_path).read_bytes()
    profile = json.loads(raw)
    fields = {
        "width_m": "APA_VEHICLE_WIDTH_M",
        "wheelbase_m": "APA_VEHICLE_WHEELBASE_M",
        "rear_to_front_m": "APA_VEHICLE_REAR_TO_FRONT_M",
        "rear_to_back_m": "APA_VEHICLE_REAR_TO_BACK_M",
        "rear_axle_offset_m": "APA_VEHICLE_REAR_AXLE_OFFSET_M",
        "max_steer_rad": "APA_VEHICLE_MAX_STEER_RAD",
        "planning_min_turn_radius_m": "APA_VEHICLE_MIN_TURN_RADIUS_M",
        "parkin_margin_m": "APA_PARKIN_MARGIN_M",
        "parkin_physical_margin_m": "APA_PARKIN_PHYSICAL_MARGIN_M",
        "parkin_back_margin_m": "APA_PARKIN_BACK_MARGIN_M",
        "goal_position_tolerance_m": "APA_DEFAULT_GOAL_POSITION_TOLERANCE_M",
        "goal_yaw_tolerance_rad": "APA_DEFAULT_GOAL_YAW_TOLERANCE_RAD",
    }
    for field in fields:
        value = profile.get(field)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{field} must be finite and positive")
    if profile["parkin_back_margin_m"] > profile["parkin_margin_m"]:
        raise ValueError("parkin_back_margin_m must not exceed parkin_margin_m")
    if profile["parkin_physical_margin_m"] < profile["parkin_margin_m"]:
        raise ValueError("parkin_physical_margin_m must be at least parkin_margin_m")
    chamfer_fields = {
        "front_chamfer_lon_m": "APA_VEHICLE_FRONT_CHAMFER_LON_M",
        "front_chamfer_lat_m": "APA_VEHICLE_FRONT_CHAMFER_LAT_M",
        "rear_chamfer_lon_m": "APA_VEHICLE_REAR_CHAMFER_LON_M",
        "rear_chamfer_lat_m": "APA_VEHICLE_REAR_CHAMFER_LAT_M",
    }
    for field in chamfer_fields:
        value = profile.get(field)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{field} must be finite and non-negative")
    front_overhang = profile["rear_to_front_m"] - profile["wheelbase_m"]
    if (profile["front_chamfer_lon_m"] >= front_overhang or
            profile["rear_chamfer_lon_m"] >= profile["rear_to_back_m"] or
            profile["front_chamfer_lat_m"] >= profile["width_m"] * 0.45 or
            profile["rear_chamfer_lat_m"] >= profile["width_m"] * 0.45):
        raise ValueError("invalid vehicle corner chamfer dimensions")
    fields.update(chamfer_fields)
    for field in ("blueprint", "carla_version", "dimension_source"):
        if not isinstance(profile.get(field), str) or not profile[field]:
            raise ValueError(f"{field} must be a nonempty string")
    if (profile["rear_to_front_m"] <= profile["wheelbase_m"] or
            profile["max_steer_rad"] >= math.pi / 2):
        raise ValueError("invalid front overhang or steering lock")
    model = profile.get("steering_model", "bicycle")
    if model == "ackermann_inner_wheel":
        for field in ("front_track_m", "carla_max_steer_rad"):
            value = profile.get(field)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{field} must be finite and positive")
        if profile["carla_max_steer_rad"] >= math.pi / 2:
            raise ValueError("invalid inner-wheel steering lock")
        radius = (profile["wheelbase_m"]/math.tan(profile["carla_max_steer_rad"]) +
                  profile["front_track_m"]/2)
        equivalent = math.atan(profile["wheelbase_m"]/radius)
        if (abs(equivalent-profile["max_steer_rad"]) > 1.e-8 or
                profile["planning_min_turn_radius_m"] <= radius):
            raise ValueError("Ackermann lock mismatch or missing planning steering headroom")
        fields.update(front_track_m="APA_VEHICLE_FRONT_TRACK_M",
                      carla_max_steer_rad="APA_VEHICLE_CARLA_MAX_STEER_RAD")
    elif model != "bicycle":
        raise ValueError("unsupported steering_model")
    lines = ["/* Generated from apa_vehicle_profile.json; do not edit. */",
             "#ifndef APA_VEHICLE_PROFILE_GENERATED_H",
             "#define APA_VEHICLE_PROFILE_GENERATED_H",
             f'#define APA_VEHICLE_PROFILE_ID "{hashlib.sha256(raw).hexdigest()}"']
    for field, macro in fields.items():
        literal = format(float(profile[field]), ".12g")
        if "." not in literal and "e" not in literal:
            literal += ".0"
        lines.append(f"#define {macro} {literal}f")
    for field, macro in (("blueprint", "APA_VEHICLE_BLUEPRINT"),
                         ("carla_version", "APA_VEHICLE_CARLA_VERSION")):
        lines.append(f"#define {macro} {json.dumps(profile[field])}")
    lines += ["#endif", ""]
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    content = "\n".join(lines)
    if not output.exists() or output.read_text() != content:
        output.write_text(content)

    model_lines = [
        "// Generated from apa_vehicle_profile.json; do not edit.",
        "#pragma once", "", '#include "FuncModule.hpp"', "",
        "namespace apa {", "",
        "/** @brief 多个规划模块共享的车辆几何及规划标定集合。 */",
        "struct ApaPlanningCalibration {",
    ]
    for field, member, description in (
        ("width_m", "vehicleWidth", "车辆包络宽度，单位 m。"),
        ("rear_to_front_m", "vehicleFrontEnvelopeLength", "后轴到车身前包络的长度，单位 m。"),
        ("rear_to_back_m", "vehicleRearOverhang", "后轴到车身后包络的长度，单位 m。"),
    ):
        literal = format(float(profile[field]), ".12g")
        if "." not in literal and "e" not in literal:
            literal += ".0"
        model_lines.append(f"  Real {member} = {literal}; ///< {description}")
    model_lines += [
        "  Real minimumFrontSafetyLength = 0.4; ///< 最小前向安全长度，单位 m。",
        "  Real verticalFallbackSpace = 4.1; ///< 垂直车位边界探测失败回退空间，单位 m。",
        "  Real diagonalFallbackSpace = 6.0; ///< 斜车位边界探测失败回退空间，单位 m。",
        "  Real parallelMinimumExitRadius = 0.0; ///< 平行车位外侧前角扫掠半径，单位 m；0 按 APA_Cal 计算。",
        "};", "", "}  // namespace apa", "",
    ]
    model_output = output.with_name("ApaPlanningCalibration.hpp")
    model_content = "\n".join(model_lines)
    if not model_output.exists() or model_output.read_text() != model_content:
        model_output.write_text(model_content)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    generate(args.profile, args.output)
