"""APA pedal measurements: same-frame recording and measured low-speed curves."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Optional


def vehicle_identity(vehicle: Any) -> Dict[str, Any]:
    physics = vehicle.get_physics_control()
    scalars = ("mass", "drag_coefficient", "max_rpm", "damping_rate_full_throttle",
               "damping_rate_zero_throttle_clutch_engaged", "damping_rate_zero_throttle_clutch_disengaged",
               "gear_switch_time", "clutch_strength", "final_ratio", "use_gear_autobox", "use_sweep_wheel_collision")
    values = {key: getattr(physics, key) for key in scalars if hasattr(physics, key)}
    values["center_of_mass"] = [getattr(physics.center_of_mass, k) for k in ("x", "y", "z")]
    values["torque_curve"] = [[p.x, p.y] for p in physics.torque_curve]
    values["steering_curve"] = [[p.x, p.y] for p in physics.steering_curve]
    values["forward_gears"] = [{k: getattr(g, k) for k in ("ratio", "down_ratio", "up_ratio")}
                               for g in physics.forward_gears]
    wheel_fields = ("tire_friction", "damping_rate", "max_steer_angle", "radius", "max_brake_torque",
                    "max_handbrake_torque", "lat_stiff_max_load", "lat_stiff_value", "long_stiff_value")
    values["wheels"] = [{k: getattr(w, k) for k in wheel_fields if hasattr(w, k)} for w in physics.wheels]
    canonical = json.dumps(values, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return {"blueprint": vehicle.type_id, "physics_sha256": hashlib.sha256(canonical.encode()).hexdigest(),
            "physics": values}


def _dot(vector: Any, axis: Any) -> float:
    return vector.x * axis.x + vector.y * axis.y + vector.z * axis.z


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class PedalRecorder:
    """Observe the Bridge's own tick; never drive an actor or advance CARLA."""

    fields = ("frame_id", "sim_time_sec", "cycle_time_sec", "gear", "direction", "speed_mps",
              "end_speed_mps", "lateral_speed_mps", "accel_mps2", "carla_accel_mps2", "slope_deg",
              "throttle", "brake", "steer", "hand_brake", "command_age_sec", "faulted")

    def __init__(self, path: Path, metadata: Dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.handle = path.open("x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.handle, fieldnames=self.fields)
        self.writer.writeheader()
        self.metadata = dict(metadata, schema_version=1, complete=False, rows=0, skipped_frames=0)
        self.rows = self.skipped_frames = 0
        self._last_command = None
        self._command_age = 0.0
        self._write_metadata()

    def _write_metadata(self) -> None:
        self.path.with_suffix(".json").write_text(json.dumps(self.metadata, indent=2, allow_nan=False) + "\n")

    def record(self, before: Any, after: Any, actor_id: int, command: Any, faulted: bool) -> None:
        first, last = before.find(actor_id), after.find(actor_id)
        dt = after.timestamp.elapsed_seconds - before.timestamp.elapsed_seconds
        if first is None or last is None or after.frame != before.frame + 1 or not 0 < dt <= 0.05:
            self.skipped_frames += 1
            self._last_command = None
            self._command_age = 0.0
            return
        axis = first.get_transform().get_forward_vector()
        velocity, end_velocity = first.get_velocity(), last.get_velocity()
        speed, end_speed = _dot(velocity, axis), _dot(end_velocity, axis)
        key = (command.throttle, command.brake, command.steer, command.gear, command.hand_brake)
        self._command_age = self._command_age + dt if key == self._last_command else 0.0
        self._last_command = key
        row = {"frame_id": after.frame, "sim_time_sec": after.timestamp.elapsed_seconds, "cycle_time_sec": dt,
               "gear": command.gear, "direction": -1 if command.gear < 0 else 1 if command.gear > 0 else 0,
               "speed_mps": speed, "end_speed_mps": end_speed,
               "lateral_speed_mps": math.sqrt(max(0.0, _dot(velocity, velocity) - speed * speed)),
               "accel_mps2": (end_speed - speed) / dt,
               "carla_accel_mps2": _dot(last.get_acceleration(), axis),
               "slope_deg": math.degrees(math.asin(max(-1.0, min(axis.z, 1.0)))),
               "throttle": command.throttle, "brake": command.brake, "steer": command.steer,
               "hand_brake": int(command.hand_brake), "command_age_sec": self._command_age,
               "faulted": int(faulted)}
        if not all(math.isfinite(float(value)) for value in row.values()):
            self.skipped_frames += 1
            return
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 50 == 0:
            self.handle.flush()

    def close(self, complete: bool = True) -> None:
        if self.handle.closed:
            return
        self.handle.flush()
        self.handle.close()
        self.metadata.update(complete=complete, rows=self.rows, skipped_frames=self.skipped_frames,
                             csv_sha256=_file_sha256(self.path))
        self._write_metadata()


def measurement_eligible(row, settle_sec):
    direction, speed = row["direction"], abs(row["speed_mps"])
    throttle, brake = row["throttle"], row["brake"]
    return (direction in (-1, 1) and row["speed_mps"] * direction >= -0.02 and
            row["end_speed_mps"] * direction >= -0.02 and speed <= 1.5 and
            abs(row["slope_deg"]) <= 0.5 and abs(row["steer"]) <= 0.01 and
            row["lateral_speed_mps"] <= 0.05 and not row["hand_brake"] and not row["faulted"] and
            row["command_age_sec"] + 1.e-6 >= settle_sec and
            0 <= throttle <= 1 and 0 <= brake <= 1 and not (throttle > 0 and brake > 0) and
            (brake == 0 or (speed > 0.05 and abs(row["end_speed_mps"]) > 0.03)))


def export_profile(paths, output: Path, settle_sec: float = 0.2, speed_step: float = 0.1,
                   min_samples: int = 5, max_stddev: float = 0.25) -> Dict[str, Any]:
    if (not all(math.isfinite(v) and v > 0 for v in (settle_sec, speed_step, max_stddev)) or min_samples < 2):
        raise ValueError("export settle/speed-step/stddev must be positive; min-samples must be >=2")
    groups = defaultdict(list)
    identity = None
    sources = []
    rejected = 0
    for path in map(Path, paths):
        meta = json.loads(path.with_suffix(".json").read_text())
        digest = _file_sha256(path)
        if meta.get("schema_version") != 1 or not meta.get("complete") or meta.get("csv_sha256") != digest:
            raise ValueError(f"incomplete or changed measurement recording: {path}")
        if meta.get("measurement_accepted") is False or meta.get("initialization") == "one_initial_velocity_before_pulse":
            raise ValueError(f"rejected measurement/forced-body-speed initialization; collect with natural pedal preparation: {path}")
        key = {k: meta[k] for k in ("vehicle", "carla_version", "fixed_delta_seconds")}
        if identity is not None and key != identity:
            raise ValueError("cannot combine different vehicles, physics, CARLA versions or time steps")
        identity = key
        sources.append({"path": str(path.resolve()), "sha256": digest, "map_name": meta["map_name"]})
        with path.open(newline="", encoding="utf-8") as handle:
            for source in csv.DictReader(handle):
                row = {k: float(source[k]) for k in PedalRecorder.fields}
                if not all(math.isfinite(v) for v in row.values()):
                    raise ValueError(f"non-finite measurement: {path}")
                direction, speed = row["direction"], abs(row["speed_mps"])
                throttle, brake = row["throttle"], row["brake"]
                if not measurement_eligible(row, settle_sec):
                    rejected += 1
                    continue
                mode, pedal = ("brake", brake) if brake > 0 else ("throttle", throttle)
                groups[(int(direction), mode, round(pedal, 4), round(speed / speed_step))].append(
                    (speed, row["accel_mps2"]))
    if identity is None:
        raise ValueError("no measurement recordings supplied")
    points = []
    for (direction, mode, pedal, _), samples in sorted(groups.items()):
        if len(samples) < min_samples:
            continue
        accel = sum(p[1] for p in samples) / len(samples)
        deviation = math.sqrt(sum((p[1] - accel) ** 2 for p in samples) / (len(samples) - 1))
        if deviation > max_stddev:
            continue
        points.append({"direction": direction, "mode": mode, "pedal": pedal,
                       "speed_mps": sum(p[0] for p in samples) / len(samples), "accel_mps2": accel,
                       "samples": len(samples), "stddev_mps2": deviation})
    if not points or not any(p["pedal"] == 0 for p in points):
        raise ValueError("no usable measurements including actual zero-throttle/coast samples")
    profile = dict(identity, schema_version=1, sources=sources, max_abs_slope_deg=0.5,
                   max_speed_gap_mps=0.25, max_pedal_gap=0.25, points=points,
                   export_settings={"settle_sec": settle_sec, "speed_step": speed_step,
                                    "min_samples": min_samples, "max_stddev_mps2": max_stddev}, rejected_rows=rejected)

    MeasuredPedalProfile(profile)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(profile, indent=2, allow_nan=False) + "\n")
    return profile


class MeasuredPedalProfile:
    def __init__(self, data: Dict[str, Any]) -> None:
        if data.get("schema_version") != 1 or not data.get("sources") or not data.get("vehicle"):
            raise ValueError("invalid measured pedal profile provenance/schema")
        if (type(data.get("max_abs_slope_deg")) not in (int, float) or
                not math.isfinite(data["max_abs_slope_deg"]) or not 0 <= data["max_abs_slope_deg"] <= 0.5 or
                type(data.get("max_speed_gap_mps")) not in (int, float) or
                not math.isfinite(data["max_speed_gap_mps"]) or not 0 < data["max_speed_gap_mps"] <= 0.25 or
                type(data.get("max_pedal_gap")) not in (int, float) or
                not math.isfinite(data["max_pedal_gap"]) or not 0 < data["max_pedal_gap"] <= 0.25 or
                type(data.get("fixed_delta_seconds")) not in (int, float) or
                not math.isfinite(data["fixed_delta_seconds"]) or not 0 < data["fixed_delta_seconds"] <= 0.05):
            raise ValueError("invalid measured pedal profile coverage/time step")
        self.data = data
        self.curves = defaultdict(list)
        for point in data["points"]:
            direction, mode, pedal = point["direction"], point["mode"], point["pedal"]
            values = (pedal, point["speed_mps"], point["accel_mps2"], point["stddev_mps2"])
            if (type(direction) is not int or direction not in (-1, 1) or mode not in ("throttle", "brake") or
                    not all(type(v) in (int, float) and math.isfinite(v) for v in values) or
                    not 0 <= pedal <= 1 or not 0 <= point["speed_mps"] <= 1.5 or
                    type(point["samples"]) is not int or point["samples"] < 2 or point["stddev_mps2"] < 0):
                raise ValueError("invalid measured pedal profile point")
            if mode == "brake" and pedal == 0:
                raise ValueError("zero-pedal coast points belong to throttle mode")
            self.curves[(direction, mode, pedal)].append((point["speed_mps"], point["accel_mps2"]))
        for points in self.curves.values():
            points.sort()
            if len({p[0] for p in points}) != len(points):
                raise ValueError("duplicate measured speed within a pedal curve")
        self.bound = False

    def bind_vehicle(self, identity: Dict[str, Any], step: float, version: str) -> None:
        self.bound = False
        if self.data.get("validation", {}).get("activation_ready") is False:
            raise ValueError("measured pedal candidate did not pass activation checks; inspect its validation results")
        recorded = self.data["vehicle"]
        if (identity["blueprint"] != recorded["blueprint"] or
                identity["physics_sha256"] != recorded["physics_sha256"] or
                abs(self.data["fixed_delta_seconds"] - step) > 1.e-6 or self.data["carla_version"] != version):
            raise ValueError("measured pedal profile does not match vehicle physics/CARLA/time step")
        self.bound = True

    def _at_speed(self, direction: int, mode: str, pedal: float, speed: float) -> Optional[float]:
        points = self.curves.get((direction, mode, pedal), [])
        for x, value in points:
            if abs(speed - x) <= 1.e-6:
                return value
        for (first, a), (last, b) in zip(points, points[1:]):
            if first < speed < last and last - first <= self.data["max_speed_gap_mps"] + 1.e-6:
                return a + (b - a) * (speed - first) / (last - first)
        return None

    def coast(self, direction: int, speed: float, slope: float) -> Optional[float]:
        if not self.bound or abs(slope) > self.data["max_abs_slope_deg"]:
            return None
        return self._at_speed(direction, "throttle", 0.0, speed)

    def invert(self, direction: int, mode: str, speed: float, target: float,
               coast: float, maximum: float) -> Optional[tuple]:
        curve = [(0.0, coast)]
        for d, m, pedal in self.curves:
            if d == direction and m == mode and 0 < pedal <= maximum:
                value = self._at_speed(direction, mode, pedal, speed)
                if value is not None:
                    curve.append((pedal, value))
        curve.sort()
        if len(curve) < 2:
            return None
        candidates = [(abs(value - target), pedal, value) for pedal, value in curve]
        for (left, a), (right, b) in zip(curve, curve[1:]):
            if right - left <= self.data["max_pedal_gap"] + 1.e-6 and abs(b - a) > 1.e-9:
                fraction = max(0.0, min((target - a) / (b - a), 1.0))
                pedal, value = left + fraction * (right - left), a + fraction * (b - a)
                candidates.append((abs(value - target), pedal, value))
        error, pedal, predicted = min(candidates)
        if error > 0.12:
            return None
        return pedal, {"source": "measured_low_speed_" + mode, "table_error": error,
                       "predicted_accel_mps2": predicted, "coast_acc": coast}

    def summary(self, include_curves: bool = False) -> Dict[str, Any]:
        result = {"bound_to_vehicle": self.bound, "points": len(self.data["points"]),
                "coast_directions": sorted({d for d, m, pedal in self.curves if m == "throttle" and pedal == 0}),
                "brake_directions": sorted({d for d, m, pedal in self.curves if m == "brake"}),
                "sources": self.data["sources"], "vehicle": self.data["vehicle"]["blueprint"]}
        if include_curves:
            result["max_interpolation_gap_mps"] = self.data["max_speed_gap_mps"]
            result["max_pedal_interpolation_gap"] = self.data["max_pedal_gap"]
            result["curves"] = [{"direction": d, "mode": m, "pedal": pedal,
                                 "speed_points_mps": [p[0] for p in values]}
                                for (d, m, pedal), values in sorted(self.curves.items())]
        return result
