#!/usr/bin/env python3
"""Convert target acceleration to throttle/brake using the 8 calibration tables.

Tables
------
- flat_acceleration_map.csv          slope ~ 0
- up3/up5/up8.csv                    +3 / +5 / +8 deg
- down3/down5/down8.csv              -3 / -5 / -8 deg
- reverse_acceleration_map.csv       reverse gear

Each table maps (speed_kmh, throttle) -> measured longitudinal accel.
Measured low-speed coast/throttle/brake curves can override covered operating
points. Missing zero-pedal data is reported as unknown; uncovered braking uses
an explicitly unmeasured linear model instead of borrowing a throttle sample.
"""

from __future__ import annotations

import csv
import json
import math
import os
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .apa_pedal_measurements import MeasuredPedalProfile, vehicle_identity


SLOPE_LAYERS = (-8.0, -5.0, -3.0, 0.0, 3.0, 5.0, 8.0)
SLOPE_FILES = {
    -8.0: "down8.csv",
    -5.0: "down5.csv",
    -3.0: "down3.csv",
    0.0: "flat_acceleration_map.csv",
    3.0: "up3.csv",
    5.0: "up5.csv",
    8.0: "up8.csv",
}


def _default_data_dir() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _finite(value: object, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if math.isfinite(number):
        return number
    return default


def _bracket(axis: Sequence[float], value: float) -> Tuple[int, float]:
    if value <= axis[0]:
        return 0, 0.0
    if value >= axis[-1]:
        return max(0, len(axis) - 2), 1.0
    for index in range(len(axis) - 1):
        left = axis[index]
        right = axis[index + 1]
        if left <= value <= right:
            span = right - left
            mix = 0.0 if span <= 1.0e-12 else (value - left) / span
            return index, mix
    return max(0, len(axis) - 2), 1.0


def _interp2(x_axis: Sequence[float], y_axis: Sequence[float], table: List[List[float]], x: float, y: float) -> float:
    i, tx = _bracket(x_axis, x)
    j, ty = _bracket(y_axis, y)
    v00 = table[i][j]
    v10 = table[i + 1][j]
    v01 = table[i][j + 1]
    v11 = table[i + 1][j + 1]
    return (1.0 - tx) * (1.0 - ty) * v00 + tx * (1.0 - ty) * v10 + (1.0 - tx) * ty * v01 + tx * ty * v11


def _load_speed_throttle_table(
    path: str,
    speed_key: str,
    throttle_key: str,
    acc_key: str,
) -> Tuple[List[float], List[float], List[List[float]]]:
    buckets: Dict[Tuple[float, float], List[float]] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            speed, throttle, acc = (float(row[key]) for key in (speed_key, throttle_key, acc_key))
            if not all(math.isfinite(v) for v in (speed, throttle, acc)) or not 0 <= throttle <= 1:
                raise ValueError("invalid calibration sample: " + path)
            speed, throttle = round(speed, 4), round(throttle, 4)
            buckets.setdefault((speed, throttle), []).append(acc)

    speeds = sorted({key[0] for key in buckets})
    throttles = sorted({key[1] for key in buckets})
    if len(speeds) < 2 or len(throttles) < 2:
        raise ValueError("calibration table needs at least two speeds and pedals: " + path)
    table = [[0.0 for _ in throttles] for _ in speeds]
    for i, speed in enumerate(speeds):
        for j, throttle in enumerate(throttles):
            samples = buckets.get((speed, throttle))
            if samples:
                table[i][j] = sum(samples) / len(samples)
            else:
                raise ValueError("missing measured calibration grid cell: " + path)
    return speeds, throttles, table


class AccelPedalMap:
    """8-table inverse map: target_acc + speed + slope -> throttle/brake."""

    def __init__(self, data_dir: Optional[str] = None, max_brake_mps2: float = 6.0,
                 low_speed_profile_path: Optional[str] = None) -> None:
        self.data_dir = data_dir or _default_data_dir()
        self.max_brake_mps2 = max(0.5, float(max_brake_mps2))
        self.slope_axis = list(SLOPE_LAYERS)
        self.speed_axis: List[float] = []
        self.throttle_axis: List[float] = []
        self.forward_layers: List[List[List[float]]] = []
        self.reverse_speed_axis: List[float] = []
        self.reverse_throttle_axis: List[float] = []
        self.reverse_table: List[List[float]] = []
        self._load_all()
        self.profile = None
        self.profile_path = low_speed_profile_path
        if low_speed_profile_path:
            with open(low_speed_profile_path, encoding="utf-8") as handle:
                self.profile = MeasuredPedalProfile(json.load(handle))

    def _path(self, name: str) -> str:
        return os.path.join(self.data_dir, name)

    def _load_all(self) -> None:
        missing = [name for name in list(SLOPE_FILES.values()) + ["reverse_acceleration_map.csv"] if not os.path.isfile(self._path(name))]
        if missing:
            raise FileNotFoundError("missing calibration tables: " + ", ".join(missing))

        layers = []
        speed_axis = None
        throttle_axis = None
        for slope in self.slope_axis:
            speeds, throttles, table = _load_speed_throttle_table(
                self._path(SLOPE_FILES[slope]),
                "speed" if slope == 0.0 else "target_speed",
                "throttle",
                "avg_acc" if slope == 0.0 else "mean_acc",
            )
            if speed_axis is None:
                speed_axis, throttle_axis = speeds, throttles
            elif speeds != speed_axis or throttles != throttle_axis:
                raise ValueError("legacy slope tables must have matching measured axes")
            layers.append(table)
        self.speed_axis = speed_axis or []
        self.throttle_axis = throttle_axis or []
        self.forward_layers = layers
        (
            self.reverse_speed_axis,
            self.reverse_throttle_axis,
            self.reverse_table,
        ) = _load_speed_throttle_table(
            self._path("reverse_acceleration_map.csv"),
            "speed",
            "throttle",
            "acc",
        )

    def predict_acc(self, speed_kmh: float, throttle: float, slope_deg: float = 0.0, reverse: bool = False) -> float:
        throttle = _clamp(throttle, 0.0, 1.0)
        minimum = self.reverse_throttle_axis[0] if reverse else self.throttle_axis[0]
        if throttle < minimum - 1.e-9:
            raise ValueError("no measured legacy throttle sample below " + str(minimum))
        if reverse:
            speed = -abs(speed_kmh)
            return _interp2(
                self.reverse_speed_axis,
                self.reverse_throttle_axis,
                self.reverse_table,
                speed,
                throttle,
            )
        speed = _clamp(speed_kmh, self.speed_axis[0], self.speed_axis[-1])
        slope = _clamp(slope_deg, self.slope_axis[0], self.slope_axis[-1])
        layer_index, mix = _bracket(self.slope_axis, slope)
        acc0 = _interp2(self.speed_axis, self.throttle_axis, self.forward_layers[layer_index], speed, throttle)
        acc1 = _interp2(self.speed_axis, self.throttle_axis, self.forward_layers[layer_index + 1], speed, throttle)
        return (1.0 - mix) * acc0 + mix * acc1

    def bind_vehicle(self, vehicle: Any, step: float, version: str) -> None:
        if self.profile is not None:
            self.profile.bind_vehicle(vehicle_identity(vehicle), step, version)

    def summary(self) -> Dict[str, Any]:
        return {"legacy_zero_throttle_forward": self.throttle_axis[0] == 0.0,
                "legacy_zero_throttle_reverse": self.reverse_throttle_axis[0] == 0.0,
                "low_speed_profile_path": self.profile_path,
                "measured_profile": self.profile.summary() if self.profile is not None else None,
                "brake_fallback": "linear_model_when_measured_coverage_missing"}

    def lookup(
        self,
        target_acc: float,
        speed_kmh: float,
        slope_deg: float = 0.0,
        reverse: bool = False,
        max_throttle: float = 1.0,
        max_brake: float = 1.0,
    ) -> Tuple[float, float, Dict[str, Any]]:
        target_acc = _finite(target_acc)
        speed_kmh = _finite(speed_kmh)
        slope_deg = _finite(slope_deg)
        max_throttle, max_brake = _clamp(max_throttle, 0.0, 1.0), _clamp(max_brake, 0.0, 1.0)
        direction = -1 if reverse else 1
        coast_acc = self.profile.coast(direction, abs(speed_kmh) / 3.6, slope_deg) if self.profile else None
        coast_source = "measured_low_speed" if coast_acc is not None else "missing"
        axis = self.reverse_throttle_axis if reverse else self.throttle_axis
        if coast_acc is None and axis[0] == 0.0:
            coast_acc = self.predict_acc(speed_kmh, 0.0, slope_deg, reverse)
            coast_source = "legacy_measured_zero"
        info = {"target_acc": target_acc, "coast_acc": coast_acc, "coast_source": coast_source,
                "speed_kmh": speed_kmh, "slope_deg": slope_deg,
                "profile_coast_covered": coast_source == "measured_low_speed"}
        # With no zero-pedal data, do not borrow the minimum positive-throttle
        # sample as coasting. The remaining brake model is explicitly unmeasured.
        deadband = 0.12
        braking = direction * target_acc < (direction * coast_acc - deadband if coast_acc is not None else 0.0)
        mode = "brake" if braking else "throttle"
        maximum = max_brake if braking else max_throttle
        if self.profile and coast_source == "measured_low_speed":
            measured = self.profile.invert(direction, mode, abs(speed_kmh) / 3.6,
                                           target_acc, coast_acc, maximum)
            if measured is not None:
                pedal, measured_info = measured
                info.update(measured_info)
                return (0.0, pedal, info) if braking else (pedal, 0.0, info)
        if braking:
            brake_gap = direction * ((coast_acc if coast_acc is not None else 0.0) - target_acc)
            brake = _clamp(brake_gap / self.max_brake_mps2, 0.0, max_brake)
            info.update(source="linear_brake_fallback", table_error=None)
            return 0.0, brake, info
        if coast_acc is None and direction * target_acc <= 0.0:
            info.update(source="unmeasured_coast", table_error=None)
            return 0.0, 0.0, info
        minimum, maximum = axis[0], min(max_throttle, axis[-1])
        if maximum < minimum:
            info.update(source="legacy_throttle_range_missing", table_error=None)
            return 0.0, 0.0, info
        candidates = []
        for step in range(201):
            throttle = minimum + (maximum - minimum) * step / 200.0
            acc = self.predict_acc(speed_kmh, throttle, slope_deg, reverse)
            candidates.append((abs(acc - target_acc), throttle, acc))
        error, throttle, predicted = min(candidates)
        if coast_acc is None and direction * target_acc < min(direction * item[2] for item in candidates):
            info.update(source="unmeasured_coast", table_error=None,
                        reason="request_below_measured_positive_throttle_range")
            return 0.0, 0.0, info
        info.update(source="legacy_throttle", table_error=error)
        return throttle, 0.0, info


def load_accel_pedal_map(data_dir: Optional[str] = None, max_brake_mps2: float = 6.0,
                         low_speed_profile_path: Optional[str] = None) -> AccelPedalMap:
    return AccelPedalMap(data_dir=data_dir, max_brake_mps2=max_brake_mps2,
                         low_speed_profile_path=low_speed_profile_path)
