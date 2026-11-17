"""Deterministic waypoint selection shared by CARLA vehicle controllers."""

from __future__ import annotations

import math
from typing import Any, Iterable, Optional, Tuple


def normalize_angle_rad(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad = angle_rad - 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad = angle_rad + 2.0 * math.pi
    return angle_rad


def waypoint_route_key(waypoint: Any, reference_yaw_deg: float) -> Tuple[float, int, int, int, float]:
    """Prefer straight continuation, then use map identifiers as stable tie-breaks."""
    reference_yaw_rad = math.radians(reference_yaw_deg)
    candidate_yaw_rad = math.radians(float(waypoint.transform.rotation.yaw))
    heading_error = abs(normalize_angle_rad(candidate_yaw_rad - reference_yaw_rad))
    return (
        heading_error,
        int(getattr(waypoint, "road_id", 0)),
        int(getattr(waypoint, "section_id", 0)),
        int(getattr(waypoint, "lane_id", 0)),
        float(getattr(waypoint, "s", 0.0)),
    )


def select_straight_waypoint(
    candidates: Iterable[Any], reference_yaw_deg: float
) -> Optional[Any]:
    """Select the deterministic, closest-to-straight continuation."""
    candidate_list = list(candidates)
    if not candidate_list:
        return None
    return min(
        candidate_list,
        key=lambda waypoint: waypoint_route_key(waypoint, reference_yaw_deg),
    )
