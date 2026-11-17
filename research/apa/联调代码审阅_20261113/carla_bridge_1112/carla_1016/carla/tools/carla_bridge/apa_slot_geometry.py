"""Validate shared surveyed vertices, edges and materialized slot coordinates."""
import math


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def validate_slot_geometry(data, opendrive_sha256=None):
    slots = data.get("slots", [])
    shared = [s for s in slots if s.get("geometry_source") == "surveyed_shared"
              or "slot_vertex_ids" in s or "slot_edge_ids" in s]
    geometry = data.get("slot_geometry")
    if geometry is None:
        if shared:
            raise ValueError("Shared slot references require slot_geometry")
        return
    if (not isinstance(geometry, dict) or geometry.get("version") != 1 or geometry.get("coordinate_frame") != "carla_world"
            or geometry.get("boundary_reference") != "paint_center"):
        raise ValueError("Unsupported shared slot geometry contract")
    revision = geometry.get("opendrive_sha256", "")
    if (not isinstance(revision, str) or len(revision) != 64
            or any(c not in "0123456789abcdef" for c in revision)):
        raise ValueError("Shared slot geometry requires its surveyed OpenDRIVE digest")
    if opendrive_sha256 is not None and revision != opendrive_sha256:
        raise ValueError("Shared slot survey does not match the loaded OpenDRIVE map")
    vertices, edges = geometry.get("vertices"), geometry.get("edges")
    if not isinstance(vertices, dict) or not isinstance(edges, dict):
        raise ValueError("Shared slot vertices/edges must be objects")
    for key, point in vertices.items():
        if not key or not isinstance(point, dict):
            raise ValueError("Invalid shared slot vertex")
        for axis in ("x", "y", "z"):
            _number(point.get(axis), f"slot_geometry.{key}.{axis}")
    for key, edge in edges.items():
        if not isinstance(edge, dict):
            raise ValueError("Shared slot edge must be an object")
        refs = edge.get("vertices")
        if (not key or not isinstance(refs, list) or len(refs) != 2
                or any(not isinstance(v, str) or v not in vertices for v in refs)
                or refs[0] == refs[1] or edge.get("role") not in ("divider", "entry", "back")):
            raise ValueError("Invalid shared slot edge")
        if edge["role"] == "divider" and _number(edge.get("paint_width_m"), "paint_width_m") <= 0:
            raise ValueError("Surveyed divider requires a positive recorded paint width")
    for slot in shared:
        if slot.get("geometry_source") != "surveyed_shared":
            raise ValueError("Shared references require geometry_source=surveyed_shared")
        refs, edge_refs = slot.get("slot_vertex_ids"), slot.get("slot_edge_ids")
        if (not isinstance(refs, list) or len(refs) != 4
                or any(not isinstance(v, str) or v not in vertices for v in refs)
                or len(set(refs)) != 4 or not isinstance(edge_refs, list) or len(edge_refs) != 4
                or any(not isinstance(e, str) or e not in edges for e in edge_refs)):
            raise ValueError("Each shared slot requires four valid vertex/edge references")
        entry = slot.get("slot_entry")
        if type(entry) is not int or not 1 <= entry <= 4:
            raise ValueError("Invalid shared slot entrance")
        points = [vertices[v] for v in refs]
        turns = []
        for i, p in enumerate(points):
            q, r = points[(i+1) % 4], points[(i+2) % 4]
            turns.append((q["x"]-p["x"])*(r["y"]-q["y"])-(q["y"]-p["y"])*(r["x"]-q["x"]))
            edge = edges[edge_refs[i]]
            if set(edge["vertices"]) != {refs[i], refs[(i+1) % 4]}:
                raise ValueError("Slot edge does not connect its referenced vertices")
            if (edge["role"] == "entry") != (i == entry-1):
                raise ValueError("Shared slot entrance disagrees with edge role")
            for axis in ("x", "y"):
                value = _number(slot.get(f"slot_pt{i+1}_{axis}"), "materialized slot corner")
                if abs(value-p[axis]) > 1e-8:
                    raise ValueError("Materialized slot corner differs from its shared vertex; rebuild catalog")
        if not (all(t > 1e-6 for t in turns) or all(t < -1e-6 for t in turns)):
            raise ValueError("Shared slot must be a strictly convex ordered quadrilateral")
        if slot.get("goal_reference_point", "slot_center") != "slot_center" or "apa_target_yaw_rad" in slot:
            raise ValueError("Surveyed shared slots derive their goal from slot geometry")
        for axis in ("x", "y"):
            centre = sum(p[axis] for p in points)/4
            entrance = (points[entry-1][axis]+points[entry % 4][axis])/2
            if (abs(_number(slot.get(f"goal_{axis}"), "slot goal")-centre) > 1e-8
                    or abs(_number(slot.get(f"slot_entry_pt_{axis}"), "slot entry")-entrance) > 1e-8):
                raise ValueError("Shared slot centre/entrance must be rebuilt with its corners")
