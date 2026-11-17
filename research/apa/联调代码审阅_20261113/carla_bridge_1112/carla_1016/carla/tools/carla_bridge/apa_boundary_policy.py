"""Map-line semantics, task frame constraints and sampling-blind-region policy."""
import hashlib
import json
import math

LINE_GROUPS = ("obs_lines", "obs_slot_lines", "obs_vlines")
LINE_KINDS = ("physical_boundary", "parking_marking", "task_forbidden", "unclassified_constraint")


def offline_collision_parts(parts):
    """Sampling shadows are diagnostic; coverage and task guards remain hard."""
    hard, blind = [], []
    for part in parts:
        identity = str(part.get("id", ""))
        guard = identity.startswith(("offline-map:", "parking-region:"))
        unknown = part.get("kind") == "unknown" or ":unknown:" in identity
        (blind if unknown and not guard else hard).append(part)
    return hard, {"blind_region_policy": "diagnostic_only",
                  "blind_region_pieces": len(blind),
                  "blind_region_ids": sorted({str(p.get("id", "")) for p in blind})}


def line_intersects_bounds(first, second, bounds):
    """Closed segment/rectangle intersection; selected endpoints remain unchanged."""
    lower, upper = 0.0, 1.0
    for axis in (0, 1):
        delta = second[axis] - first[axis]
        if delta == 0.0:
            if not bounds[axis] <= first[axis] <= bounds[axis + 2]:
                return False
            continue
        a = (bounds[axis] - first[axis]) / delta
        b = (bounds[axis + 2] - first[axis]) / delta
        lower, upper = max(lower, min(a, b)), min(upper, max(a, b))
        if lower > upper:
            return False
    return True


def line_key(line):
    return tuple(sorted(((line['x1'], line['y1']), (line['x2'], line['y2']))))


def classified_map_lines(obstacles):
    if not isinstance(obstacles, dict):
        raise ValueError("obstacles 必须是对象")
    defaults = obstacles.get("line_defaults", {})
    if not isinstance(defaults, dict) or set(defaults) - set(LINE_GROUPS):
        raise ValueError("obstacles.line_defaults 只能配置 obs_lines/obs_slot_lines/obs_vlines")
    result = []
    for group in LINE_GROUPS:
        rows = obstacles.get(group, [])
        default = defaults.get(group, {})
        if not isinstance(rows, list):
            raise ValueError(f"{group} 必须是数组")
        if (not isinstance(default, dict) or set(default) - {"kind", "source"}
                or (default and (default.get("kind") not in LINE_KINDS
                    or not isinstance(default.get("source"), str) or not default["source"].strip()))):
            raise ValueError(f"{group}.line_defaults 须提供有效 kind 和 source")
        for index, row in enumerate(rows):
            label = f"{group}[{index}]"
            if not isinstance(row, dict):
                raise ValueError(f"{label} 必须是对象")
            line = {}
            for axis in ("x1", "y1", "x2", "y2"):
                value = row.get(axis)
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(f"{label}.{axis} 必须是有限数值")
                line[axis] = value
            if math.hypot(line["x2"]-line["x1"], line["y2"]-line["y1"]) <= 1e-9:
                raise ValueError(f"{label} 不能是零长度线")

            kind = row.get("kind", default.get("kind", "unclassified_constraint"))
            if not isinstance(kind, str) or kind not in LINE_KINDS:
                raise ValueError(f"{label}.kind 无效")
            source = row.get("source", default.get("source", "legacy_unspecified"))
            if not isinstance(source, str) or not source.strip():
                raise ValueError(f"{label}.source 必须是非空来源")
            if kind != "unclassified_constraint" and source == "legacy_unspecified":
                raise ValueError(f"{label} 分类须提供来源，不能按数组名称推断")
            provenance = {"source": group, "source_index": index, "kind": kind, "provenance": source}
            for key in ("source_id", "source_name"):
                if key in row:
                    if not isinstance(row[key], str) or not row[key].strip():
                        raise ValueError(f"{label}.{key} 必须是非空字符串")
                    provenance[key] = row[key]
            result.append((line, provenance))
    return result


def build_boundary_inputs(obstacles, slot_frames, bounds=None):
    """Pure task geometry: no ego pose, CARLA calls or occupancy-dependent rules."""
    mapped = classified_map_lines(obstacles)
    boundaries, virtual = [], []
    policy = {
        "version": 6,
        "neighbor_slot_access": "task_frozen_confirmed_free",
        "marking_collision_policy": "target_frame_and_nonfree_neighbors",
        "target_frame_policy": "non_entry_edges",
        "map_line_revision": hashlib.sha256(json.dumps(mapped, sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
        "boundary_sources": [], "virtual_sources": [],
        "selection_bounds": list(bounds) if bounds is not None else None,
        "duplicate_map_lines_removed": 0,
        "non_collision_map_lines": [],
    }
    indices = {"boundary": {}, "virtual": {}}
    for line, source in mapped:
        if bounds is not None and not line_intersects_bounds(
                (line['x1'], line['y1']), (line['x2'], line['y2']), bounds):
            continue
        kind = source["kind"]
        if kind == "parking_marking" or (kind == "unclassified_constraint" and
                source["provenance"] == "legacy_calibrated_map_lines_unverified"):
            policy["non_collision_map_lines"].append(dict(source, line=dict(line),
                reason="paint" if kind == "parking_marking" else "unverified_legacy_calibration"))
            continue
        if kind == "task_forbidden":
            channel = "virtual"
        else:
            channel = ("virtual" if kind == "unclassified_constraint" and source["source"] == "obs_vlines"
                       else "boundary")
        target = virtual if channel == "virtual" else boundaries
        key = line_key(line)
        index = indices[channel].get(key)
        if index is None:
            index = len(target)
            indices[channel][key] = index
            target.append(line)
        else:
            policy["duplicate_map_lines_removed"] += 1
        policy[channel + "_sources"].append(dict(source, start=index, count=1))
    shared = {}
    policy["shared_slot_edges_deduplicated"] = 0
    policy["slot_frame_revision"] = hashlib.sha256(json.dumps(slot_frames, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    for line in slot_frames:
        edge_id = line.get("shared_edge_id")
        slot_edge = {"slot_id": line["slot_id"], "edge": line["edge"]}
        if edge_id and edge_id in shared:
            index, source = shared[edge_id]
            first = virtual[index]
            coordinates = lambda item: {(item["x1"], item["y1"]), (item["x2"], item["y2"])}
            if coordinates(first) != coordinates(line):
                raise ValueError("Shared parking edge has inconsistent endpoints")
            if line["is_target"]:
                source["slot_edges"].insert(0, slot_edge)
                source["provenance"] = "target_slot_geometry"
                first.update(is_target=True, slot_id=line["slot_id"], edge=line["edge"])
            else:
                source["slot_edges"].append(slot_edge)
            policy["shared_slot_edges_deduplicated"] += 1
            continue
        source = {"source": "derived_slot_frames", "kind": "task_forbidden",
            "provenance": "target_slot_geometry" if line["is_target"] else "neighbor_slot_geometry",
            "start": len(virtual), "count": 1,
            "slot_edges": [slot_edge]}
        if edge_id:
            source["shared_edge_id"] = edge_id
            shared[edge_id] = (len(virtual), source)
        policy["virtual_sources"].append(source)
        virtual.append(dict(line))

    for name, lines in (("obs_lines", boundaries), ("obs_vlines", virtual)):
        if len(lines) > 64:
            raise ValueError(f"{name} 语义转换后超过 64 条，不能截断约束")
    return boundaries, virtual, policy
