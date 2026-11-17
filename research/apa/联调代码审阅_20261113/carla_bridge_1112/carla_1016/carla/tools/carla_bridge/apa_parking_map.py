"""Local parking metadata and geometry; no CARLA RPC or simulator mutations."""

import bisect
from collections import defaultdict
import hashlib
import math
import xml.etree.ElementTree as ET

PARKING_MAP_PRESETS = (("Town04", "parking_slots.json", 29),
                       ("Town15", "parking_slots15.json", 6))


def parking_map_preset(map_name):
    name = str(map_name).rsplit("/", 1)[-1]
    for preset in PARKING_MAP_PRESETS:
        if preset[0] == name:
            return preset
    raise ValueError(f"地图 {name} 没有车位预设，请显式配置 apa.slot_file")


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non-finite OpenDRIVE geometry")
    return result


def polynomial(records, s):
    selected = next((row for row in reversed(records) if number(row.get("s")) <= s), None)
    if selected is None:
        return 0.0, 0.0
    ds = s - number(selected.get("s"))
    a, b, c, d = (number(selected.get(key, "0")) for key in ("a", "b", "c", "d"))
    return a + ds * (b + ds * (c + ds * d)), b + ds * (2 * c + ds * 3 * d)


def reference_pose(geometry, s):
    x, y, heading = (number(geometry.get(key)) for key in ("x", "y", "hdg"))
    ds = s - number(geometry.get("s"))
    length = number(geometry.get("length"))
    if ds < -1e-6 or ds > length + 1e-6:
        raise ValueError("parking object outside reference-line geometry")
    primitive = geometry[0]
    if primitive.tag == "line":
        return x + ds * math.cos(heading), y + ds * math.sin(heading), heading
    if primitive.tag == "arc":
        curvature = number(primitive.get("curvature"))
        if abs(curvature) < 1e-12:
            return x + ds * math.cos(heading), y + ds * math.sin(heading), heading
        end = heading + curvature * ds
        return (x + (math.sin(end) - math.sin(heading)) / curvature,
                y - (math.cos(end) - math.cos(heading)) / curvature, end)
    if primitive.tag != "spiral" or length <= 0:
        raise ValueError(f"unsupported parking reference line: {primitive.tag}")
    start = number(primitive.get("curvStart"))
    rate = (number(primitive.get("curvEnd")) - start) / length

    def angle(u):
        return heading + start * u + 0.5 * rate * u * u

    intervals = max(2, 2 * math.ceil(abs(ds) / 0.1))
    step = ds / intervals
    sx, sy = math.cos(angle(0)) + math.cos(angle(ds)), math.sin(angle(0)) + math.sin(angle(ds))
    for index in range(1, intervals):
        weight = 4 if index % 2 else 2
        theta = angle(index * step)
        sx += weight * math.cos(theta)
        sy += weight * math.sin(theta)
    return x + step * sx / 3, y + step * sy / 3, angle(ds)


def rotate(point, yaw, pitch=0.0, roll=0.0):
    x, y, z = point
    y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
    x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)
    return x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw), z


class OpenDriveParkingMap:
    """Read source-identified native rectangles; entrance and goal heading belong to task configuration."""

    def __init__(self, xml):
        self.sha256 = hashlib.sha256(xml.encode()).hexdigest()
        self.records = []
        self.by_source = {}
        self.index = defaultdict(list)
        self.rejected = []
        self.raw_count = 0
        root = ET.fromstring(xml)
        for road in root.findall("road"):
            geometries = sorted(road.findall("./planView/geometry"), key=lambda row: number(row.get("s")))
            starts = [number(row.get("s")) for row in geometries]
            elevations = sorted(road.findall("./elevationProfile/elevation"), key=lambda row: number(row.get("s")))
            banks = sorted(road.findall("./lateralProfile/superelevation"), key=lambda row: number(row.get("s")))
            for obj in road.findall("./objects/object"):
                if obj.get("type") != "parkingSpace":
                    continue
                self.raw_count += 1
                source = (road.get("id"), obj.get("id"))
                try:
                    s, t = number(obj.get("s")), number(obj.get("t"))
                    index = bisect.bisect_right(starts, s) - 1
                    if index < 0:
                        raise ValueError("missing reference line")
                    x, y, heading = reference_pose(geometries[index], s)
                    z, grade = polynomial(elevations, s)
                    bank, _ = polynomial(banks, s)
                    width, length = number(obj.get("width")), number(obj.get("length"))
                    if min(width, length) <= 0:
                        raise ValueError("empty parking rectangle")
                    offset = rotate((0, t, number(obj.get("zOffset", "0"))), heading, -math.atan(grade), bank)
                    centre = (x + offset[0], y + offset[1], z + offset[2])
                    yaw, pitch, roll = (number(obj.get(key, "0")) for key in ("hdg", "pitch", "roll"))
                    corners = []
                    for u, v in ((-length/2, -width/2), (length/2, -width/2),
                                 (length/2, width/2), (-length/2, width/2)):
                        local = rotate((u, v, 0), yaw, pitch, roll)
                        point = rotate(local, heading, -math.atan(grade), bank)
                        corners.append({"x": centre[0] + point[0], "y": -(centre[1] + point[1]),
                                        "z": centre[2] + point[2]})
                    record = {"road_id": source[0], "object_ids": [source[1]],
                              "x": centre[0], "y": -centre[1], "z": centre[2],
                              "width_m": width, "length_m": length,
                              "object_yaw_rad": math.atan2(math.sin(-heading-yaw), math.cos(-heading-yaw)),
                              "corners": corners, "markings": [dict(row.attrib) for row in obj.findall("./parkingSpace/marking")],
                              "rectangle_supported": not (obj.find("outline") is not None or
                                  obj.find("outlines") is not None or any(
                                      any(abs(number(row.get(key, "0"))) > 1e-10 for key in ("a", "b", "c", "d"))
                                      for row in road.findall("./lateralProfile/shape")+road.findall("./lateralProfile/crossfall")))}
                    self.records.append(record)
                    self.by_source[source] = record
                except (ValueError, TypeError, IndexError) as exc:
                    self.rejected.append({"road_id": source[0], "object_id": source[1], "reason": str(exc)})

        unique = {}
        for record in self.records:
            key = (record["road_id"], tuple(sorted(tuple(round(p[axis], 5) for axis in ("x", "y", "z"))
                                                   for p in record["corners"])))
            if key in unique:
                existing = unique[key]
                existing["object_ids"].extend(record["object_ids"])
                for marking in record["markings"]:
                    if marking not in existing["markings"]:
                        existing["markings"].append(marking)
                for object_id in record["object_ids"]:
                    self.by_source[(record["road_id"], object_id)] = existing
            else:
                unique[key] = record
        self.records = list(unique.values())
        for record in self.records:
            self.index[(math.floor(record["x"]/10), math.floor(record["y"]/10))].append(record)

    def match(self, slot, radius=0.75):
        explicit = slot.get("native_parking", {})
        if explicit:
            record = self.by_source.get((str(explicit["road_id"]), str(explicit["object_id"])))
            if record is None:
                raise ValueError("configured native parking object is absent from the loaded map")
            return record
        x = sum(number(slot[f"slot_pt{i}_x"]) for i in range(1, 5))/4
        y = sum(number(slot[f"slot_pt{i}_y"]) for i in range(1, 5))/4
        bx, by = math.floor(x/10), math.floor(y/10)
        nearby = [record for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                  for record in self.index.get((bx+dx, by+dy), ())
                  if math.hypot(record["x"]-x, record["y"]-y) <= radius]

        return nearby[0] if len(nearby) == 1 else None


def convex_polygons_overlap(first, second, epsilon=1e-7):
    """Positive-area convex intersection; touching edges are not occupancy."""
    if len(first) < 3 or len(second) < 3:
        return False
    for polygon in (first, second):
        for a, b in zip(polygon, polygon[1:]+polygon[:1]):
            nx, ny = b["y"]-a["y"], a["x"]-b["x"]
            length = math.hypot(nx, ny)
            if length <= epsilon:
                continue
            left = [p["x"]*nx+p["y"]*ny for p in first]
            right = [p["x"]*nx+p["y"]*ny for p in second]
            if min(max(left), max(right))-max(min(left), min(right)) <= epsilon*length:
                return False
    return bool(first and second)
