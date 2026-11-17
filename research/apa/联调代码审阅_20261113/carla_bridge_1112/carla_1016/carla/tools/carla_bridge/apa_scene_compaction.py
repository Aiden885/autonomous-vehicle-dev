"""Exact convex-piece compaction; gaps, unknown coverage and actors stay separate."""
from collections import defaultdict, deque
import hashlib
import math


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _hull(points):
    points = sorted(set(points))
    def chain(values):
        out = []
        for p in values:

            while len(out) > 1 and _cross(out[-2], out[-1], p) <= 1e-8:
                out.pop()
            out.append(p)
        return out
    return chain(points)[:-1]+chain(reversed(points))[:-1]


def _wire_convex(points):
    return 3 <= len(points) <= 8 and all(
        _cross(a, points[(i+1) % len(points)], c) >= 1e-8
        for i, a in enumerate(points) for j, c in enumerate(points)
        if j not in (i, (i+1) % len(points)))


def _area(points):
    if len(points) < 3:
        return 0.0

    return abs(sum(_cross(points[0], a, b)
                   for a, b in zip(points[1:], points[2:])))/2


def _intersection(subject, clip):
    for a, b in zip(clip, clip[1:]+clip[:1]):
        output = []
        for p, q in zip(subject, subject[1:]+subject[:1]):
            dp, dq = _cross(a, b, p), _cross(a, b, q)
            if dp >= 0:
                output.append(p)
            if (dp < 0 < dq) or (dq < 0 < dp):
                t = dp/(dp-dq)
                output.append((p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1])))
        subject = output
        if not subject:
            break
    return subject


def compact_static_scene(obstacles):
    """Compact equivalent convex pieces to at most 8 vertices, preserving union area within 1e-10 m²."""
    polygons, buckets, memberships, large = {}, defaultdict(set), {}, set()
    passthrough = []
    source_count = len(obstacles)
    for part in obstacles:
        if not part['id'].startswith(('map:', 'occupancy:map:')) or part['velocity_x'] or part['velocity_y']:
            passthrough.append(part)
            continue
        points = _hull((p['x'], p['y']) for p in part['footprint'])
        polygons[len(polygons)] = (points, [part], ':unknown:' in part['id'])

    def cells(points):
        xs, ys = zip(*points)
        left, right = math.floor(min(xs)/2), math.floor(max(xs)/2)
        bottom, top = math.floor(min(ys)/2), math.floor(max(ys)/2)
        if (right-left+1)*(top-bottom+1) > 4096:
            return None
        return [(x, y) for x in range(left, right+1) for y in range(bottom, top+1)]

    def insert(index):
        keys = cells(polygons[index][0])
        memberships[index] = keys
        if keys is None:
            large.add(index)
        else:
            for key in keys:
                buckets[key].add(index)

    def remove(index):
        keys = memberships.pop(index)
        if keys is None:
            large.remove(index)
        else:
            for key in keys:
                buckets[key].remove(index)
                if not buckets[key]:
                    del buckets[key]
        del polygons[index]

    for index in polygons:
        insert(index)
    pending = deque(polygons)
    serial = len(polygons)
    comparisons = 0
    while pending:
        index = pending.popleft()
        if index not in polygons:
            continue
        points, sources, unknown = polygons[index]
        keys = memberships[index]
        neighbours = set(polygons) if keys is None else set(large)
        if keys is not None:
            for key in keys:
                neighbours.update(buckets.get(key, ()))
        for neighbour in sorted(neighbours):
            if neighbour == index:
                continue
            other, other_sources, other_unknown = polygons[neighbour]
            if other_unknown != unknown:
                continue
            if sources[0]['id'].startswith('occupancy:map:') != other_sources[0]['id'].startswith('occupancy:map:'):
                continue
            comparisons += 1
            hull = _hull(points+other)
            if not _wire_convex(hull):
                continue
            union_area = _area(points)+_area(other)-_area(_intersection(points, other))
            if abs(_area(hull)-union_area) > 1e-10:
                continue
            remove(index)
            remove(neighbour)
            polygons[serial] = (hull, sources+other_sources, unknown)
            insert(serial)
            pending.append(serial)
            serial += 1
            break

    result, provenance = list(passthrough), {}
    for points, sources, unknown in polygons.values():
        if len(sources) == 1:
            result.append(sources[0])
            continue
        ids = sorted(part['id'] for part in sources)
        digest = hashlib.sha256(repr((ids, points)).encode()).hexdigest()[:20]
        prefix = 'occupancy:' if sources[0]['id'].startswith('occupancy:map:') else ''
        identifier = f'{prefix}map:compact:{"unknown:" if unknown else ""}{digest}'
        result.append(dict(sources[0], id=identifier,
            footprint=[{'x': x, 'y': y} for x, y in points],
            z_min=min(p['z_min'] for p in sources), z_max=max(p['z_max'] for p in sources)))
        provenance[identifier] = ids
    result.sort(key=lambda part: part['id'])
    return result, {'input_pieces': source_count, 'output_pieces': len(result),
                    'known_pieces': sum(':unknown:' not in p['id'] for p in result),
                    'unknown_pieces': sum(':unknown:' in p['id'] for p in result),
                    'pair_comparisons': comparisons, 'sources': provenance}


class StaticSceneCompactor:
    """Reuse the derived layer while the actual input geometry is unchanged."""
    def __init__(self):
        self.signature = None
        self.result = None
        self.summary = {}

    def compact(self, obstacles):
        static = [p for p in obstacles if p['id'].startswith(('map:', 'occupancy:map:'))]
        signature = tuple((p['id'], p['velocity_x'], p['velocity_y'], p['z_min'], p['z_max'],
                           tuple((v['x'], v['y']) for v in p['footprint'])) for p in static)
        if signature != self.signature:
            self.result, self.summary = compact_static_scene(static)
            self.signature = signature
        others = [p for p in obstacles if not p['id'].startswith(('map:', 'occupancy:map:'))]
        result = self.result+others
        summary = {k: v for k, v in self.summary.items() if k != 'sources'}
        summary.update(input_pieces=len(obstacles), output_pieces=len(result),
                       known_pieces=sum(':unknown:' not in p['id'] for p in result),
                       unknown_pieces=sum(':unknown:' in p['id'] for p in result))
        return result, summary
