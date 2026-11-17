"""Forbidden surfaces and omissions certified by cell, height and source profile."""
import hashlib
import json
import math

from apa_map_interior import profile_signature
from apa_obstacle_tiles import StaticObstacleIndex
from apa_scene_compaction import _wire_convex

FORBIDDEN_REGIONS_VERSION = 2


def reference_digest(data):
    source = {k: v for k, v in data.items() if k != 'forbidden_regions'}
    return hashlib.sha256(json.dumps(source, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


class ForbiddenRegions:
    def __init__(self, data, slots_sha256=None):
        self.policy = data.get('forbidden_regions')
        self.bands, self.signatures, self.indices = {}, {}, {}
        self.full_cells = []
        self.reason = 'not_built'
        if self.policy is None:
            return
        p = self.policy
        if (not isinstance(p, dict) or p.get('version') != FORBIDDEN_REGIONS_VERSION or
                p.get('method') != 'opendrive_surface_complement' or
                p.get('surface_level_policy') != 'all_supported_levels_union' or
                p.get('original_geometry_preserved') is not True or
                p.get('reference_sha256') != reference_digest(data) or
                p.get('opendrive_sha256') != data.get('opendrive_sha256') or
                not isinstance(p.get('geometries'), list) or len(p['geometries']) > 512 or
                not isinstance(p.get('full_cells'), list) or len(p['full_cells']) != len(p['geometries']) or
                not isinstance(p.get('layers'), list) or len(p['layers']) > 512):
            raise ValueError('整体禁行区域的版本或来源校验失败')
        coverage = {tuple(c) for c in data['coverage_cells']}
        for cells in p['full_cells']:
            if (not isinstance(cells, list) or any(not isinstance(c, list) or len(c) != 2 or
                    any(type(v) is not int for v in c) or tuple(c) not in coverage for c in cells)):
                raise ValueError('整体禁行区域整格证据无效')
            self.full_cells.append(frozenset(map(tuple, cells)))
        for parts in p['geometries']:
            if not isinstance(parts, list) or len(parts) > 65536:
                raise ValueError('整体禁行区域凸片记录无效')
            for polygon in parts:
                if (not isinstance(polygon, list) or not 3 <= len(polygon) <= 8 or
                        any(not isinstance(v, list) or len(v) != 2 or any(
                            type(n) not in (int, float) or not math.isfinite(n) or abs(n) > 1e6
                            for n in v) for v in polygon) or not _wire_convex(polygon)):
                    raise ValueError('整体禁行区域须为有效的3至8顶点凸多边形')
        for index, layer in enumerate(p['layers']):
            g = layer.get('geometry')
            if (type(g) is not int or not 0 <= g < len(p['geometries']) or
                    not isinstance(layer.get('omit_cells'), dict) or
                    not isinstance(layer.get('height_bands'), list)):
                raise ValueError('整体禁行区域高度层无效')
            for cell, sources in layer['omit_cells'].items():
                try:
                    coords = [int(v) for v in cell.split(',')]
                except (ValueError, AttributeError):
                    raise ValueError('整体禁行区域索引格无效')
                if len(coords) != 2 or not isinstance(sources, dict):
                    raise ValueError('整体禁行区域索引格无效')
                for record in sources.values():
                    if (not isinstance(record, dict) or
                            not isinstance(record.get('profile_signatures'), list) or
                            not record['profile_signatures'] or
                            not all(isinstance(s, str) for s in record['profile_signatures']) or
                            not isinstance(record.get('indices'), list) or
                            any(type(i) is not int or i < 0 for i in record['indices'])):
                        raise ValueError('整体禁行区域省略证据无效')
            for band in layer['height_bands']:
                if (not isinstance(band, list) or len(band) != 2 or any(
                        type(n) not in (int, float) or not math.isfinite(n) for n in band) or
                        band[0] >= band[1] or tuple(band) in self.bands):
                    raise ValueError('整体禁行区域高度索引无效')
                self.bands[tuple(band)] = index
        if slots_sha256 is not None and slots_sha256 != p.get('slots_sha256'):
            raise ValueError('整体禁行区域与车位目录不匹配，需重新离线构建')
        self.reason = ''

    def select(self, low, high):
        if self.reason:
            return None, self.reason
        index = self.bands.get((low, high))
        return (index, '') if index is not None else (None, 'height_not_built')

    def omitted(self, record, entry):
        if not record:
            return ()
        key = id(entry)
        if key not in self.signatures:
            self.signatures[key] = profile_signature(entry)
        return (tuple(record['indices']) if self.signatures[key] in record['profile_signatures'] else ())

    def covered_cells(self, layer_index):
        return self.full_cells[self.policy['layers'][layer_index]['geometry']]

    def obstacles(self, layer_index, bounds, low, high, clip):
        geometry = self.policy['layers'][layer_index]['geometry']
        if geometry not in self.indices:
            rows = []
            for index, points in enumerate(self.policy['geometries'][geometry]):
                xs, ys = zip(*points)
                rows.append({'id': index, 'footprint': [{'x': x, 'y': y} for x, y in points],
                             '_xy_bounds': [min(xs), min(ys), max(xs), max(ys)]})
            self.indices[geometry] = StaticObstacleIndex(rows, 8.0)
        result = []
        revision = self.policy['reference_sha256'][:16]
        for row in sorted(self.indices[geometry].query(bounds), key=lambda r: r['id']):
            points = clip(row['footprint'], bounds, max_vertices=None)
            if len(points) < 3:
                continue
            parts = [points] if len(points) <= 8 else [
                [points[0]]+points[i:i+7] for i in range(1, len(points)-1, 6)]
            for index, polygon in enumerate(parts):
                result.append({'id': f'offline-map:forbidden:{revision}:{geometry}:{row["id"]}:{index}',
                               'footprint': polygon, 'z_min': low, 'z_max': high,
                               'velocity_x': 0.0, 'velocity_y': 0.0})
        return result
