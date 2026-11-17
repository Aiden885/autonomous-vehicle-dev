"""Read-only static map queries with sparse XY and height coverage."""
from collections import OrderedDict, defaultdict, deque
import gzip
import hashlib
import json
import math

from apa_obstacle_tiles import StaticObstacleIndex
from apa_map_interior import InteriorFilter
from apa_forbidden_regions import ForbiddenRegions
from apa_scene_compaction import _hull, _intersection

SCHEMA_VERSION = 1
TILE_M = 8.0
MAX_BYTES = 128 * 1024 * 1024
PLANNING_GEOMETRY_VERSION = 1
MAX_PLANNING_OUTWARD_M = 0.02


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def body_geometry_digest(source):
    return digest((source['id'], source['_asset_name'], source['_vertices'],
                   sorted(source['_semantic_labels'])))


def validate_raw_sampling_profile(profile):
    """Require the raw-cell evidence written by the version-10 survey tool."""
    evidence = profile.get('sampling_geometry', {})
    rows = evidence.get('hit_cell_rows')
    if (evidence.get('version') != 10 or evidence.get('neighbor_padding_cells') != 0 or
            evidence.get('grid_m') != 0.1 or not isinstance(rows, list) or any(
                not isinstance(row, list) or len(row) != 3 or
                any(type(v) is not int for v in row) or row[1] >= row[2] for row in rows)):
        raise ValueError('新版采样轮廓缺少有效原始命中格证据，不能标记为去膨胀底图')
    if profile.get('footprints') and not rows:
        raise ValueError('非空采样轮廓缺少原始命中格')
    for previous, current in zip(rows, rows[1:]):
        if current[0] < previous[0] or (current[0] == previous[0] and current[1] <= previous[2]):
            raise ValueError('原始命中格须按行排序且合并相邻区间')


def inventory_digest(items):
    geometry = [[p['id'], p['_vertices'], sorted(p['_semantic_labels'])]
                for p in sorted(items, key=lambda row: row['id']) if p['_profile_eligible']]

    return hashlib.sha256(json.dumps(geometry).encode()).hexdigest()


def tile_cells(bounds):
    a, b, c, d = bounds
    return [(x, y) for x in range(math.floor(a/TILE_M), math.ceil(c/TILE_M))
            for y in range(math.floor(b/TILE_M), math.ceil(d/TILE_M))]


def intersects(a, b):
    return a[0] < b[2] and a[2] > b[0] and a[1] < b[3] and a[3] > b[1]


def cell_rectangles(cells):
    """Partition index cells into exact rectangles; never bridge a missing cell."""
    pending = set(cells)
    while pending:
        x, y = min(pending)
        right = x+1
        while (right, y) in pending:
            right += 1
        top = y+1
        while all((ix, top) in pending for ix in range(x, right)):
            top += 1
        pending.difference_update((ix, iy) for ix in range(x, right) for iy in range(y, top))
        yield x, y, right, top


def planning_geometry_policy(data):
    """Validate the optional offline-derived layer; never approximate at runtime."""
    policy = data.get('planning_geometry')
    if policy is None:
        if any('planning_footprints' in (p.get('profile') or {})
               for s in data['sources'] for p in s.get('profiles', [])):
            raise ValueError('简化轮廓缺少离线误差策略')
        return {}
    if not isinstance(policy, dict):
        raise ValueError('离线地图简化策略结构无效')
    tolerance = policy.get('max_outward_m')
    if (policy.get('version') != PLANNING_GEOMETRY_VERSION or
            policy.get('method') != 'bounded_convex_union' or
            policy.get('reference') != 'original_profile_footprints' or
            policy.get('preserve_components_and_holes') is not True or
            type(tolerance) not in (int, float) or not math.isfinite(tolerance) or
            not 0 <= tolerance <= MAX_PLANNING_OUTWARD_M):
        raise ValueError('离线地图简化策略无效或外扩误差超过 2 cm')
    return dict(policy)


def read_map(path, map_name, opendrive, geometry, server, sampling_version,
             *, compatible_sampling_versions=()):
    with gzip.open(path, 'rb') as source:
        raw = source.read(MAX_BYTES+1)
    if len(raw) > MAX_BYTES:
        raise ValueError('离线泊车底图超过容量上限')
    envelope = json.loads(raw)
    if not isinstance(envelope, dict) or not isinstance(envelope.get('map'), dict):
        raise ValueError('离线泊车底图结构无效')
    data = envelope['map']
    if envelope.get('schema') != SCHEMA_VERSION or envelope.get('sha256') != digest(data):
        raise ValueError('离线泊车底图版本或内容校验失败')
    expected = {'map_name': map_name.rsplit('/', 1)[-1], 'opendrive_sha256': opendrive,
                'geometry_sha256': geometry, 'carla_server': server}
    for key, value in expected.items():
        if data.get(key) != value:
            raise ValueError(f'离线泊车底图与当前场景不匹配：{key}，请重新离线构建')

    if data.get('sampling_version') not in (sampling_version, *compatible_sampling_versions):
        raise ValueError('离线泊车底图采样版本不匹配，请重新离线构建')
    if data.get('sampling_version') >= 10:
        for source in data['sources']:
            for entry in source.get('profiles', []):
                profile = entry.get('profile')
                if profile is None:
                    continue
                validate_raw_sampling_profile(profile)
    for source in data.get('body_sources', []):
        if source.get('_body_geometry_id') != body_geometry_digest(source):
            raise ValueError('静态车轮廓与包围盒身份不匹配')
        for entry in source.get('profiles', []):
            if entry.get('profile') is not None:
                validate_raw_sampling_profile(entry['profile'])
                hits = entry['profile']['sampling_geometry'].get('body_axis_hit_levels')
                if (not isinstance(hits, list) or len(hits) != 2 or
                        any(type(count) is not int or count < 2 for count in hits)):
                    raise ValueError('静态车轮廓缺少双轴车体命中证据')
    planning_geometry_policy(data)
    return data, envelope['sha256']


class OfflineParkingMap:
    def __init__(self, data, slice_obstacle, clip_polygon, needs_profile,
                 vehicle_profile_id=None, slots_sha256=None, vehicle_geometry=None):
        self.data = data
        self.planning_policy = planning_geometry_policy(data)
        self.interior = InteriorFilter(data, vehicle_profile_id, slots_sha256, vehicle_geometry)
        self.forbidden = ForbiddenRegions(data, slots_sha256)
        self.slice = slice_obstacle
        self.clip = clip_polygon
        self.needs_profile = needs_profile
        cells = data['coverage_cells']
        if (data.get('tile_size_m') != TILE_M or not isinstance(cells, list) or
                any(not isinstance(cell, list) or len(cell) != 2 or
                    any(type(v) is not int or abs(v)*TILE_M > 1e6 for v in cell) for cell in cells)):
            raise ValueError('离线泊车底图空间索引无效')
        self.cells = {tuple(cell) for cell in cells}
        if not self.cells or len(self.cells) > 65536:
            raise ValueError('离线泊车底图覆盖无效')
        self.sources = data['sources']
        self.body_sources = {source['id']: source for source in data.get('body_sources', [])}
        self.index = StaticObstacleIndex(self.sources, TILE_M)
        self.survey_bands = {tuple(row['cell']): tuple(sorted(map(tuple, row['height_bands'])))
                             for row in data['survey_layers']}
        if (set(self.survey_bands) != self.cells or any(
                not bands or any(len(band) != 2 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in band) or
                    band[0] >= band[1] for band in bands)
                for bands in self.survey_bands.values())):
            raise ValueError('离线地图采集格的高度带无效')
        self.query_bands = {}
        for band in {b for values in self.survey_bands.values() for b in values}:
            if self.forbidden.policy is None:
                self.query_bands[band] = (band,)
                continue

            certified = [b for b in self.forbidden.bands if band[0] <= b[0] < b[1] <= band[1]]
            selected, covered = [], band[0]
            while covered < band[1]:

                candidate = max((b for b in certified if b[0] <= covered < b[1]),
                                key=lambda b: b[1], default=None)
                if candidate is None:
                    end = min((b[0] for b in certified if b[0] > covered), default=band[1])
                    candidate = covered, end
                selected.append(candidate)
                covered = candidate[1]
            self.query_bands[band] = tuple(selected)
        self.cache = OrderedDict()
        self.body_cache = OrderedDict()

    def intersects_coverage(self, bounds):
        """Coarse selection only; uncovered space remains an explicit constraint."""
        return any(cell in self.cells for cell in tile_cells(bounds))

    def height_seed(self, origin, low, high):
        """Identify the sampled volumes containing the current vehicle height."""
        origin_cell = tuple(math.floor(v/TILE_M) for v in origin)
        seed = tuple(b for b in self.survey_bands.get(origin_cell, ())
                     if b[0] <= low and high <= b[1])
        return origin_cell, seed

    def connected_height_bands(self, bounds, origin, low, high):
        """Select connected sampled volumes conservatively; overlap does not certify road topology."""
        origin_cell, seed = self.height_seed(origin, low, high)
        key = ('height_bands', tuple(bounds), origin_cell, seed)
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        cells = set(tile_cells(bounds))
        pending = deque((origin_cell, band) for band in seed)
        reached = set(pending)
        while pending:
            (x, y), band = pending.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    cell = (x+dx, y+dy)
                    if cell not in cells:
                        continue
                    for other in self.survey_bands.get(cell, ()):
                        node = cell, other
                        if node not in reached and band[0] <= other[1] and other[0] <= band[1]:
                            reached.add(node)
                            pending.append(node)
        selected = defaultdict(list)
        for cell, band in sorted(reached):
            selected[cell].extend(self.query_bands[band])
        return self._remember(key, {cell: tuple(sorted(set(bands))) for cell, bands in selected.items()})

    def query_cells(self, bounds, low, high, cell_bands, *, planning_center=None):
        """Reuse certified per-band queries, joining equal-height index cells."""
        bands = sorted({band for values in cell_bands.values() for band in values})

        layers = tuple((band, self.interior.select(*band, planning_center)) for band in bands)
        key = ('cells', tuple(bounds), tuple((c, tuple(b)) for c, b in sorted(cell_bands.items())), layers)
        if key in self.cache:
            self.cache.move_to_end(key)
            parts, summary = self.cache[key]
            return parts, dict(summary, query_height_band=[low, high])
        groups = defaultdict(set)
        for cell in tile_cells(bounds):
            selected = cell_bands.get(cell, ())
            if selected:
                for band in selected:
                    groups[(band, '')].add(cell)
            else:
                groups[(None, 'height' if cell in self.cells else 'coverage')].add(cell)
        result, seen, missing = [], set(), set()
        outside = unknown_height = 0
        for (band, reason), cells in sorted(groups.items(), key=lambda row: (row[0][0] or (math.inf, math.inf), row[0][1])):
            for x, y, right, top in cell_rectangles(cells):
                rectangle = [max(x*TILE_M, bounds[0]), max(y*TILE_M, bounds[1]),
                             min(right*TILE_M, bounds[2]), min(top*TILE_M, bounds[3])]
                if band is None:
                    a, b, c, d = rectangle
                    parts = [{'id': f'offline-map:unknown:{reason}:{x},{y},{right},{top}',
                              'footprint': [{'x': px, 'y': py} for px, py in ((a,b),(c,b),(c,d),(a,d))],
                              'z_min': low, 'z_max': high, 'velocity_x': 0.0, 'velocity_y': 0.0}]
                    count = (right-x)*(top-y)
                    outside += count if reason == 'coverage' else 0
                    unknown_height += count if reason == 'height' else 0
                else:
                    parts, summary = self.query(rectangle, *band, planning_center=planning_center)
                    missing.update(summary['missing_source_ids'])
                for part in parts:
                    signature = (':unknown:' in part['id'], part['id'].startswith('offline-map:forbidden:'),
                                 tuple(sorted((p['x'], p['y']) for p in part['footprint'])))
                    if signature in seen:
                        continue
                    seen.add(signature)

                    identity = digest((part['id'], band, rectangle))[:20]
                    prefix = ':'.join(part['id'].split(':')[:3])
                    result.append(dict(part, id=f'{prefix}:height:{identity}'))
        summary = {'missing_sources': len(missing), 'missing_source_ids': sorted(missing),
                   'outside_cells': outside, 'unknown_height_cells': unknown_height,
                   'source_pieces': len(result), 'height_selection': 'connected_survey_volumes',
                   'query_height_band': [low, high],
                   'query_cell_height_bands': [[*cell, *band] for cell, values in sorted(cell_bands.items()) for band in values]}
        if self.planning_policy:
            summary['planning_geometry'] = self.planning_policy
        self._remember(key, (result, summary))
        return result, summary

    def _remember(self, key, value, *, cache=None):
        cache = self.cache if cache is None else cache
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > 80:
            cache.popitem(last=False)
        return value

    def query_body(self, obstacle, bounds, cell_bands):
        """Slice stationary bodies per sampled cell; measured moving sweeps stay conservative."""
        cells = tuple((cell, band) for cell in tile_cells(bounds) for band in cell_bands.get(cell, ()))
        if not cells:
            return []
        if math.hypot(obstacle['velocity_x'], obstacle['velocity_y']) > 1.e-4:
            return self.slice(obstacle, min(b[0] for _, b in cells), max(b[1] for _, b in cells))
        key = (obstacle['id'], obstacle.get('_body_geometry_id'),
               tuple(map(tuple, obstacle.get('_vertices', ()))),
               tuple((p['x'], p['y']) for p in obstacle['footprint']),
               obstacle['z_min'], obstacle['z_max'], obstacle['velocity_x'], obstacle['velocity_y'],
               tuple(bounds), cells)
        cached = self.body_cache.get(key)
        if cached is not None:
            self.body_cache.move_to_end(key)
            return cached
        record = self.body_sources.get(obstacle['id'])
        if record and record['_body_geometry_id'] != obstacle.get('_body_geometry_id'):
            record = None
        slices, result = {}, []
        for (x, y), band in cells:
            rectangle = [max(bounds[0], x*TILE_M), max(bounds[1], y*TILE_M),
                         min(bounds[2], (x+1)*TILE_M), min(bounds[3], (y+1)*TILE_M)]
            tile_key = f'body:{x}:{y}:{digest(band)[:10]}'
            entry = self._profile(record, rectangle, *band) if record else None
            if entry is not None:
                envelopes = None
                if (entry.get('profile') or {}).get('occlusions'):
                    if band not in slices:
                        slices[band] = self.slice(obstacle, *band)
                    envelopes = [part['footprint'] for part in slices[band]]
                result.extend(self._parts(record, rectangle, *band, tile_key, set(),
                                          occlusion_envelopes=envelopes))
                continue
            if band not in slices:
                slices[band] = self.slice(obstacle, *band)
            for part in slices[band]:
                polygon = self.clip(part['footprint'], rectangle, max_vertices=None)
                if len(polygon) < 3:
                    continue
                fans = [polygon] if len(polygon) <= 8 else [
                    [polygon[0]]+polygon[i:i+7] for i in range(1, len(polygon)-1, 6)]
                for i, points in enumerate(fans):
                    result.append(dict(part, id=f'{obstacle["id"]}:{tile_key}:{i}', footprint=points))
        return self._remember(key, result, cache=self.body_cache)

    def _profile(self, source, bounds, low, high):
        profiles = source.get('profiles', [])
        matches = [p for p in profiles if p['z_min'] <= low and p['z_max'] >= high and
                   (p.get('xy_bounds') is None or
                    all((p['xy_bounds'][i] <= bounds[i] if i < 2 else p['xy_bounds'][i] >= bounds[i]) for i in range(4)))]

        matches.sort(key=lambda p: (p.get('profile') is None,
            bool((p.get('profile') or {}).get('occlusions')), p['z_max']-p['z_min'],
            float('inf') if p.get('xy_bounds') is None else
            (p['xy_bounds'][2]-p['xy_bounds'][0])*(p['xy_bounds'][3]-p['xy_bounds'][1])))
        return matches[0] if matches else None

    def _parts(self, source, bounds, low, high, tile_key, missing, omitted=(),
               occlusion_envelopes=None):
        if source['z_min'] > high or source['z_max'] < low:
            return []
        entry = self._profile(source, bounds, low, high)
        unknown = False
        uncovered = False
        unknown_index_offset = 0
        if entry is not None:
            profile = entry.get('profile')
            if profile is not None:

                footprints = profile.get('planning_footprints', profile['footprints'])
                unknown_index_offset = len(profile['footprints'])-len(footprints)
                polygons = [(p, False) for p in footprints]
                polygons += [(p, True) for p in profile.get('occlusions', [])]
            else:
                unknown = entry.get('reason') != 'box_already_conservative'
                polygons = [(p['footprint'], unknown) for p in self.slice(source, low, high)]
        elif self.needs_profile(source, high):
            uncovered = True
            polygons = [(p['footprint'], True) for p in self.slice(source, low, high)]
        else:
            polygons = [(p['footprint'], False) for p in self.slice(source, low, high)]
        output = []
        omitted = set(omitted)
        for i, (polygon, unknown) in enumerate(polygons):
            if not unknown and i in omitted:
                continue
            piece_index = i+unknown_index_offset if unknown else i
            clipped = self.clip(polygon, bounds, max_vertices=None)
            pieces = [clipped]
            if unknown and occlusion_envelopes is not None:
                points = [(p['x'], p['y']) for p in clipped]
                pieces = [[{'x': x, 'y': y} for x, y in _hull(_intersection(
                    points, [(p['x'], p['y']) for p in envelope]))]
                    for envelope in occlusion_envelopes]
            fan_index = 0
            for clipped in pieces:
                if len(clipped) < 3:
                    continue
                fans = [clipped] if len(clipped) <= 8 else [
                    [clipped[0]]+clipped[start:start+7] for start in range(1,len(clipped)-1,6)]
                for part in fans:
                    output.append({'id':f'{source["id"]}:{"unknown" if unknown else "base"}:{tile_key}:{piece_index}:{fan_index}',
                        'footprint':part, 'z_min':low, 'z_max':high, 'velocity_x':0.0, 'velocity_y':0.0})
                    fan_index += 1
        if output and uncovered:
            missing.add(source['id'])
        return output

    def query(self, bounds, low, high, *, planning_center=None):
        """Height-select cached static profiles; uncovered cells remain unknown."""
        if not all(math.isfinite(v) for v in list(bounds)+[low,high]) or low >= high:
            raise ValueError('离线地图查询范围无效')
        if not (0 < bounds[2]-bounds[0] <= 128 and 0 < bounds[3]-bounds[1] <= 128):
            raise ValueError('离线地图查询范围过大或为空')

        low, high = math.floor(low*4)/4, math.ceil(high*4)/4
        interior_index, interior_reason = self.interior.select(low, high, planning_center)
        interior_layer = (self.interior.policy['layers'][interior_index]
                          if interior_index is not None else None)
        omit_cells = interior_layer['omit_cells'] if interior_layer else {}
        region_index, region_reason = self.forbidden.select(low, high)
        if self.forbidden.policy is not None and region_index is None:

            a, b, c, d = bounds
            return [{'id': 'offline-map:unknown:forbidden-region-height',
                     'footprint': [{'x': x, 'y': y} for x, y in ((a,b),(c,b),(c,d),(a,d))],
                     'z_min': low, 'z_max': high, 'velocity_x': 0.0, 'velocity_y': 0.0}], {
                         'unavailable': True, 'missing_sources': 0, 'missing_source_ids': [],
                         'outside_cells': 0, 'source_pieces': 1, 'query_height_band': [low, high],
                         'forbidden_regions': {'active': False, 'fallback': region_reason,
                             'layer': None, 'filled_pieces': 1, 'omitted_profile_parts': 0,
                             'fully_forbidden_cells_skipped': 0,
                             'method': 'opendrive_surface_complement'}}
        region_layer = (self.forbidden.policy['layers'][region_index]
                        if region_index is not None else None)
        region_omit = region_layer['omit_cells'] if region_layer else {}
        forbidden_cells = self.forbidden.covered_cells(region_index) if region_layer else ()
        key = tuple(bounds)+(low, high, interior_index, interior_reason, region_index, region_reason)
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        result, missing, outside = [], set(), 0
        outside_cells = []
        known_groups = defaultdict(list)
        known_cell_pieces = 0
        interior_removed = 0
        region_omitted = 0
        forbidden_cells_skipped = 0
        for ix, iy in tile_cells(bounds):
            cell = [max(ix*TILE_M,bounds[0]),max(iy*TILE_M,bounds[1]),
                    min((ix+1)*TILE_M,bounds[2]),min((iy+1)*TILE_M,bounds[3])]
            if (ix,iy) not in self.cells:
                outside_cells.append((ix,iy))
                outside += 1
                continue
            if (ix, iy) in forbidden_cells:

                forbidden_cells_skipped += 1
                continue
            expanded=[cell[0]-.4,cell[1]-.4,cell[2]+.4,cell[3]+.4]
            for source in self.index.query(expanded):
                if intersects(source['_xy_bounds'], expanded):
                    entry = self._profile(source, cell, low, high)
                    excluded = self.forbidden.omitted(region_omit.get(f'{ix},{iy}', {}).get(source['id']), entry)
                    region_omitted += len(excluded)
                    parts = self._parts(source,cell,low,high,f'{ix},{iy}',missing, excluded)
                    record = omit_cells.get(f'{ix},{iy}', {}).get(source['id'])
                    omitted = self.interior.omitted_indices(record, entry)
                    retained = self.interior.retain(parts, omitted)
                    interior_removed += len(parts)-len(retained)
                    parts = retained
                    omitted = tuple(sorted(set(omitted).union(excluded)))
                    if not self.planning_policy:
                        result.extend(parts)
                        continue

                    result.extend(p for p in parts if ':unknown:' in p['id'])
                    known = sum(':unknown:' not in p['id'] for p in parts)
                    if known:
                        known_groups[(source['id'], id(entry), omitted)].append((ix, iy, source, entry))
                        known_cell_pieces += known

        for x,y,right,top in cell_rectangles(outside_cells):
            a,b,c,d=max(x*8.,bounds[0]),max(y*8.,bounds[1]),min(right*8.,bounds[2]),min(top*8.,bounds[3])
            result.append({'id':f'offline-map:unknown:coverage:joined:{x},{y},{right},{top}',
                'footprint':[{'x':px,'y':py} for px,py in [(a,b),(c,b),(c,d),(a,d)]],
                'z_min':low,'z_max':high,'velocity_x':0.0,'velocity_y':0.0})
        joined_pieces = 0
        for (_, _, omitted), rows in known_groups.items():
            source, selected = rows[0][2:]
            for x, y, right, top in cell_rectangles((r[0], r[1]) for r in rows):
                rectangle = [max(x*TILE_M, bounds[0]), max(y*TILE_M, bounds[1]),
                             min(right*TILE_M, bounds[2]), min(top*TILE_M, bounds[3])]
                if self._profile(source, rectangle, low, high) is not selected:
                    raise ValueError('合并索引格后的离线高度/覆盖记录不一致')
                parts = self._parts(source, rectangle, low, high,
                                    f'joined:{x},{y},{right},{top}', missing, omitted)
                known = [p for p in parts if ':unknown:' not in p['id']]
                result.extend(known)
                joined_pieces += len(known)
        region_parts = (self.forbidden.obstacles(region_index, bounds, low, high, self.clip)
                        if region_index is not None else [])
        result.extend(region_parts)
        summary = {'missing_sources':len(missing),'missing_source_ids':sorted(missing),'outside_cells':outside,
                   'source_pieces':len(result),'query_height_band':[low,high]}
        if self.planning_policy:
            summary['planning_geometry'] = self.planning_policy
            summary['index_seam_pieces_removed'] = known_cell_pieces-joined_pieces
        summary['interior_filter'] = {
            'active': interior_layer is not None, 'fallback': interior_reason,
            'layer': interior_index, 'removed_cell_pieces': interior_removed,
            'scope': 'shared_parking_area_offline',
        }
        summary['forbidden_regions'] = {
            'active': region_layer is not None, 'fallback': region_reason,
            'layer': region_index, 'filled_pieces': len(region_parts),
            'omitted_profile_parts': region_omitted,
            'fully_forbidden_cells_skipped': forbidden_cells_skipped,
            'method': 'opendrive_surface_complement',
        }
        value = result, summary
        return self._remember(key, value)


def parking_region_guards(bounds, low, high):
    """Explicit unknown perimeter: outside a prepared task map is not free space."""
    a, b, c, d = bounds
    width = 0.5
    boxes = [(a-width, b-width, a, d+width), (c, b-width, c+width, d+width),
             (a, b-width, c, b), (a, d, c, d+width)]
    return [{"id": f"parking-region:unknown:{i}", "velocity_x": 0.0, "velocity_y": 0.0,
             "z_min": low, "z_max": high,
             "footprint": [{"x": x, "y": y} for x, y in ((l, t), (r, t), (r, u), (l, u))]}
            for i, (l, t, r, u) in enumerate(boxes)]
