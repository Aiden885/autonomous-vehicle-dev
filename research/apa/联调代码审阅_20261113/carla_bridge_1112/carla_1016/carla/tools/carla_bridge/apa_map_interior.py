"""Height-indexed offline interior filters with full-map fallback."""
import math
import hashlib
import json


def vehicle_geometry_key(vehicle):
    """Only dimensions used by the offline component proof belong in its key."""
    fields = ('width_m', 'rear_to_front_m', 'rear_to_back_m')
    if not vehicle or any(type(vehicle.get(k)) not in (int, float) or
                          not math.isfinite(vehicle[k]) or vehicle[k] <= 0 for k in fields):
        raise ValueError('内部筛选需要有效车身尺寸')
    return {k: float(vehicle[k]) for k in fields}


def profile_signature(entry):
    profile = (entry or {}).get('profile')
    if profile is None:
        return 'height_slice'
    points = profile.get('planning_footprints', profile['footprints'])
    return hashlib.sha256(json.dumps(points, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def inside_ring(point, ring):
    x, y = point
    inside = False
    for a, b in zip(ring, ring[1:]+ring[:1]):
        if (a[1] > y) != (b[1] > y):
            if x < a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]):
                inside = not inside
    return inside


def inside_regions(point, regions):
    return any(inside_ring(point, rings[0]) and
               not any(inside_ring(point, ring) for ring in rings[1:]) for rings in regions)


class InteriorFilter:
    def __init__(self, data, vehicle_profile_id=None, slots_sha256=None, vehicle_geometry=None):
        self.policy = data.get('interior_filter')
        self.bands = {}
        self.profile_signatures = {}
        self.reason = 'not_built'
        if self.policy is None:
            return
        p = self.policy
        if (not isinstance(p, dict) or p.get('version') not in (1, 2) or
                p.get('method') != 'offline_static_component_interior' or
                p.get('preserve_unknown') is not True or
                not isinstance(p.get('layers'), list) or len(p['layers']) > 512):
            raise ValueError('离线泊车区域内部筛选策略无效')
        for index, layer in enumerate(p['layers']):
            if (not isinstance(layer.get('omit_cells'), dict) or
                    not isinstance(layer.get('centers'), list)):
                raise ValueError('离线泊车区域内部筛选记录无效')
            for rings in layer['centers']:
                if not rings or any(len(ring) < 3 or any(len(v) != 2 or
                        any(type(n) not in (int, float) or not math.isfinite(n) for n in v)
                        for v in ring) for ring in rings):
                    raise ValueError('离线泊车区域连通边界无效')
            for cell, sources in layer['omit_cells'].items():
                if not isinstance(cell, str) or not isinstance(sources, dict) or any(
                        not isinstance(record, dict) or
                        not isinstance(record.get('profile_signatures'), list) or
                        not record['profile_signatures'] or
                        not all(isinstance(s, str) for s in record['profile_signatures']) or
                        not isinstance(record.get('indices'), list) or
                        any(type(i) is not int or i < 0 for i in record['indices'])
                        for record in sources.values()):
                    raise ValueError('离线泊车区域筛选索引无效')
            for band in layer['height_bands']:
                if (len(band) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) for v in band)
                        or band[0] >= band[1] or tuple(band) in self.bands):
                    raise ValueError('离线泊车区域高度索引无效')
                self.bands[tuple(band)] = index
        if p['version'] == 2 and (vehicle_geometry is None or
                vehicle_geometry_key(vehicle_geometry) != p.get('vehicle_geometry')):
            self.reason = 'vehicle_geometry_mismatch'
        elif p['version'] == 1 and vehicle_profile_id != p.get('vehicle_profile_id'):
            self.reason = 'vehicle_profile_mismatch'
        elif slots_sha256 != p.get('slots_sha256'):
            self.reason = 'parking_catalog_mismatch'
        else:
            self.reason = ''

    def select(self, low, high, center):
        if center is None:
            return None, 'full_geometry_query'
        if self.reason:
            return None, self.reason
        index = self.bands.get((low, high))
        if index is None:
            return None, 'height_not_built'
        layer = self.policy['layers'][index]
        if (len(center) != 2 or not all(math.isfinite(v) for v in center) or
                not inside_regions(center, layer['centers'])):
            return None, 'outside_certified_component'
        return index, ''

    def omitted_indices(self, record, entry):
        if not record:
            return ()
        key = id(entry)
        if key not in self.profile_signatures:
            self.profile_signatures[key] = profile_signature(entry)
        if self.profile_signatures[key] not in record['profile_signatures']:
            return ()
        return tuple(record['indices'])

    @staticmethod
    def retain(parts, omitted):
        if not omitted:
            return parts
        omitted = set(omitted)

        return [p for p in parts if ':unknown:' in p['id'] or
                int(p['id'].rsplit(':', 2)[-2]) not in omitted]
