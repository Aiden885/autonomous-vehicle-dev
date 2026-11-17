"""Display-only parking markings matched to the active OpenDRIVE hash."""
from functools import lru_cache
import gzip
import json
import math
from pathlib import Path
import re


def _corners(points, native=False):
    minimum, maximum = (3, 256) if native else (4, 4)
    if not isinstance(points, list) or not minimum <= len(points) <= maximum:
        raise ValueError('invalid parking outline')
    result = []
    for point in points:
        if not isinstance(point, dict):
            raise ValueError('invalid parking corner')
        if any(type(point.get(axis)) not in (int, float) or not math.isfinite(point[axis])
               for axis in ('x', 'y')):
            raise ValueError('invalid parking coordinate')
        result.append({axis: point[axis] for axis in ('x', 'y')})
    return result


@lru_cache(maxsize=4)
def _read_markings(path, mtime_ns, size):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        data = json.load(stream)
    if (data.get('schema') not in (1, 2) or data.get('coordinate_frame') != 'carla_world'
            or not isinstance(data.get('outlines'), list)):
        raise ValueError('invalid parking marking asset')
    for row in data['outlines']:
        if (not isinstance(row.get('id'), str) or not isinstance(row.get('road_id'), str)
                or not isinstance(row.get('object_ids'), list) or not row['object_ids']
                or any(not isinstance(value, str) for value in row['object_ids'])):
            raise ValueError('invalid parking marking source')
        _corners(row.get('corners'), native=True)
        if 'parts' in row:
            if not isinstance(row['parts'], list) or not 1 <= len(row['parts']) <= 32:
                raise ValueError('invalid parking boundary parts')
            for part in row['parts']:
                _corners(part, native=True)
    return data


def build_parking_display(status, scenario_dir):
    name = str(status.get('map_name', '')).rsplit('/', 1)[-1]
    source = (status.get('parking_map') or {}).get('opendrive_sha256', '')
    if not re.fullmatch(r'[A-Za-z0-9_]+', name) or not re.fullmatch(r'[0-9a-f]{64}', source):
        raise ValueError('active map identity unavailable')
    path = Path(scenario_dir) / 'parking_maps' / f'{name}.parking-lines.json.gz'
    stat = path.stat()
    native = _read_markings(str(path), stat.st_mtime_ns, stat.st_size)
    if native.get('map_name') != name or native.get('opendrive_sha256') != source:
        raise ValueError('parking markings do not match the active map')
    configured = []
    physical_slots = {}
    replaced = set()
    for row in status.get('slot_choices', []):
        if type(row.get('slot_id')) is not int:
            raise ValueError('invalid configured parking identity')
        identifier = row.get('physical_slot_id', row['slot_id'])
        if type(identifier) is not int:
            raise ValueError('invalid physical parking identity')
        corners = _corners(row.get('corners'))
        if identifier in physical_slots:
            if corners != physical_slots[identifier]:
                raise ValueError('parking variants disagree on physical outline')
        else:
            physical_slots[identifier] = corners
            configured.append({'id': f'slot:{identifier}', 'slot_id': identifier,
                               'source': 'configured', 'corners': corners})
        reference = row.get('native_source') or {}
        replaced.update((str(reference.get('road_id')), str(object_id))
                        for object_id in reference.get('object_ids', []))
    # A surveyed target replaces its native counterpart; occupied spaces and
    # spaces absent from the target chooser remain in the display catalog.
    extra = [dict(row, slot_id=None, source='map_marking') for row in native['outlines']
             if not any((row['road_id'], object_id) in replaced for object_id in row['object_ids'])]
    return {'available': True, 'session_id': status['session_id'], 'map_name': status['map_name'],
            'opendrive_sha256': source, 'coordinate_frame': 'carla_world',
            'configured_count': len(configured), 'native_count': len(extra),
            'native_marking_count': len(native['outlines']), 'slots': configured + extra}
