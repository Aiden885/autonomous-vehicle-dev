"""Read-only geometry from the Bridge messages consumed by APA."""
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'carla_bridge'))
from apa_scene_limits import SCENE_MAX_OBSTACLES
from apa_scene_diagnostics import obstacle_kind

EGO_TOPIC = 'gaasd.carla.ego_state.v1'
REQUEST_TOPIC = 'gaasd.carla.apa_plan_request.v1'


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('invalid_geometry')
    return value


def _point(x, y):
    return [_number(x), _number(y)]


def _lines(payload, count_key, fields):
    count = payload.get(count_key, 0)
    if type(count) is not int or not 0 <= count <= 64:
        raise ValueError('invalid_geometry')
    arrays = [payload.get(key, []) for key in fields]
    if any(not isinstance(a, list) or len(a) < count for a in arrays):
        raise ValueError('invalid_geometry')
    return [[_point(arrays[0][i], arrays[1][i]), _point(arrays[2][i], arrays[3][i])]
            for i in range(count)]


def build_input_map(messages, status, now, stale_sec=1.5):
    """Current scene and same-task, target-matched boundaries.

    Boundaries retain their source frame independently of the live scene frame.
    """
    unavailable = {'available': False, 'reason': 'waiting_for_scene'}
    session, task = status.get('session_id'), status.get('task_id')
    entry = messages.get(EGO_TOPIC, {})
    ego = entry.get('payload') or {}
    received = entry.get('received_at', 0)
    if (not session or (ego.get('session_id'), ego.get('task_id')) != (session, task)
            or not 0 <= now-received <= stale_sec):
        return unavailable
    scene = ego.get('scene')
    if not isinstance(scene, dict) or not isinstance(scene.get('obstacles'), list):
        return unavailable
    request_entry = messages.get(REQUEST_TOPIC, {})
    request = request_entry.get('payload') or {}
    matching_request = ((request.get('session_id'), request.get('task_id')) == (session, task)
                        and request.get('target_slot_id') == status.get('slot_id'))
    try:
        obstacles = []
        if len(scene['obstacles']) > SCENE_MAX_OBSTACLES:
            raise ValueError('invalid_geometry')
        for item in scene['obstacles']:
            if not isinstance(item, dict):
                raise ValueError('invalid_geometry')
            points = item.get('footprint', [])
            if not isinstance(points, list) or not 3 <= len(points) <= 8:
                raise ValueError('invalid_geometry')
            identifier = str(item.get('id', ''))
            kind = {'unknown': 'unknown', 'forbidden_region': 'forbidden',
                    'map_vehicle': 'live', 'live_actor': 'live'}.get(obstacle_kind(identifier), 'static')
            obstacles.append({'id': identifier, 'kind': kind,
                              'points': [_point(p['x'], p['y']) for p in points],
                              'velocity': _point(item.get('velocity_x', 0), item.get('velocity_y', 0))})
        contract = scene.get('contract') or {}
        frame = ego.get('carla_frame_id', ego.get('frame_id'))
        if (scene.get('version') != frame or
                contract.get('coordinate_frame', 'carla_world') != 'carla_world'):
            raise ValueError('invalid_geometry')
        bounds = contract.get('query_bounds')
        if bounds is not None:
            if not isinstance(bounds, list) or len(bounds) != 4:
                raise ValueError('invalid_geometry')
            bounds = [_number(v) for v in bounds]
        result = {'available': True, 'session_id': session, 'task_id': task,
                  'scene_version': scene.get('version'), 'frame': frame,
                  'height_band': contract.get('vehicle_height_band'),
                  'bounds': bounds, 'obstacles': obstacles, 'boundaries_available': matching_request,
                  'boundary_frame': None, 'boundary_lines': [], 'virtual_lines': [], 'stopper_lines': [],
                  'constraint_points': [], 'target_slot': [], 'goal': None,
                  'ego_pose': _point(ego.get('vehicle_x'), ego.get('vehicle_y'))+[_number(ego.get('vehicle_yaw'))]}
        if matching_request:
            result['boundary_frame'] = request.get('carla_frame_id', request.get('frame_id'))
            result['boundary_lines'] = _lines(request, 'boundary_line_num',
                ('boundary_line_x1', 'boundary_line_y1', 'boundary_line_x2', 'boundary_line_y2'))
            result['virtual_lines'] = _lines(request, 'virtual_boundary_line_num',
                ('virtual_boundary_p1_x', 'virtual_boundary_p1_y', 'virtual_boundary_p2_x', 'virtual_boundary_p2_y'))
            result['stopper_lines'] = _lines(request, 'slot_track_stopper_num',
                ('slot_track_stopper_x1', 'slot_track_stopper_y1', 'slot_track_stopper_x2', 'slot_track_stopper_y2'))
            for prefix in ('boundary_sep_point', 'stopper_point'):
                count = request.get(prefix+'_num', 0)
                xs, ys = request.get(prefix+'_x', []), request.get(prefix+'_y', [])
                if (type(count) is not int or not 0 <= count <= 128
                        or not isinstance(xs, list) or not isinstance(ys, list) or min(len(xs), len(ys)) < count):
                    raise ValueError('invalid_geometry')
                result['constraint_points'].extend({'id': f'{prefix}:{i}', 'point': _point(xs[i], ys[i])}
                                                   for i in range(count))
            result['target_slot'] = [_point(request[f'slot_point{i}_x'], request[f'slot_point{i}_y'])
                                     for i in range(1, 5)]
            result['goal'] = _point(request['target_x'], request['target_y'])+[_number(request['target_theta'])]
        return result
    except (ValueError, KeyError, TypeError):
        return {'available': False, 'reason': 'invalid_geometry'}
