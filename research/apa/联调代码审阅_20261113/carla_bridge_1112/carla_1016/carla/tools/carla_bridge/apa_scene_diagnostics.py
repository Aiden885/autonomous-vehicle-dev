"""Frame-matched scene snapshots and failure provenance."""
from collections import Counter, deque
import hashlib
import json


def fields(value):
    return value if isinstance(value, dict) else {}


def obstacle_kind(identifier):
    if ':unknown:' in identifier:
        return 'unknown'
    if identifier.startswith('offline-map:forbidden:'):
        return 'forbidden_region'
    if identifier.startswith('occupancy:'):
        return 'map_vehicle'
    if identifier.startswith('actor:'):
        return 'live_actor'
    if identifier.startswith('map:'):
        return 'static_geometry'
    return 'unclassified'


def scene_contract(map_name, revision, profile_id, bounds, height_band, obstacles):
    return {'version': 1, 'coordinate_frame': 'carla_world',
            'vehicle_reference_point': 'rear_axle_center', 'map_name': map_name,
            'static_map_revision': revision, 'vehicle_profile_id': profile_id,
            'query_bounds': list(bounds), 'vehicle_height_band': list(height_band),
            'obstacle_classes': dict(Counter(obstacle_kind(p['id']) for p in obstacles))}


def failure_stage(control, safety_reason, command, map_state):
    """Classify only reported evidence; native planner stages are not on the wire."""
    validation = fields(control.get('input_validation'))
    online = fields(control.get('online_safety'))
    replan = fields(control.get('replan'))
    if safety_reason.startswith('collision:'):
        return 'carla_collision', safety_reason
    if validation.get('error'):
        return 'input_validation', validation['error']
    if online.get('blocked') is True:
        return 'online_safety', online.get('reason', '')
    if control.get('control_status') == 'plan_failed':
        reason = replan.get('reason', '')
        if reason in ('candidate_blocked', 'unsafe_candidate', 'stale_plan_start'):
            return 'candidate_validation', reason
        return 'planning', reason
    if control.get('control_status') == 'tracking_fault':
        return 'tracking', str(fields(control.get('diagnostic')).get('stop_reason', ''))
    if safety_reason.startswith('invalid_ego_feedback:'):
        return 'feedback_build', safety_reason
    if safety_reason.startswith('invalid_plan_request:'):
        return 'request_build', safety_reason
    if safety_reason and safety_reason not in ('cancelled', 'shutdown'):
        return 'bridge_safety', safety_reason
    if map_state == 'unavailable':
        return 'map_readiness', 'offline_map_unavailable'
    if command.get('ok') is False:
        return 'task_admission', command.get('message', '')
    return '', ''


def describe_blocker(identifier, feedback, request):
    if not identifier:
        return {}
    blocker = {'id': identifier, 'kind': obstacle_kind(identifier)}
    if identifier.startswith('json:'):
        blocker['kind'] = 'native_constraint'
        parts = identifier.split(':')
        if request and len(parts) == 3 and parts[2].isdigit():
            index = int(parts[2])
            group = {'boundary': 'boundary_sources', 'virtual': 'virtual_sources'}.get(parts[1])
            policy = request['payload'].get('boundary_policy', {})
            for source in policy.get(group, []):
                if source['start'] <= index < source['start'] + source['count']:
                    blocker.update(kind=source.get('kind', 'virtual_constraint'), source=source['source'])
                    for key in ('source_index', 'source_id', 'source_name', 'provenance'):
                        if key in source:
                            blocker[key] = source[key]
                    if source.get('slot_edges'):
                        blocker.update(source['slot_edges'][index-source['start']])
                    if source.get('shared_edge_id'):
                        blocker.update(shared_edge_id=source['shared_edge_id'],
                                       slot_edges=list(source['slot_edges']))
                    break
    elif feedback:
        blocker['source_piece_ids'] = feedback[2].get(identifier, [identifier])
        blocker['footprint'] = next((p['footprint'] for p in feedback[1]['payload']['scene']['obstacles']
                                     if p['id'] == identifier), None)
    return blocker


class SceneDiagnostics:
    """Four matching-frame records and two bounded, atomically replaced snapshots."""
    def __init__(self, directory, config, source_paths):
        endpoint = hashlib.sha256(config['zmq']['publish_bind'].encode()).hexdigest()[:12]
        self.directory = directory / endpoint
        self.config = config
        self.sources = {}
        for path in source_paths:
            try:
                self.sources[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
            except OSError:
                self.sources[path.name] = None
        self.feedback = deque(maxlen=4)
        self.requests = deque(maxlen=4)
        self.task = None
        self.started = False
        self.saved_events = set()
        self.files = {}
        self.error = ''
        self.response = None
        self.response_feedback = self.response_request = None

    @staticmethod
    def identity(payload):
        return payload.get('session_id'), payload.get('task_id')

    def observe(self, topic, envelope, provenance):
        payload = envelope['payload']
        if topic.endswith('.ego_state.v1'):
            key = self.identity(payload) + (payload.get('carla_frame_id'),)
            item = (key, envelope, provenance)
            if self.feedback and self.feedback[-1][0] == key:
                self.feedback[-1] = item
            else:
                self.feedback.append(item)
        elif topic.endswith('.apa_plan_request.v1'):
            self.requests.append(envelope)

    def _request(self, task, frame):
        return next((e for e in reversed(self.requests)
                     if self.identity(e['payload']) == task and
                     e['payload'].get('carla_frame_id', 0) <= frame), None)

    def observe_control(self, control):

        task, frame = self.identity(control), control.get('carla_frame_id')
        online = fields(control.get('online_safety'))
        matched = next((item for item in reversed(self.feedback) if item[0] == task + (frame,)), None)
        if (type(frame) is not int or online.get('scene_version', frame) != frame or
                (matched and control.get('sim_time_sec') != matched[1]['payload'].get('sim_time_sec'))):
            matched = None
        self.response, self.response_feedback = control, matched
        self.response_request = self._request(task, frame) if matched else None

    def _save(self, name, diagnostic, feedback, request, control, context, latest):
        evidence = {'schema_version': 1, 'producer': 'carla_bridge_apa',
                    'session_id': self.task[0], 'task_id': self.task[1],
                    'source_sha256': self.sources, 'startup_config': self.config,
                    'diagnostic': diagnostic, 'feedback': feedback[1] if feedback else None,
                    'plan_request': request, 'control_response': control,
                    'compacted_sources': feedback[2] if feedback else {},
                    'latest_feedback_context': latest[1] if latest and latest is not feedback else None,
                    'context': context}
        try:
            data = json.dumps(evidence, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory / (name + '.json')
            temporary = path.with_suffix('.tmp')
            temporary.write_text(data + '\n', encoding='utf-8')
            temporary.replace(path)
            self.files[name] = str(path)
            self.error = ''
            print(f'[APA diagnostic] {name}: {path}', flush=True)
        except (OSError, TypeError, ValueError) as exc:
            self.error = str(exc)
            print(f'[APA diagnostic] snapshot unavailable: {exc}', flush=True)

    def update(self, task, control, safety_reason, command, context):
        if task != self.task:
            self.task, self.started = task, False
            self.saved_events.clear()
            self.files, self.error = {}, ''
        control = control if self.identity(control) == task else {}
        command = command if (command.get('source_session_id'), command.get('task_id')) == task else {}
        latest = next((item for item in reversed(self.feedback) if item[0][:2] == task), None)
        request = self._request(task, latest[0][2]) if latest else None
        if not self.started and latest and request and latest[1]['payload'].get('parking_enabled'):
            self.started = True
            self._save('task-start', {'stage': 'task_start'}, latest, request, {}, context, latest)

        stage, reason = failure_stage(control, safety_reason, command,
                                      context.get('map_preparation', {}).get('state'))
        online = fields(control.get('online_safety'))
        frame = control.get('carla_frame_id')
        matched = self.response_feedback if self.response is control else None
        feedback = matched if control else latest
        request = self.response_request if matched else self._request(task, feedback[0][2]) if feedback else None
        identifier = str(online.get('obstacle_id') or '')
        planner_error = fields(control.get('replan')).get('planner_error_type', 0)
        planner_error = planner_error if type(planner_error) is int else None
        diagnostic = {'stage': stage, 'reason': reason, 'native_planner_stage': None,
                      'response_frame': frame, 'feedback_matches_response': bool(matched) if control else None,
                      'planner_error_type': planner_error,
                      'blocker': describe_blocker(identifier, feedback, request)}
        if stage:
            event = (stage, str(reason), identifier, planner_error, bool(feedback),
                     command.get('event_id') if stage == 'task_admission' else None)

            if event not in self.saved_events and len(self.saved_events) < 16:
                self.saved_events.add(event)
                self._save('last-event', diagnostic, feedback, request, control, context, latest)
        diagnostic.update(snapshot_files=dict(self.files), snapshot_error=self.error,
                          snapshot_event_limit_reached=len(self.saved_events) >= 16)
        return diagnostic
