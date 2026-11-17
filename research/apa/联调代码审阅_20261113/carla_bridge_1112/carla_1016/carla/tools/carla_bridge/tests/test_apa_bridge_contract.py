import importlib.util
import math
import sys
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

BRIDGE_DIR = Path(__file__).resolve().parents[1]


def load_bridge_module():
    module_name = "carla_bridge_apa_contract_test"
    spec = importlib.util.spec_from_file_location(
        module_name, BRIDGE_DIR / "carla_bridge_apa.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeVehicleControl:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class FakeEgo:
    def __init__(self, transform):
        self.transform = transform
        self.applied_control = None

    def get_transform(self):
        return self.transform

    def get_velocity(self):
        return SimpleNamespace(x=0.0, y=0.0, z=0.0)

    def get_control(self):
        return self.applied_control or SimpleNamespace(gear=1, steer=0.0)

    def apply_control(self, control):
        self.applied_control = control

    def set_transform(self, transform):
        self.transform = transform


class ApaBridgeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_module = load_bridge_module()

    def make_bridge(self):
        bridge = object.__new__(self.bridge_module.ApaCarlaBridge)
        bridge.vehicle_profile_id = self.bridge_module.default_config()["apa"]["vehicle_profile_id"]
        bridge.safety_stop = self.bridge_module.SafetyStop(self.bridge_module.default_config()["safety"])
        bridge._recovery_config = self.bridge_module.default_config()["safety"]
        bridge.recovery_stop = self.bridge_module.SafetyStop(bridge._recovery_config)
        bridge._recovery_id = bridge._path_version = 0
        bridge.client = bridge.world = bridge.ego = bridge.carla = None
        bridge.environment_obstacles = []
        bridge.disabled_environment_ids = set()
        bridge.scene_payload = mock.Mock(side_effect=lambda snapshot: {"version": snapshot.frame, "obstacles": []})
        return bridge

    def make_slot(self, overrides=None):
        slot = {"slot_id": 29, "slot_kind": "vertical", "goal_x": 30.0, "goal_y": 40.0,
                "goal_reference_point": "rear_axle_center",
                "goal_yaw": 0.0, "slot_entry_pt_x": 0.0, "slot_entry_pt_y": 1.0,
                "slot_pt1_x": 0.0, "slot_pt1_y": 0.0, "slot_pt2_x": 0.0, "slot_pt2_y": 2.0,
                "slot_pt3_x": 5.0, "slot_pt3_y": 2.0, "slot_pt4_x": 5.0, "slot_pt4_y": 0.0,
                "slot_entry": 1}
        slot.update(overrides or {})
        return slot

    def make_transform(self):
        return SimpleNamespace(
            location=SimpleNamespace(x=10.0, y=20.0),
            rotation=SimpleNamespace(yaw=90.0),
            get_up_vector=lambda: SimpleNamespace(x=0.0, y=0.0, z=1.0),
        )

    def test_safety_standstill_requires_distinct_continuous_physics_frames(self):
        stop = self.bridge_module.SafetyStop(self.bridge_module.default_config()["safety"])
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            stop.latch("collision: wall")
            for _ in range(30):
                stop.observe(10, 1.0, 0.0, 0.02)
            self.assertEqual(stop.state, "braking")
            self.assertEqual(stop.standstill_sec, 0.0)
            for i in range(1, 11):
                stop.observe(10 + i, 1.0 + i * 0.02, 0.0, 0.02)
            self.assertEqual(stop.state, "stopped")
            self.assertAlmostEqual(stop.standstill_sec, 0.20)

    def test_repeated_collision_cannot_extend_budget_or_clear_fault(self):
        stop = self.bridge_module.SafetyStop(self.bridge_module.default_config()["safety"])
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            stop.latch("collision: wall")
        deadline = stop.deadline
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=102.9):
            stop.latch("collision: wall")
        self.assertEqual(stop.deadline, deadline)
        self.assertFalse(stop.clear())
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=103.1):
            stop.observe(11, 1.0, 0.0, 0.02)
        self.assertEqual(stop.state, "unconfirmed")
        self.assertEqual(stop.reason, "collision: wall")

    def test_collision_during_cancel_requires_scene_reset(self):
        bridge = self.make_bridge()
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            bridge.safety_stop.latch("cancelled")
            bridge.safety_stop.latch("collision: wall")
        self.assertEqual(bridge.safety_stop.reason, "collision: wall")
        with self.assertRaisesRegex(ValueError, "重置"):
            bridge.apply_apa_command({"action": "park_in"})

    def test_safety_frame_gap_requires_a_fresh_stationary_dwell(self):
        stop = self.bridge_module.SafetyStop(self.bridge_module.default_config()["safety"])
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            stop.latch("control timeout")
            stop.observe(10, 1.0, 0.0, 0.02)
            stop.observe(25, 1.3, 0.0, 0.02)
            self.assertEqual(stop.state, "unconfirmed")
            self.assertFalse(stop.clear())
            for _ in range(30):
                stop.observe(25, 1.3, 0.0, 0.02)
            self.assertEqual(stop.standstill_sec, 0.0)
            for i in range(1, 11):
                stop.observe(25 + i, 1.3 + i * 0.02, 0.0, 0.02)
            self.assertEqual(stop.state, "stopped")
            self.assertEqual(stop.reason, "control timeout")

    def test_safety_feedback_detects_sideways_motion(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.ego = SimpleNamespace(id=7)
        moving = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=0.3, y=0.0, z=0.0))
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            bridge.safety_stop.latch("collision: wall")
            for i in range(11):
                bridge.observe_safety_stop(SimpleNamespace(frame=10 + i,
                    timestamp=SimpleNamespace(elapsed_seconds=1.0 + i * 0.02, delta_seconds=0.02),
                    find=lambda _: moving))
        self.assertEqual(bridge.safety_stop.state, "braking")
        self.assertEqual(bridge.safety_stop.standstill_sec, 0.0)

    def test_old_algorithm_command_never_overrides_a_latched_collision(self):
        bridge = self.make_control_bridge()
        bridge._pending_control = None
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            bridge.handle_collision(SimpleNamespace(other_actor=None))
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=110.0):
            bridge.apply_control(self.reverse_command(True))
        self.assertEqual(bridge.ego.applied_control.throttle, 0.0)
        self.assertEqual(bridge.ego.applied_control.brake, 1.0)
        self.assertTrue(bridge.safety_stop.reason.startswith("collision:"))

    def test_stopping_rpc_failure_can_reconfirm_after_rpc_recovers(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.ego.id = 7
        bridge.ego_payload = mock.Mock(return_value={})
        bridge.publish = mock.Mock()
        actor = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0))
        def snapshot(frame):
            return SimpleNamespace(frame=frame,
                timestamp=SimpleNamespace(elapsed_seconds=frame * 0.02, delta_seconds=0.02), find=lambda _: actor)
        bridge.tick_world = mock.Mock(side_effect=[RuntimeError("CARLA unavailable")] +
                                     [snapshot(frame) for frame in range(11, 21)])
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            bridge.safety_stop.latch("control timeout")
            bridge.advance_safety_stop(snapshot(10))
            self.assertEqual(bridge.safety_stop.state, "unconfirmed")
            for frame in range(10, 20):
                bridge.advance_safety_stop(snapshot(frame))
        self.assertEqual(bridge.safety_stop.state, "stopped")
        self.assertEqual(bridge.safety_stop.reason, "control timeout")
        self.assertAlmostEqual(bridge.safety_stop.standstill_sec, 0.20)
        self.assertEqual(bridge.ego.applied_control.brake, 1.0)

    def test_safety_reset_is_rejected_before_physical_standstill(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.ego.id = 7
        bridge.ego.set_transform = mock.Mock()
        moving = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=0.3, y=0.0, z=0.0))
        snapshot = SimpleNamespace(frame=10,
            timestamp=SimpleNamespace(elapsed_seconds=1.0, delta_seconds=0.02), find=lambda _: moving)
        bridge.world = SimpleNamespace(get_snapshot=lambda: snapshot)
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            bridge.safety_stop.latch("collision: wall")
            with self.assertRaisesRegex(ValueError, "尚未确认停稳"):
                bridge.reset_parking_task({"action": "reset"})
        bridge.ego.set_transform.assert_not_called()

    def test_safety_braking_does_not_depend_on_zmq_feedback(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.ego.id = 7
        bridge.ego_payload = mock.Mock(return_value={})
        bridge.publish = mock.Mock(side_effect=RuntimeError("ZMQ unavailable"))
        actor = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=0.2, y=0.0, z=0.0))
        def snapshot(frame):
            return SimpleNamespace(frame=frame,
                timestamp=SimpleNamespace(elapsed_seconds=frame * 0.02, delta_seconds=0.02),
                find=lambda _: actor)
        bridge.tick_world = mock.Mock(return_value=snapshot(11))
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=100.0):
            bridge.safety_stop.latch("control timeout")
            bridge.advance_safety_stop(snapshot(10))
        bridge.tick_world.assert_called_once()
        self.assertEqual(bridge.safety_stop.state, "braking")
        self.assertEqual(bridge.ego.applied_control.brake, 1.0)

    def test_actor_to_rear_axle_respects_heading_and_configured_offset(self):
        bridge = self.make_bridge()
        bridge.cfg = {"apa": {"rear_axle_offset_m": 2.0}}
        for yaw, expected in (
            (0.0, (8.0, 20.0)),
            (90.0, (10.0, 18.0)),
            (-90.0, (10.0, 22.0)),
            (180.0, (12.0, 20.0)),
            (45.0, (10.0 - math.sqrt(2.0), 20.0 - math.sqrt(2.0))),
        ):
            with self.subTest(yaw=yaw):
                transform = self.make_transform()
                transform.rotation.yaw = yaw
                x, y, theta = bridge.rear_axle_pose(transform)
                self.assertAlmostEqual(x, expected[0])
                self.assertAlmostEqual(y, expected[1])
                self.assertAlmostEqual(theta, math.radians(yaw))
                self.assertEqual((transform.location.x, transform.location.y), (10.0, 20.0))
        self.assertEqual(bridge.rear_axle_pose(None), (0.0, 0.0, 0.0))

    def test_timing_requires_a_fixed_step_that_divides_control_period(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.validate_timing()
        bridge.cfg["carla"]["fixed_delta_seconds"] = 0.01
        bridge.validate_timing()
        bridge.cfg["carla"]["fixed_delta_seconds"] = 1.0 / 30.0
        with self.assertRaises(ValueError):
            bridge.validate_timing()
        bridge.cfg["carla"]["fixed_delta_seconds"] = 0.02
        bridge.cfg["carla"]["sync_mode"] = False
        with self.assertRaises(ValueError):
            bridge.validate_timing()

    def test_feedback_is_sampled_once_per_frame_using_simulation_time(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.ego = FakeEgo(self.make_transform())
        bridge.parking_action = 1
        bridge.parking_enabled = True
        bridge.current_apa_gear_pos = 1
        velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        bridge.ego.get_velocity = mock.Mock(side_effect=lambda: velocity)
        snapshot = SimpleNamespace(frame=1, timestamp=SimpleNamespace(elapsed_seconds=1.0, delta_seconds=0.02))
        initial = bridge.ego_payload(snapshot)
        velocity.y = 0.4
        with mock.patch.object(self.bridge_module.time, "monotonic", return_value=1000000.0):
            repeated = bridge.ego_payload(snapshot)
        self.assertEqual(repeated["vehicle_acceleration"], initial["vehicle_acceleration"])
        bridge.ego.get_velocity.assert_called_once()
        snapshot = SimpleNamespace(frame=2, timestamp=SimpleNamespace(elapsed_seconds=1.02, delta_seconds=0.02))
        next_frame = bridge.ego_payload(snapshot)

        self.assertAlmostEqual(next_frame["vehicle_acceleration"], 1.0)
        self.assertEqual(next_frame["carla_frame_id"], 2)

    def make_sync_bridge(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.parking_enabled = True
        bridge._sync_fault = ""
        bridge.parking_action = 1
        bridge.session_id = "session-a"
        bridge.task_id = 3
        bridge._sync_task_key = (True, 1)
        bridge._control_ack = False
        bridge._recovery_id = bridge._path_version = 1
        bridge._pending_control = {"session_id": "session-a", "task_id": 3, "carla_frame_id": 42, "sim_time_sec": 1.0}
        bridge.apply_control = mock.Mock()
        bridge.apply_sync_hold = mock.Mock()
        return bridge

    def synchronized_control(self):
        return {
            "actuator": {"owner": "C:apaMocoActuate:v2", "throttle": 0.0, "brake": 1.0,
                         "hand_brake": False, "launch_hold": False, "gear_pos": 1},
            "vehicle_profile_id": self.bridge_module.default_config()["apa"]["vehicle_profile_id"],
            "session_id": "session-a", "task_id": 3, "carla_frame_id": 42,
            "sim_time_sec": 1.0, "control_dt_sec": 0.02, "control_status": "tracking",
            "enable": True, "StopReq": False, "TargetSpeed": 0.2,
            "TargetAcceleration": 0.1, "FrontWheelAngle": 0.0,
            "TargetGearReq": 1, "DirectionSign": 1, "GearChangeAllowed": False, "CtrlPhase": 3,
            "safety_state": {"event_id": 0, "reason": "", "state": "none", "standstill_sec": 0.0},
            "input_validation": {"feedback_valid": True, "request_valid": True, "error": ""},
            "online_safety": {"blocked": False, "scene_version": 42, "plan_scene_version": 40,
                              "reason": "", "obstacle_id": "", "checked_distance_m": 1.0},
            "replan": {"recovery_id": 1, "path_version": 1, "state": "ready", "attempt": 1, "reason": "", "elapsed_sec": 0.0},
            "actuation_limits": {"max_abs_target_speed_mps": 0.8, "max_abs_accel_mps2": 1.5,
                                 "max_abs_steer_rad": self.bridge_module.default_config()["control"]["max_abs_steer_rad"]},
        }

    def motion_braking_control(self):
        payload = self.synchronized_control()
        payload.update(enable=False, StopReq=True, TargetSpeed=0.0,
                       TargetAcceleration=0.0, CtrlPhase=4, control_status="motion_braking")
        payload["diagnostic"] = {"stop_reason": 2, "segment": 1, "segment_count": 1}
        payload["online_safety"].update(blocked=True, reason="clearance_predicted", obstacle_id="json:virtual:0")
        return payload

    def test_motion_prediction_brakes_without_invalidating_path(self):
        for phase in (3, 4, 7, 8):
            for reason in ("clearance_predicted", "motion_blocked", ""):
                for dt in (0.0, 0.02):
                    with self.subTest(phase=phase, reason=reason, dt=dt):
                        bridge = self.make_sync_bridge()
                        payload = self.motion_braking_control()
                        payload.update(CtrlPhase=phase, control_dt_sec=dt)
                        payload["online_safety"].update(blocked=bool(reason), reason=reason)
                        bridge.accept_control(payload)
                        bridge.apply_control.assert_called_once_with(payload)
                        self.assertTrue(bridge._control_ack)
                        self.assertFalse(bridge._sync_fault)
                        self.assertFalse(bridge.recovery_stop.reason)
                        self.assertEqual(bridge._path_version, payload["replan"]["path_version"])

    def test_motion_braking_never_admits_drive_contact_or_wrong_frame(self):
        changes = [({"enable": True}, {}), ({"StopReq": False}, {}),
                   ({"TargetSpeed": 0.1}, {}), ({"TargetAcceleration": 0.1}, {}),
                   ({"GearChangeAllowed": True}, {}),
                   ({}, {"reason": "body_blocked"}), ({}, {"reason": "line_contact"}),
                   ({}, {"reason": "clearance_deficit"}), ({}, {"reason": "path_blocked"}),
                   ({"control_status": "tracking"}, {}),
                   ({"carla_frame_id": 41}, {})]
        for fields, safety in changes:
            with self.subTest(fields=fields, safety=safety):
                bridge = self.make_sync_bridge()
                payload = self.motion_braking_control()
                payload.update(fields)
                payload["online_safety"].update(safety)
                bridge.accept_control(payload)
                bridge.apply_control.assert_not_called()

    def test_control_caps_and_commands_must_fit_bridge_limits(self):
        for key, value in (("TargetSpeed", 0.9), ("TargetAcceleration", -1.6), ("FrontWheelAngle", 0.9)):
            bridge = self.make_sync_bridge()
            payload = self.synchronized_control()
            payload[key] = value
            bridge.accept_control(payload)
            bridge.apply_control.assert_not_called()
            self.assertIn("limit", bridge._sync_fault)
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload["actuation_limits"]["max_abs_target_speed_mps"] = bridge.cfg["control"]["max_abs_target_speed_mps"] + 0.1
        bridge.accept_control(payload)
        self.assertIn("align configurations", bridge._sync_fault)
        bridge.apply_control.assert_not_called()

    def test_mismatched_compiled_vehicle_profile_cannot_drive(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload["vehicle_profile_id"] = "outdated-vehicle-profile"
        bridge.accept_control(payload)
        bridge.apply_control.assert_not_called()
        self.assertIn("vehicle profile mismatch", bridge._sync_fault)

    def test_cancel_follows_only_the_identified_pending_task_transition(self):
        for pending_event, expected in (("start-event", True), ("unrelated-event", False)):
            bridge = self.make_sync_bridge()
            bridge.ego = bridge.carla = object()
            bridge._task_command_results = {
                "start-event": {"event_id": "start-event", "ok": True,
                                "source_task_id": 2, "task_id": 3},
            }
            bridge.apply_manual_hold = mock.Mock()

            def stop_task():
                bridge.parking_enabled = False
                bridge.task_id += 1
                bridge._sync_task_key = (False, bridge.parking_action)

            bridge.stop_parking_task = mock.Mock(side_effect=stop_task)
            payload = {"session_id": "session-a", "task_id": 2, "action": "cancel",
                       "event_id": "cancel-event", "cancel_pending_event_id": pending_event}
            bridge.handle_task_command(payload)
            self.assertEqual(bridge.last_task_command["ok"], expected)
            self.assertEqual(bridge.stop_parking_task.call_count, int(expected))
            bridge.handle_task_command(payload)
            self.assertEqual(bridge.stop_parking_task.call_count, int(expected))

    def test_old_protocol_without_actuation_caps_cannot_drive(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload.pop("actuation_limits")
        bridge.accept_control(payload)
        bridge.apply_control.assert_not_called()
        self.assertTrue(bridge.safety_stop.reason)

    def test_legacy_inactive_control_settings_are_rejected(self):
        bridge = self.make_sync_bridge()
        bridge.validate_control_config()
        bridge.cfg["control"]["reverse_steer_sign"] = -1.0
        with self.assertRaisesRegex(ValueError, "reverse_steer_sign"):
            bridge.validate_control_config()

    def test_only_matching_control_is_applied_once(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        for key, wrong in (("session_id", "old-session"), ("task_id", 2), ("carla_frame_id", 41)):
            bridge.accept_control(dict(payload, **{key: wrong}))
        bridge.apply_control.assert_not_called()
        bridge.accept_control(payload)
        bridge.accept_control(payload)
        bridge.apply_control.assert_called_once_with(payload)
        self.assertTrue(bridge._control_ack)

    def test_input_rejection_stops_and_exposes_the_field_reason(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload["input_validation"] = {"feedback_valid": True, "request_valid": False,
                                       "error": "array_length:boundary_line_y2"}
        bridge.accept_control(payload)
        bridge.apply_control.assert_not_called()
        self.assertIn("boundary_line_y2", bridge._sync_fault)
        self.assertEqual(bridge.last_control_payload["input_validation"], payload["input_validation"])

    def test_unvalidated_and_invalid_enum_controls_cannot_drive(self):
        payloads = []
        missing = self.synchronized_control()
        missing.pop("input_validation")
        payloads.append(missing)
        payloads.append(dict(self.synchronized_control(), TargetGearReq=True))
        payloads.append(dict(self.synchronized_control(), DirectionSign=9))
        for payload in payloads:
            bridge = self.make_sync_bridge()
            bridge.accept_control(payload)
            bridge.apply_control.assert_not_called()
            self.assertTrue(bridge._sync_fault)

    def test_obstacle_stop_uses_independent_braking_and_stays_latched(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload["online_safety"].update(blocked=True, reason="path_blocked", obstacle_id="actor:12")
        bridge.accept_control(dict(payload, task_id=2))
        self.assertFalse(bridge.safety_stop.reason)
        bridge.accept_control(payload)
        self.assertIn("actor:12", bridge.safety_stop.reason)
        bridge.accept_control(self.synchronized_control())
        bridge.apply_control.assert_not_called()
        bridge.apply_sync_hold.assert_called()

    def test_control_requires_current_scene_check(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload["online_safety"]["scene_version"] = 41
        bridge.accept_control(payload)
        bridge.apply_control.assert_not_called()
        self.assertIn("online scene", bridge._sync_fault)

    def test_recovery_holds_until_stopped_and_requires_new_path_version(self):
        bridge = self.make_sync_bridge()
        stopped = self.synchronized_control()
        stopped.update(control_status="replan_stopping", control_dt_sec=0.0, enable=False, StopReq=True)
        stopped["replan"].update(recovery_id=2, state="stopping")
        stopped["online_safety"].update(blocked=True, reason="path_blocked", obstacle_id="actor:12")
        bridge.accept_control(stopped)
        self.assertFalse(bridge.safety_stop.reason)
        self.assertEqual(bridge.recovery_stop.state, "braking")
        self.assertTrue(bridge._control_ack)
        self.assertIsNotNone(bridge._pending_control)
        bridge.apply_control.assert_called_once_with(stopped)
        bridge.apply_sync_hold.assert_not_called()
        bridge.apply_control.reset_mock()
        for i in range(12):
            bridge.recovery_stop.observe(42+i, 1.0+i*0.02, 0.0, 0.02)
        self.assertEqual(bridge.recovery_stop.state, "stopped")
        bridge._pending_control = {"session_id": "session-a", "task_id": 3, "carla_frame_id": 54, "sim_time_sec": 1.24}
        bridge._control_ack = False
        ready = self.synchronized_control()
        ready.update(carla_frame_id=54, sim_time_sec=1.24)
        ready["online_safety"]["scene_version"] = 54
        ready["replan"].update(recovery_id=2, path_version=2)
        bridge.accept_control(ready)
        bridge.apply_control.assert_called_once_with(ready)
        self.assertFalse(bridge.recovery_stop.reason)
        self.assertEqual(bridge._path_version, 2)
        bridge._control_ack = False
        bridge.apply_control.reset_mock()
        ready["replan"]["path_version"] = 1
        bridge.accept_control(ready)
        bridge.apply_control.assert_not_called()

    def test_recovery_braking_keeps_each_physics_frame_acknowledged(self):
        for acknowledged in (False, True):
            bridge = self.make_sync_bridge()
            snapshot = SimpleNamespace(frame=42)
            bridge.ego = object()
            bridge.world = SimpleNamespace(get_snapshot=lambda: snapshot)
            bridge.running = True
            bridge.ignore_control_until = 0.0
            bridge.recovery_stop.latch("replan")
            bridge.process_zmq_messages = mock.Mock()
            bridge.observe_safety_stop = mock.Mock()
            bridge.ego_payload = mock.Mock(return_value={"frame": 42})
            bridge.wait_for_control = mock.Mock(return_value=acknowledged)
            bridge.advance_safety_stop = mock.Mock()
            bridge.tick_world = mock.Mock(return_value=snapshot)
            bridge.record_actual_path = mock.Mock()
            bridge.publish_status_if_needed = mock.Mock()
            bridge.draw_trajectories = mock.Mock()
            bridge.cfg["apa"].update(draw_debug=False, follow_spectator=False)
            bridge._run_frame()
            bridge.wait_for_control.assert_called_once_with(snapshot, {"frame": 42})
            bridge.advance_safety_stop.assert_not_called()
            self.assertEqual(bridge.tick_world.call_count, int(acknowledged))

    def test_recovery_cannot_clear_collision_or_resume_while_braking(self):
        for collision in (False, True):
            bridge = self.make_sync_bridge()
            bridge.recovery_stop.latch("replan")
            if collision:
                bridge.safety_stop.latch("collision")
            ready = self.synchronized_control()
            ready["replan"]["path_version"] = 2
            bridge.accept_control(ready)
            bridge.apply_control.assert_not_called()
            self.assertTrue(bridge.safety_stop.reason)

    def test_predicted_body_crossing_waits_without_latching_collision_fault(self):
        for reason in ("body_predicted", "planning_clearance"):
            with self.subTest(reason=reason):
                bridge = self.make_sync_bridge()
                bridge.recovery_stop.latch("replan")
                for i in range(12):
                    bridge.recovery_stop.observe(30+i, 0.76+i*0.02, 0.0, 0.02)
                payload = self.synchronized_control()
                payload.update(control_status="replan_waiting", control_dt_sec=0.0, enable=False, StopReq=True)
                payload["replan"].update(state="waiting_scene", reason=reason)
                payload["online_safety"].update(blocked=True, reason=reason, obstacle_id="actor:12")
                bridge.accept_control(payload)
                bridge.apply_control.assert_called_once_with(payload)
                bridge.apply_sync_hold.assert_not_called()
                self.assertFalse(bridge.safety_stop.reason)
                self.assertTrue(bridge._control_ack)

    def test_actual_body_overlap_is_terminal_even_during_recovery(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload.update(control_status="replan_stopping", control_dt_sec=0.0, enable=False, StopReq=True)
        payload["replan"].update(state="stopping", recovery_id=2)
        payload["online_safety"].update(blocked=True, reason="body_blocked")
        bridge.accept_control(payload)
        self.assertIn("body_blocked", bridge.safety_stop.reason)
        bridge.apply_control.assert_not_called()

    def test_planning_timeout_is_terminal_and_visible(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload.update(control_status="planning_timeout", control_dt_sec=0.0, enable=False, StopReq=True)
        payload["replan"].update(state="failed", reason="planning_timeout")
        bridge.accept_control(payload)
        self.assertIn("planning_timeout", bridge.safety_stop.reason)
        bridge.apply_control.assert_not_called()

    def test_scene_samples_actor_snapshot_and_filters_ego_height_and_distance(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()

        def transform(x, z=0.0):
            return SimpleNamespace(location=SimpleNamespace(x=x, y=0.0, z=z), rotation=SimpleNamespace(yaw=0.0))

        def vertices(pose):
            return [SimpleNamespace(x=pose.location.x+dx, y=dy, z=pose.location.z+dz)
                    for dx in (-0.5, 0.5) for dy in (-0.5, 0.5) for dz in (0.0, 1.0)]

        box = SimpleNamespace(get_world_vertices=vertices)
        bridge.ego.id = 1
        bridge.ego.bounding_box = box
        actors = [SimpleNamespace(id=i, type_id="vehicle.test", bounding_box=box) for i in (1, 2, 3, 4)]
        bridge.world = SimpleNamespace(get_actors=lambda: actors)
        states = {i: SimpleNamespace(get_transform=lambda i=i: transform(3.0 if i in (2, 3) else 100.0 if i == 4 else 0.0,
                    10.0 if i == 3 else 0.0), get_velocity=lambda: SimpleNamespace(x=0.0, y=0.0)) for i in (1, 2, 3, 4)}
        snapshot = SimpleNamespace(frame=7, find=states.get)
        scene = self.bridge_module.ApaCarlaBridge.scene_payload(bridge, snapshot)
        self.assertEqual(scene["version"], 7)
        self.assertEqual([item["id"] for item in scene["obstacles"]], ["actor:2"])

    def test_scene_is_sampled_once_when_feedback_is_retransmitted(self):
        bridge = self.make_control_bridge()
        bridge.parking_action = 1
        snapshot = SimpleNamespace(frame=10, timestamp=SimpleNamespace(elapsed_seconds=1.0, delta_seconds=0.02))
        first = bridge.ego_payload(snapshot)
        second = bridge.ego_payload(snapshot)
        self.assertEqual(first["scene"], second["scene"])
        bridge.scene_payload.assert_called_once_with(snapshot)

    def test_height_slab_clips_a_leaning_box_and_keeps_its_low_body(self):
        module = self.bridge_module
        vertices = [SimpleNamespace(x=z+dx, y=dy, z=z)
                    for z in (0.0, 4.0) for dx in (-0.1, 0.1) for dy in (-0.2, 0.2)]
        obstacle = module.obstacle_from_vertices("map:leaning", vertices)
        parts = module.obstacle_at_height(obstacle, 0.0, 1.0)
        self.assertAlmostEqual(min(p["x"] for p in parts[0]["footprint"]), -0.1)
        self.assertAlmostEqual(max(p["x"] for p in parts[0]["footprint"]), 1.1)
        self.assertFalse(any(key.startswith("_") for key in parts[0]))
        self.assertEqual(module.obstacle_at_height(obstacle, 5.0, 6.0), [])

    def test_obstacle_height_band_follows_elevated_and_tilted_vehicle_body(self):
        module = self.bridge_module
        vertices = [SimpleNamespace(z=z) for z in (6.15, 6.3, 6.4, 6.55, 7.7, 7.85, 7.95, 8.1)]
        low, high, covered = module.vehicle_obstacle_height_band(vertices)
        self.assertEqual((low, high), (6.15, 8.1))
        self.assertLessEqual(covered[0], low)
        self.assertGreaterEqual(covered[1], high)
        obstacle = module.obstacle_from_vertices("map:overhead", [SimpleNamespace(x=x, y=y, z=z)
            for x in (-1., 1.) for y in (-1., 1.) for z in (9., 10.)])
        self.assertFalse(module.obstacle_at_height(obstacle, low, high))

    def test_current_task_rejection_stops_even_with_previous_validated_frame(self):
        bridge = self.make_sync_bridge()
        bridge._control_ack = True
        payload = self.synchronized_control()
        payload.update(carla_frame_id=41, sim_time_sec=0.98)
        payload["input_validation"]["error"] = "missing:target_x"
        bridge.accept_control(dict(payload, task_id=2))
        self.assertFalse(bridge._sync_fault)
        bridge.accept_control(payload)
        self.assertIn("missing:target_x", bridge._sync_fault)
        bridge.apply_control.assert_not_called()

    def test_waiting_for_complete_request_advances_only_brake_hold(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload.update(control_status="waiting_request", control_dt_sec=0.0, enable=False, StopReq=True)
        payload["input_validation"]["request_valid"] = False
        bridge.accept_control(payload)
        self.assertTrue(bridge._control_ack)
        bridge.apply_control.assert_called_once_with(payload)
        self.assertEqual(bridge._sync_fault, "")

    def test_bad_source_obstacle_is_rejected_instead_of_dropped(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.parking_action = 1
        bridge.target_slot = self.make_slot()
        bridge.slot_provider = SimpleNamespace(obstacles={"obs_points": [{"x": 1.0}]})
        with self.assertRaisesRegex(ValueError, "obstacle.y"):
            bridge.plan_request_payload()
        bridge.slot_provider.obstacles = {"obs_lines": [None]}
        with self.assertRaisesRegex(ValueError, "非对象"):
            bridge.plan_request_payload()
        bridge.cfg["apa"]["use_slot_lines_as_boundaries"] = True
        bridge.slot_provider.obstacles = {"obs_lines": None}
        with self.assertRaisesRegex(ValueError, "obs_lines"):
            bridge.plan_request_payload()
        bridge.slot_provider.obstacles = {}
        bridge.slot_provider.slots = {i: self.make_slot({"slot_id": i}) for i in range(22)}
        with self.assertRaisesRegex(ValueError, "obs_vlines.*64"):
            bridge.plan_request_payload()

    def test_missing_slot_corner_latches_brake_before_publishing_request(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.parking_enabled = True
        bridge.parking_action = 1
        bridge.target_slot = self.make_slot()
        bridge.target_slot.pop("slot_pt1_x")
        bridge.slot_provider = SimpleNamespace(obstacles={}, occupancy={})
        bridge.publish = mock.Mock()
        snapshot = SimpleNamespace(frame=10, timestamp=SimpleNamespace(elapsed_seconds=1.0))
        bridge.publish_plan_request_if_needed(snapshot)
        bridge.publish.assert_not_called()
        self.assertIn("slot_pt1_x", bridge.safety_stop.reason)
        self.assertEqual(bridge.ego.applied_control.brake, 1.0)

    def test_nonfinite_measured_pose_is_never_zero_filled(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.ego.transform.location.x = float("nan")
        snapshot = SimpleNamespace(frame=10, timestamp=SimpleNamespace(elapsed_seconds=1.0, delta_seconds=0.02))
        with self.assertRaisesRegex(ValueError, "vehicle_x"):
            bridge.ego_payload(snapshot)
        self.assertIn("invalid_ego_feedback", bridge.safety_stop.reason)
        self.assertEqual(bridge.ego.applied_control.brake, 1.0)

    def test_invalid_time_and_incomplete_control_stop_the_barrier(self):
        for payload in ({"session_id": "session-a", "task_id": 3, "carla_frame_id": 42},
                        dict(self.synchronized_control(), sim_time_sec=1.02),
                        dict(self.synchronized_control(), FrontWheelAngle=float("nan")),
                        dict(self.synchronized_control(), control_status="timing_error")):
            with self.subTest(payload=payload):
                bridge = self.make_sync_bridge()
                bridge.accept_control(payload)
                bridge.apply_control.assert_not_called()
                bridge.apply_sync_hold.assert_called_once()
                self.assertTrue(bridge._sync_fault)
                self.assertFalse(bridge._control_ack)

    def test_spawn_pose_round_trips_to_explicit_and_derived_rear_axle_starts(self):
        bridge = self.make_bridge()
        bridge.cfg = {"apa": {"rear_axle_offset_m": 1.5, "approach_distance_m": 4.5}}
        bridge.carla = SimpleNamespace(
            Location=SimpleNamespace,
            Rotation=SimpleNamespace,
            Transform=lambda location, rotation: SimpleNamespace(location=location, rotation=rotation),
        )
        bridge.sample_ground_z = mock.Mock(return_value=0.0)
        cases = (
            ({"apa_start_rear_x": 4.0, "apa_start_rear_y": 5.0, "apa_start_rear_yaw_rad": -0.7}, (4.0, 5.0, -0.7)),
            ({"goal_x": 0.0, "goal_y": -4.0, "slot_entry_pt_x": 0.0, "slot_entry_pt_y": 0.0}, (0.0, 4.5, -math.pi / 2.0)),
            ({"goal_x": 4.0, "goal_y": 0.0, "goal_yaw": 90.0, "slot_kind": "parallel"}, (-4.5, 0.0, math.pi / 2.0)),
        )
        for slot, expected in cases:
            with self.subTest(slot=slot):
                rear_pose = bridge.rear_axle_pose(bridge.approach_transform(slot))
                for value, expected_value in zip(rear_pose, expected):
                    self.assertAlmostEqual(value, expected_value)

    def test_feedback_request_and_actual_trail_share_rear_axle_coordinates(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.cfg["apa"]["rear_axle_offset_m"] = 2.0
        bridge.parking_action = 1
        bridge.current_apa_gear_pos = 1
        bridge.ego = FakeEgo(self.make_transform())
        bridge.actual_path_xy = deque()
        bridge.measured_longitudinal_accel = mock.Mock(return_value=0.0)
        bridge.target_slot = self.make_slot({"slot_id": 29, "slot_kind": "vertical", "goal_x": 30.0, "goal_y": 40.0})
        bridge.slot_provider = SimpleNamespace(obstacles={}, occupancy={})

        feedback = bridge.ego_payload(SimpleNamespace(frame=42, timestamp=SimpleNamespace(elapsed_seconds=1.0, delta_seconds=0.02)))
        request = bridge.plan_request_payload()
        bridge.record_actual_path()

        for payload in (feedback, request):
            self.assertEqual(payload["vehicle_reference_point"], "rear_axle_center")
            self.assertAlmostEqual(payload["vehicle_x"], 10.0)
            self.assertAlmostEqual(payload["vehicle_y"], 18.0)
            self.assertEqual(payload["VehicleX"], payload["vehicle_x"])
            self.assertEqual(payload["VehicleY"], payload["vehicle_y"])
        self.assertEqual((feedback["carla_actor_x"], feedback["carla_actor_y"]), (10.0, 20.0))
        self.assertEqual(list(bridge.actual_path_xy), [(feedback["vehicle_x"], feedback["vehicle_y"])])
        self.assertEqual((request["TarX"], request["TarY"]), (30.0, 40.0))

    def test_parkin_goal_keeps_clearance_when_envelope_and_tolerance_fit(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.cfg["apa"].update(rear_axle_offset_m=2.0,
                                 vehicle_geometry=dict(bridge.cfg["apa"]["vehicle_geometry"], rear_to_front_m=3.8,
                                                       rear_to_back_m=1.0, width_m=2.0, parkin_back_margin_m=0.10))
        slot = self.make_slot({"goal_reference_point": "slot_center", "goal_x": 2.5,
                               "goal_y": 1.0, "goal_yaw": 0.0,
                               "slot_pt1_y": -0.5, "slot_pt2_y": 2.5,
                               "slot_pt3_y": 2.5, "slot_pt4_y": -0.5})
        original = dict(slot)
        x, y, yaw = bridge.parkin_target(slot)
        self.assertAlmostEqual(x, 3.899)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(abs(yaw), math.pi)
        self.assertEqual(slot, original)
        slot["apa_slot_kind"] = "vh"
        x, y, yaw = bridge.parkin_target(slot)
        self.assertAlmostEqual(x, 1.099)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_parkin_parallel_converts_body_centre_along_the_entry(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.cfg["apa"]["vehicle_geometry"].update(
            rear_to_front_m=3.8, rear_to_back_m=1.0, width_m=2.0)
        slot = self.make_slot({"goal_reference_point": "slot_center", "slot_kind": "parallel",
                               "slot_entry": 2, "goal_x": 2.5, "goal_y": 1.0,
                               "apa_start_rear_yaw_rad": math.pi,
                               "slot_pt1_x": -0.5, "slot_pt1_y": -0.5,
                               "slot_pt2_x": -0.5, "slot_pt2_y": 2.5,
                               "slot_pt3_x": 5.5, "slot_pt3_y": 2.5,
                               "slot_pt4_x": 5.5, "slot_pt4_y": -0.5})
        x, y, yaw = bridge.parkin_target(slot)
        self.assertAlmostEqual(x, 3.9)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(abs(yaw), math.pi)

    def test_port_conflict_never_changes_carla_and_releases_sockets(self):
        bridge = self.make_bridge()
        bridge.stop = mock.Mock()
        bridge.setup_zmq = mock.Mock(side_effect=RuntimeError("port occupied"))
        bridge.connect_carla = mock.Mock()
        bridge.restore_settings = mock.Mock()
        bridge.pub = mock.Mock()
        bridge.sub = mock.Mock()
        bridge.command_sub = mock.Mock()

        with mock.patch.object(self.bridge_module.signal, "signal"):
            with self.assertRaisesRegex(RuntimeError, "port occupied"):
                bridge.run()

        bridge.connect_carla.assert_not_called()
        bridge.restore_settings.assert_called_once()
        for socket in (bridge.pub, bridge.sub, bridge.command_sub):
            socket.close.assert_called_once_with(0)

    def test_failed_carla_startup_restores_settings_and_releases_ports(self):
        bridge = self.make_bridge()
        bridge.stop = mock.Mock()
        bridge.setup_zmq = mock.Mock()
        bridge.connect_carla = mock.Mock(side_effect=RuntimeError("camera failed"))
        bridge.restore_settings = mock.Mock()
        bridge.pub = mock.Mock()
        bridge.sub = mock.Mock()
        bridge.command_sub = mock.Mock()

        with mock.patch.object(self.bridge_module.signal, "signal"):
            with self.assertRaisesRegex(RuntimeError, "camera failed"):
                bridge.run()

        bridge.setup_zmq.assert_called_once()
        bridge.restore_settings.assert_called_once()
        for socket in (bridge.pub, bridge.sub, bridge.command_sub):
            socket.close.assert_called_once_with(0)

    def test_plan_request_contains_complete_native_planner_payload(self):
        bridge = self.make_bridge()
        bridge._planning_query_region = {"bounds": [-10.0, -10.0, 50.0, 50.0]}
        profile = self.bridge_module.default_config()["apa"]["vehicle_geometry"]
        bridge.cfg = {"apa": {"r_min": 5.5, "vehicle_geometry": profile},
                      "control": {"max_abs_steer_rad": self.bridge_module.default_config()["control"]["max_abs_steer_rad"]}}
        bridge.parking_action = 1
        bridge.ego = FakeEgo(self.make_transform())
        bridge.target_slot = self.make_slot({
            "slot_id": 29,
            "apa_slot_kind": "vh",
            "goal_x": 30.0,
            "goal_y": 40.0,
            "goal_yaw": 180.0,
            "slot_entry_pt_x": 25.0,
            "slot_entry_pt_y": 35.0,
            "slot_pt1_x": 20.0,
            "slot_pt1_y": 45.0,
            "slot_pt2_x": 40.0,
            "slot_pt2_y": 45.0,
            "slot_pt3_x": 40.0,
            "slot_pt3_y": 35.0,
            "slot_pt4_x": 20.0,
            "slot_pt4_y": 35.0,
            "slot_entry": 1,
            "parking_type": 1,
            "surrounding_type": 0,
        })
        bridge.slot_provider = SimpleNamespace(
            obstacles={
                "obs_lines": [{"x1": 1.0, "y1": 2.0, "x2": 3.0, "y2": 4.0}],
                "obs_vlines": [],
                "obs_slot_stoppers": [],
                "obs_points": [{"x": 5.0, "y": 6.0}],
                "obs_stoppers": [],
            }, slots={}
        )

        payload = bridge.plan_request_payload()

        required_fields = {
            "ParkingAction",
            "TargetSlotKind",
            "VehicleX",
            "VehicleY",
            "VehicleYaw",
            "TarX",
            "TarY",
            "TarTheta",
            "Rmin",
            "EntrPtX",
            "EntrPtY",
            "SltPt1X",
            "SltPt1Y",
            "SltPt2X",
            "SltPt2Y",
            "SltPt3X",
            "SltPt3Y",
            "SltPt4X",
            "SltPt4Y",
            "SltEntr",
            "PrkType",
            "SurdType",
            "AgrOfst",
            "BoundaryLineNum",
            "BoundaryLineX1",
            "BoundaryLineY1",
            "BoundaryLineX2",
            "BoundaryLineY2",
            "VirtualBoundaryLineNum",
            "VirtualBoundaryP1X",
            "VirtualBoundaryP1Y",
            "VirtualBoundaryP2X",
            "VirtualBoundaryP2Y",
            "SlotTrackStopperNum",
            "SlotTrackStopperX1",
            "SlotTrackStopperY1",
            "SlotTrackStopperX2",
            "SlotTrackStopperY2",
            "BoundarySepPtNum",
            "BoundarySepPtX",
            "BoundarySepPtY",
            "StopperPtNum",
            "StopperPtX",
            "StopperPtY",
            "ShortCutSw",
            "ShortCutThd",
        }
        self.assertTrue(required_fields.issubset(payload))
        self.assertAlmostEqual(payload["r_min"], profile["planning_min_turn_radius_m"])
        self.assertAlmostEqual(payload["Rmin"], payload["r_min"])
        self.assertAlmostEqual(payload["VehicleYaw"], math.pi / 2.0)
        self.assertAlmostEqual(payload["TarTheta"], math.pi)
        self.assertEqual(payload["BoundaryLineNum"], len(payload["BoundaryLineX1"]))
        self.assertEqual(payload["BoundarySepPtNum"], len(payload["BoundarySepPtX"]))

    def test_entry_variants_preserve_geometry_and_reverse_base_direction(self):
        module = self.bridge_module
        for base_kind, variant_kind in (("vertical", "vh"), ("vh", "vertical"),
                                        ("diagonal", "vh"), ("vh", "diagonal")):
            with self.subTest(base_kind=base_kind, variant_kind=variant_kind):
                base = self.make_slot({"apa_slot_kind": base_kind,
                                       "slot_kind": "diagonal" if "diagonal" in (base_kind, variant_kind)
                                       else "vertical"})
                variant = dict(base, slot_id=129, physical_slot_id=29, apa_slot_kind=variant_kind)
                catalog = {"slots": [base, variant]}
                with mock.patch.object(module, "load_json", return_value=catalog):
                    provider = module.ParkingSlots(Path("entry-variants.json"))
                self.assertEqual(list(provider.physical_slots()), [base])
                self.assertEqual(module.physical_slot_id(provider.get(129)), 29)
                for invalid in (dict(variant, apa_slot_kind=base_kind),
                                dict(variant, slot_pt1_x=base["slot_pt1_x"] + 0.1),
                                dict(variant, physical_slot_id=999)):
                    with mock.patch.object(module, "load_json", return_value={"slots": [base, invalid]}):
                        with self.assertRaises(ValueError):
                            module.ParkingSlots(Path("entry-variants.json"))

    def test_parking_actions_share_fresh_neighbor_boundary_access(self):
        target = self.make_slot({"slot_id": 108, "physical_slot_id": 8})
        slots = [self.make_slot({"slot_id": i}) for i in (8, 15, 16, 17, 18, 19)]
        occupancy = {15: {"state": "free", "frame": 42, "static_frame": 42},
                     16: {"state": "occupied", "frame": 42, "static_frame": 42},
                     17: {"state": "unknown", "frame": 42},
                     18: {"state": "free", "frame": 41, "static_frame": 41},
                     19: {"state": "free", "frame": 42, "static_frame": 41}}
        for action in (1, 2):
            with self.subTest(action=action):
                bridge = self.make_bridge()
                bridge.target_slot = target
                bridge.session_id, bridge.task_id, bridge.parking_action = "session", 7, action
                bridge._slot_occupancy_summary = {"frame": 42}
                bridge._planning_query_region = {"bounds": [-10, -10, 50, 50]}
                bridge.slot_provider = SimpleNamespace(physical_slots=lambda: iter(slots),
                                                       occupancy=occupancy)
                lines = bridge.slot_boundary_lines()
                self.assertEqual({row["slot_id"] for row in lines}, {108, 16, 17, 18, 19})
                self.assertEqual(sum(row["is_target"] for row in lines), 3)
                self.assertEqual(bridge._neighbor_boundary_access["confirmed_free"],
                                 [{"slot_id": 15, "frame": 42}])

    def test_neighbor_boundary_access_freezes_only_within_current_task(self):
        bridge = self.make_bridge()
        bridge.target_slot = self.make_slot({"slot_id": 8})
        neighbor = self.make_slot({"slot_id": 15})
        bridge.session_id, bridge.task_id, bridge.parking_action = "session", 6, 1
        bridge._slot_occupancy_summary = {"frame": 42}
        bridge._planning_query_region = {"bounds": [-10, -10, 50, 50]}
        bridge.slot_provider = SimpleNamespace(
            physical_slots=lambda: iter([bridge.target_slot, neighbor]),
            occupancy={15: {"state": "free", "frame": 42, "static_frame": 42}})
        self.assertEqual({row["slot_id"] for row in bridge.slot_boundary_lines()}, {8})
        bridge.slot_provider.occupancy[15] = {"state": "unknown", "frame": 43}
        bridge._slot_occupancy_summary = {"frame": 43}
        self.assertEqual({row["slot_id"] for row in bridge.slot_boundary_lines()}, {8})
        bridge.task_id, bridge.parking_action = 7, 2
        self.assertEqual({row["slot_id"] for row in bridge.slot_boundary_lines()}, {8, 15})
        self.assertEqual(bridge._neighbor_boundary_access["confirmed_free"], [])
        self.assertEqual(bridge._neighbor_boundary_access["frame"], 43)

    def test_park_out_refreshes_occupancy_before_starting_task(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.parking_enabled = bridge.planning_only = False
        bridge.parking_action, bridge.history_revision = 1, 0
        bridge.ego = FakeEgo(self.make_transform())
        snapshot = SimpleNamespace(frame=42)
        bridge.world = SimpleNamespace(get_snapshot=lambda: snapshot)
        bridge._planning_query_region = {"bounds": [-10, -10, 50, 50]}
        bridge._ego_bounding_box = SimpleNamespace(
            get_world_vertices=lambda transform: [SimpleNamespace(x=10, y=20)])
        bridge._offline_parking_map = object()
        bridge.parkout_target = mock.Mock(return_value={"x": 0, "y": 0, "yaw_rad": 0})
        bridge.begin_algorithm_parking = mock.Mock()
        bridge.clear_trajectory_history = mock.Mock()
        bridge.scene_payload = mock.Mock()
        self.assertTrue(bridge.apply_apa_command({"action": "park_out"}))
        bridge.scene_payload.assert_called_once_with(snapshot, force_occupancy=True)
        self.assertEqual(bridge.parking_action, 2)
        self.assertTrue(bridge.parking_enabled)

    def test_park_out_uses_live_ego_pose_and_configured_exit_target(self):
        bridge = self.make_bridge()
        bridge._planning_query_region = {"bounds": [-10.0, -10.0, 50.0, 50.0]}
        profile = self.bridge_module.default_config()["apa"]["vehicle_geometry"]
        bridge.cfg = {"apa": {"rear_axle_offset_m": 2.0, "vehicle_geometry": profile},
                      "control": {"max_abs_steer_rad": math.radians(35)}}
        bridge.parking_action = 2
        bridge.ego = FakeEgo(self.make_transform())
        bridge.target_slot = self.make_slot({
            "slot_id": 29,
            "apa_slot_kind": "vertical",
            "goal_x": 30.0,
            "goal_y": 40.0,
            "goal_yaw": 180.0,
            "parkout_target_x": 5.0,
            "parkout_target_y": 6.0,
            "parkout_target_yaw_rad": -1.2,
        })
        bridge.slot_provider = SimpleNamespace(obstacles={}, slots={}, occupancy={})

        payload = bridge.plan_request_payload()

        self.assertEqual(payload["ParkingAction"], 2)
        self.assertAlmostEqual(payload["r_min"], profile["wheelbase_m"] / math.tan(math.radians(35)))
        self.assertAlmostEqual(payload["VehicleX"], 10.0)
        self.assertAlmostEqual(payload["VehicleY"], 18.0)
        self.assertAlmostEqual(payload["TarX"], 5.0)
        self.assertAlmostEqual(payload["TarY"], 6.0)
        self.assertAlmostEqual(payload["TarTheta"], -1.2)

    def test_park_out_returns_to_custom_start_when_slot_has_default_pose(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.parking_action = 2
        bridge.ego = FakeEgo(self.make_transform())
        bridge.target_slot = self.make_slot({
            "slot_id": 30,
            "apa_start_rear_x": 286.0,
            "apa_start_rear_y": -231.968323,
            "apa_start_rear_yaw_rad": -math.pi / 2.0,
        })
        bridge.slot_provider = SimpleNamespace(obstacles={}, occupancy={})
        bridge.parkout_return_pose = {
            "slot_id": 30, "x": 284.5, "y": -230.5, "yaw_rad": -1.4,
            "source": "park_in_start",
        }

        payload = bridge.plan_request_payload()
        self.assertEqual((payload["TarX"], payload["TarY"], payload["TarTheta"]),
                         (284.5, -230.5, -1.4))

        bridge.target_slot.update({"parkout_target_x": 280.0, "parkout_target_y": -220.0,
                                   "parkout_target_yaw_rad": -1.0})
        payload = bridge.plan_request_payload()
        self.assertEqual((payload["TarX"], payload["TarY"], payload["TarTheta"]),
                         (280.0, -220.0, -1.0))

        for key in ("parkout_target_x", "parkout_target_y", "parkout_target_yaw_rad"):
            del bridge.target_slot[key]

        bridge.parkout_return_pose["slot_id"] = 29
        payload = bridge.plan_request_payload()
        self.assertEqual((payload["TarX"], payload["TarY"], payload["TarTheta"]),
                         (286.0, -231.968323, -math.pi / 2.0))

    def test_apa_action_switch_requires_standstill(self):
        bridge = self.make_bridge()
        bridge.history_revision = 0
        bridge.cfg = {
            "apa": {"action_switch_max_speed_mps": 0.03},
            "control": {"actuation": "pedal"},
        }
        bridge.parking_action = 1
        bridge.target_slot = self.make_slot({
            "parkout_target_x": 5.0,
            "parkout_target_y": 6.0,
            "parkout_target_yaw_rad": -1.2,
        })
        bridge.ego = FakeEgo(self.make_transform())
        bridge.last_plan_result = {"success": True}

        self.assertTrue(bridge.apply_apa_command({"action": "park_out"}))
        self.assertEqual(bridge.parking_action, 2)
        self.assertIsNone(bridge.last_plan_result)

    def make_control_bridge(self):
        bridge = self.make_bridge()
        bridge.history_revision = 0
        bridge.ego = FakeEgo(self.make_transform())
        bridge.carla = SimpleNamespace(VehicleControl=FakeVehicleControl)
        bridge._submit_vehicle_control = lambda command, **_kwargs: bridge.ego.apply_control(command)
        bridge.cfg = {
            "control": {
                "max_abs_target_speed_mps": 0.45,
                "max_abs_steer_rad": 0.6,
                "max_accel_mps2": 1.0,
                "max_brake_mps2": 6.0,
                "max_throttle": 0.35,
                "max_brake": 0.8,
            }
        }
        bridge.ignore_control_until = 0.0
        defaults = self.bridge_module.default_config()
        defaults["control"].update(bridge.cfg["control"])
        bridge.cfg = defaults
        bridge.current_apa_gear_pos = 1
        bridge.last_control_id = 0
        return bridge

    def reverse_command(self, gear_change_allowed):
        return {
            "command_id": 1,
            "actuator": {"owner": "C:apaMocoActuate:v2", "throttle": 0.17 if gear_change_allowed else 0.0,
                         "brake": 0.0 if gear_change_allowed else 1.0,
                         "hand_brake": False, "launch_hold": False, "gear_pos": 2 if gear_change_allowed else 1},
            "ControlEnabled": True,
            "StopReq": False,
            "TargetSpeed": -0.2,
            "TargetAcceleration": -0.5,
            "FrontWheelAngle": 0.1,
            "TargetGearReq": 2,
            "DirectionSign": -1,
            "GearChangeAllowed": gear_change_allowed,
        }

    def test_gear_change_waits_for_algorithm_interlock(self):
        bridge = self.make_control_bridge()

        bridge.apply_control(self.reverse_command(False))

        self.assertEqual(bridge.current_apa_gear_pos, 1)
        self.assertEqual(bridge.ego.applied_control.brake, 1.0)
        self.assertFalse(bridge.ego.applied_control.reverse)

    def test_normal_braking_keeps_acceleration_mapping_and_steering(self):
        for gear, acceleration in ((1, -1.2), (2, 1.2)):
            with self.subTest(gear=gear):
                bridge = self.make_control_bridge()
                bridge.current_apa_gear_pos = gear
                payload = self.reverse_command(True)
                payload.update(TargetGearReq=gear, DirectionSign=1 if gear == 1 else -1,
                               TargetAcceleration=acceleration, TargetSpeed=0.0)
                payload["actuator"].update(throttle=0, brake=0.2, gear_pos=gear)
                bridge.apply_control(payload)
                self.assertAlmostEqual(bridge.ego.applied_control.brake, 0.2)
                self.assertEqual(bridge.ego.applied_control.throttle, 0.0)
                self.assertNotEqual(bridge.ego.applied_control.steer, 0.0)
                payload["StopReq"] = True
                payload["actuator"].update(throttle=0, brake=1)
                bridge.apply_control(payload)
                self.assertEqual(bridge.ego.applied_control.brake, 1.0)

    def test_tracking_fault_acknowledges_feedback_and_applies_brake_hold(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        payload.update(control_status="tracking_fault", enable=False, StopReq=True, CtrlPhase=9)
        bridge.accept_control(payload)
        bridge.apply_control.assert_called_once_with(payload)
        self.assertTrue(bridge._control_ack)
        self.assertEqual(bridge._sync_fault, "")

    def test_gear_feedback_uses_carla_applied_control(self):
        bridge = self.make_control_bridge()
        bridge.cfg["carla"] = {"fixed_delta_seconds": 0.02}
        bridge.cfg["apa"]["rear_axle_offset_m"] = 1.38421
        bridge.parking_action = 1
        bridge.parking_enabled = True
        bridge.ego.applied_control = SimpleNamespace(gear=-1, steer=0.0)
        snapshot = SimpleNamespace(frame=1, timestamp=SimpleNamespace(elapsed_seconds=1.0, delta_seconds=0.02))
        feedback = bridge.ego_payload(snapshot)
        self.assertEqual(feedback["gear_pos"], 2)
        self.assertEqual(feedback["gear_feedback_source"], "carla_applied_control")

    def test_allowed_standstill_gear_change_applies_reverse_drive(self):
        bridge = self.make_control_bridge()

        bridge.apply_control(self.reverse_command(True))

        self.assertEqual(bridge.current_apa_gear_pos, 2)
        self.assertTrue(bridge.ego.applied_control.reverse)
        self.assertEqual(bridge.ego.applied_control.gear, -1)
        self.assertGreater(bridge.ego.applied_control.throttle, 0.0)
        self.assertAlmostEqual(bridge.ego.applied_control.steer, 0.1 / 0.6, places=3)

    def test_steer_command_is_normalized_and_signed(self):
        command = self.bridge_module.carla_steer_command
        self.assertAlmostEqual(command(0.3, 0.6), 0.5)
        self.assertAlmostEqual(command(-0.3, 0.6), -0.5)
        self.assertEqual(command(2.0, 0.6), 1.0)
        self.assertEqual(command(-2.0, 0.6), -1.0)

    def test_manual_steer_uses_the_same_carla_command(self):
        steer = self.bridge_module.manual_steer_command({"steer_rad": -0.6}, 0.6)
        self.assertAlmostEqual(steer, -1.0)
        self.assertAlmostEqual(self.bridge_module.manual_steer_command({"steer": 0.4}, 0.6), 0.4)

    def test_algorithm_control_is_ignored_until_park_command(self):
        bridge = self.make_control_bridge()
        bridge.parking_enabled = False
        bridge.cfg["apa"] = {"action_switch_max_speed_mps": 0.03}
        bridge.target_slot = self.make_slot({
            "parkout_target_x": 5.0,
            "parkout_target_y": 6.0,
            "parkout_target_yaw_rad": -1.2,
        })
        bridge.parking_action = 1
        bridge.apply_control(self.reverse_command(True))
        self.assertIsNone(bridge.ego.applied_control)
        self.assertTrue(bridge.apply_apa_command({"action": "park_out"}))
        bridge.apply_control(self.reverse_command(True))
        self.assertIsNotNone(bridge.ego.applied_control)
        self.assertTrue(bridge.ego.applied_control.reverse)

    def test_final_c_pedals_are_copied_even_when_speed_or_acceleration_differs(self):
        bridge = self.make_control_bridge()
        bridge.current_speed = lambda: -1.2
        bridge.ego_slope_deg = mock.Mock(side_effect=AssertionError("no Bridge acceleration loop"))
        payload = self.reverse_command(True)
        payload.update(TargetSpeed=-0.2, TargetAcceleration=-1.5)
        bridge.apply_control(payload)
        self.assertEqual(bridge.ego.applied_control.throttle, 0.17)
        self.assertEqual(bridge.ego.applied_control.brake, 0)
        self.assertEqual(bridge.ego.applied_control.gear, -1)
        bridge.ego_slope_deg.assert_not_called()

    def test_old_acceleration_only_command_cannot_drive(self):
        bridge = self.make_sync_bridge()
        payload = self.synchronized_control()
        del payload["actuator"]
        bridge.accept_control(payload)
        bridge.apply_control.assert_not_called()
        self.assertTrue(bridge.safety_stop.reason)

    def test_control_submission_waits_for_ack_without_ticking(self):
        bridge = self.make_control_bridge()
        del bridge._submit_vehicle_control
        bridge.ego.id = 7
        bridge.carla.command = SimpleNamespace(ApplyVehicleControl=mock.Mock(return_value="batch_command"))
        bridge.client = SimpleNamespace(apply_batch_sync=mock.Mock(return_value=[SimpleNamespace(error="")]))
        control = object()
        bridge._submit_vehicle_control(control)
        bridge.carla.command.ApplyVehicleControl.assert_called_once_with(7, control)
        bridge.client.apply_batch_sync.assert_called_once_with(["batch_command"], False)
        self.assertIsNone(bridge.ego.applied_control)

    def test_control_submission_rejects_missing_or_failed_ack(self):
        bridge = self.make_control_bridge()
        del bridge._submit_vehicle_control
        bridge.ego.id = 7
        bridge.carla.command = SimpleNamespace(ApplyVehicleControl=lambda *_: "command")
        for responses in ([], [SimpleNamespace(error="actor missing")]):
            bridge.client = SimpleNamespace(apply_batch_sync=lambda *_: responses)
            with self.assertRaisesRegex(RuntimeError, "acknowledge"):
                bridge._submit_vehicle_control(object())

    def test_drivetrain_calibration_normalizes_wheel_lock_even_when_already_close(self):
        bridge = self.make_control_bridge()
        bridge.cfg["apa"]["drivetrain_clutch_strength"] = 1.0
        wheel = SimpleNamespace(max_steer_angle=math.degrees(bridge.cfg["control"]["max_abs_steer_rad"]) - 4.e-6)
        physics = SimpleNamespace(wheels=[wheel], forward_gears=[object()],
            torque_curve=[object(), object()], steering_curve=[object()], clutch_strength=10.0)
        bridge.ego.get_physics_control = lambda: physics
        bridge.ego.apply_physics_control = mock.Mock()
        bridge.apply_front_wheel_lock()
        self.assertEqual(physics.clutch_strength, 1.0)
        self.assertEqual(wheel.max_steer_angle, math.degrees(bridge.cfg["control"]["max_abs_steer_rad"]))
        bridge.ego.apply_physics_control.assert_called_once_with(physics)

    def test_invalid_drivetrain_calibration_is_rejected(self):
        bridge = self.make_control_bridge()
        bridge.cfg = self.bridge_module.default_config()
        for value in (0, -1, float("nan"), True, 11):
            bridge.cfg["apa"]["drivetrain_clutch_strength"] = value
            with self.assertRaisesRegex(ValueError, "clutch_strength"):
                bridge.validate_control_config()

    def test_pedal_recorder_uses_control_from_the_completed_physics_tick(self):
        bridge = self.make_control_bridge()
        bridge.cfg["carla"] = {"sync_mode": True}
        bridge.ego.id = 7
        first, last = SimpleNamespace(frame=10), SimpleNamespace(frame=11)
        previous_control, applied_control = object(), object()
        state = {"snapshot": first, "control": previous_control}
        bridge.ego.get_control = lambda: state["control"]
        def tick(_timeout):
            state.update(snapshot=last, control=applied_control)
            return 11
        bridge.world = SimpleNamespace(get_snapshot=lambda: state["snapshot"], tick=tick)
        bridge.calibration_recorder = SimpleNamespace(record=mock.Mock())
        bridge.tick_world()
        bridge.calibration_recorder.record.assert_called_once_with(first, last, 7, applied_control, False)

    def test_ego_box_is_cached_per_actor_and_reloaded_after_respawn(self):
        bridge = self.make_bridge()
        reads = []
        box = SimpleNamespace(get_world_vertices=lambda _: [SimpleNamespace(z=0), SimpleNamespace(z=1)])
        class Ego:
            id = 7
            @property
            def bounding_box(self):
                reads.append(self.id)
                return box
        bridge.ego = Ego()
        bridge.world = SimpleNamespace(get_actors=lambda: [])
        bridge.rear_axle_pose = lambda _: (0, 0, 0)
        snapshot = SimpleNamespace(frame=10, find=lambda _: SimpleNamespace(get_transform=lambda: object()))
        method = self.bridge_module.ApaCarlaBridge.scene_payload
        method(bridge, snapshot)
        method(bridge, snapshot)
        self.assertEqual(reads, [7])
        bridge.ego.id = 8
        method(bridge, snapshot)
        self.assertEqual(reads, [7, 8])

    def test_headers_reuse_the_loaded_map_name_without_map_rpc(self):
        bridge = self.make_bridge()
        bridge.cfg = self.bridge_module.default_config()
        bridge.world = SimpleNamespace(get_map=mock.Mock(side_effect=RuntimeError("unneeded map RPC")))
        bridge._map_name = "Carla/Maps/Town04"
        bridge.sequence = {}
        for _ in range(2):
            self.assertEqual(bridge.make_header("ego")["map_name"], bridge._map_name)
        bridge.world.get_map.assert_not_called()

if __name__ == "__main__":
    unittest.main()
