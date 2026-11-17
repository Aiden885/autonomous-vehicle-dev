import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


BRIDGE_DIR = Path(__file__).resolve().parents[1]


def load_bridge_module():
    module_name = "carla_bridge_acc_manual_control_test"
    spec = importlib.util.spec_from_file_location(
        module_name, BRIDGE_DIR / "carla_bridge.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeVehicleControl:
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle = throttle
        self.brake = brake
        self.steer = steer


class FakeEgo:
    def __init__(self):
        self.applied_control = None

    def get_velocity(self):
        return SimpleNamespace(x=0.0, y=0.0, z=0.0)

    def apply_control(self, control):
        self.applied_control = control


class AccManualControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_module = load_bridge_module()

    def make_bridge(self, command_type=0, command_active=False):
        bridge = object.__new__(self.bridge_module.CarlaGaasdBridge)
        bridge.ego = FakeEgo()
        bridge.carla = SimpleNamespace(VehicleControl=FakeVehicleControl)
        bridge.cfg = {
            "control": {
                "max_abs_steer_rad": 0.6,
                "max_throttle": 0.4,
                "max_brake": 0.7,
                "lane_keep_enabled": True,
                "lane_keep_override_zero_steer": True,
                "lane_keep_zero_steer_epsilon_rad": 1.0e-6,
            }
        }
        bridge.last_command = self.bridge_module.LastCommand()
        bridge.driver_command_level = command_type
        bridge.driver_command_level_active = command_active
        bridge.control_timeout_applied = False
        bridge.acceleration_pedal_controller = (
            self.bridge_module.AccelerationPedalController(bridge.cfg["control"])
        )
        bridge.compute_lane_keep_steer_norm = lambda: 0.2
        bridge.longitudinal_to_throttle_brake = lambda _target, _safety: (0.3, 0.0)
        bridge.reset_speed_pid = lambda: None
        return bridge

    def apply_command(self, bridge, enable):
        bridge.apply_control_message(
            {
                "payload": {
                    "enable": enable,
                    "target": {"target_accel_mps2": 0.5},
                }
            }
        )
        return bridge.ego.applied_control

    def test_w_applies_driver_throttle_while_acc_is_in_standby(self):
        bridge = self.make_bridge(command_type=5, command_active=True)

        control = self.apply_command(bridge, enable=False)

        self.assertAlmostEqual(control.throttle, 0.4)
        self.assertAlmostEqual(control.brake, 0.0)
        self.assertAlmostEqual(control.steer, 0.2)

    def test_acc_standby_releases_longitudinal_control_without_braking(self):
        bridge = self.make_bridge()
        bridge.acceleration_pedal_controller.integral_error_mps = 2.0

        control = self.apply_command(bridge, enable=False)

        self.assertAlmostEqual(control.throttle, 0.0)
        self.assertAlmostEqual(control.brake, 0.0)
        self.assertAlmostEqual(control.steer, 0.2)
        self.assertAlmostEqual(
            bridge.acceleration_pedal_controller.integral_error_mps, 0.0
        )

    def test_s_overrides_active_acc_with_driver_brake(self):
        bridge = self.make_bridge(command_type=6, command_active=True)

        control = self.apply_command(bridge, enable=True)

        self.assertAlmostEqual(control.throttle, 0.0)
        self.assertAlmostEqual(control.brake, 0.7)
        self.assertAlmostEqual(control.steer, 0.2)


if __name__ == "__main__":
    unittest.main()
