import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


BRIDGE_DIR = Path(__file__).resolve().parents[1]


def load_bridge_module():
    module_name = "carla_bridge_acceleration_pedal_controller_test"
    spec = importlib.util.spec_from_file_location(
        module_name, BRIDGE_DIR / "carla_bridge.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class AccelerationPedalControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_module = load_bridge_module()

    def make_controller(self, **overrides):
        config = {
            "accel_kp": 0.0,
            "accel_ki": 0.0,
            "accel_integral_limit": 10.0,
            "accel_filter_tau_sec": 0.0,
            "rolling_resistance_coefficient": 0.01,
            "drag_reference_area_m2": 2.37,
            "air_density_kgpm3": 1.184,
            "gravity_mps2": 9.81,
            "accel_feedforward_limit_mps2": 2.0,
        }
        config.update(overrides)
        return self.bridge_module.AccelerationPedalController(config)

    def update(self, controller, **overrides):
        values = {
            "target_accel_mps2": 0.0,
            "measured_accel_mps2": 0.0,
            "speed_mps": 10.0,
            "forward_z": 0.0,
            "mass_kg": 1500.0,
            "drag_coefficient": 0.3,
            "dt_sec": 0.05,
            "max_accel_mps2": 1.5,
            "max_brake_mps2": 6.0,
            "max_throttle": 1.0,
            "max_brake": 1.0,
        }
        values.update(overrides)
        return controller.update(**values)

    def test_resistance_feedforward_supplies_cruise_throttle(self):
        controller = self.make_controller()

        output = self.update(controller)

        expected_rolling = 0.01 * 9.81
        expected_drag = 0.5 * 1.184 * 0.3 * 2.37 * 100.0 / 1500.0
        expected_feedforward = expected_rolling + expected_drag
        self.assertAlmostEqual(
            output.feedforward_accel_mps2, expected_feedforward, places=6
        )
        self.assertAlmostEqual(
            output.throttle, expected_feedforward / 1.5, places=6
        )
        self.assertEqual(output.brake, 0.0)

    def test_integral_retains_hold_throttle_after_acceleration_error_clears(self):
        controller = self.make_controller(
            accel_ki=0.5,
            rolling_resistance_coefficient=0.0,
            drag_reference_area_m2=0.0,
        )

        for _ in range(10):
            self.update(
                controller,
                measured_accel_mps2=-0.5,
                speed_mps=0.0,
                dt_sec=0.1,
            )
        output = self.update(
            controller,
            measured_accel_mps2=0.0,
            speed_mps=0.0,
            dt_sec=0.1,
        )

        self.assertAlmostEqual(output.integral_error_mps, 0.5)
        self.assertAlmostEqual(output.feedback_accel_mps2, 0.25)
        self.assertGreater(output.throttle, 0.0)
        self.assertEqual(output.brake, 0.0)

    def test_pi_removes_steady_error_when_resistance_model_is_imperfect(self):
        controller = self.make_controller(accel_kp=0.2, accel_ki=0.25)
        measured_accel = 0.0
        actual_resistance_accel = 0.25

        for _ in range(600):
            output = self.update(
                controller,
                measured_accel_mps2=measured_accel,
                speed_mps=15.0,
                dt_sec=0.05,
            )
            measured_accel = (
                output.throttle * 1.5
                - output.brake * 6.0
                - actual_resistance_accel
            )

        self.assertAlmostEqual(measured_accel, 0.0, delta=0.002)
        self.assertAlmostEqual(output.throttle, actual_resistance_accel / 1.5, delta=0.002)

    def test_positive_saturation_does_not_wind_up_integral(self):
        controller = self.make_controller(
            accel_kp=1.0,
            accel_ki=1.0,
            rolling_resistance_coefficient=0.0,
            drag_reference_area_m2=0.0,
        )

        for _ in range(20):
            output = self.update(
                controller,
                target_accel_mps2=2.0,
                measured_accel_mps2=0.0,
                speed_mps=0.0,
                dt_sec=0.1,
                max_throttle=0.2,
            )

        self.assertAlmostEqual(output.throttle, 0.2)
        self.assertTrue(output.saturated)
        self.assertAlmostEqual(output.integral_error_mps, 0.0)

    def test_negative_request_uses_brake_and_reset_clears_memory(self):
        controller = self.make_controller(
            accel_ki=0.5,
            rolling_resistance_coefficient=0.0,
            drag_reference_area_m2=0.0,
        )
        self.update(
            controller,
            measured_accel_mps2=-0.5,
            speed_mps=0.0,
            dt_sec=0.1,
        )

        output = self.update(
            controller,
            target_accel_mps2=-2.0,
            measured_accel_mps2=-2.0,
            speed_mps=0.0,
        )
        controller.reset()

        self.assertEqual(output.throttle, 0.0)
        self.assertGreater(output.brake, 0.0)
        self.assertAlmostEqual(controller.integral_error_mps, 0.0)


class FakeEgo:
    def __init__(self, speed_mps=10.0, acceleration_mps2=0.0):
        self.id = 42
        self.speed_mps = speed_mps
        self.acceleration_mps2 = acceleration_mps2

    def get_velocity(self):
        return SimpleNamespace(x=self.speed_mps, y=0.0, z=0.0)

    def get_acceleration(self):
        return SimpleNamespace(x=self.acceleration_mps2, y=0.0, z=0.0)

    def get_transform(self):
        return SimpleNamespace(
            get_forward_vector=lambda: SimpleNamespace(x=1.0, y=0.0, z=0.0)
        )

    def get_physics_control(self):
        return SimpleNamespace(mass=1500.0, drag_coefficient=0.3)


class AccelerationPedalBridgeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_module = load_bridge_module()

    def make_bridge(self):
        bridge = object.__new__(self.bridge_module.CarlaGaasdBridge)
        bridge.cfg = self.bridge_module.default_config()
        bridge.cfg["control"].update(
            {
                "prefer_speed_target": False,
                "max_accel_mps2": 1.5,
                "max_brake_mps2": 6.0,
                "max_throttle": 1.0,
                "max_brake": 1.0,
                "accel_kp": 0.0,
                "accel_ki": 0.0,
                "rolling_resistance_coefficient": 0.01,
                "drag_reference_area_m2": 2.37,
            }
        )
        bridge.ego = FakeEgo()
        bridge.speed_pid_prev_error = 0.0
        bridge.speed_pid_integral_error = 0.0
        bridge.speed_pid_prev_time = 0.0
        bridge.acceleration_pedal_controller = (
            self.bridge_module.AccelerationPedalController(bridge.cfg["control"])
        )
        bridge.ego_physics_actor_id = None
        bridge.ego_mass_kg = 1500.0
        bridge.ego_drag_coefficient = 0.3
        return bridge

    def test_acceleration_target_uses_resistance_compensating_closed_loop(self):
        bridge = self.make_bridge()

        throttle, brake = bridge.longitudinal_to_throttle_brake(
            {"target_accel_mps2": 0.0}, {}
        )

        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        self.assertGreater(
            bridge.acceleration_pedal_controller.last_output.feedforward_accel_mps2,
            0.0,
        )

    def test_speed_target_keeps_legacy_speed_pid_mapping(self):
        bridge = self.make_bridge()
        bridge.cfg["control"]["prefer_speed_target"] = True
        bridge.speed_pid_accel = lambda _target, _current: 0.6
        bridge.acceleration_pedal_controller.integral_error_mps = 2.0

        throttle, brake = bridge.longitudinal_to_throttle_brake(
            {"target_speed_mps": 12.0}, {"max_speed_mps": 20.0}
        )

        self.assertAlmostEqual(throttle, 0.4)
        self.assertEqual(brake, 0.0)
        self.assertEqual(
            bridge.acceleration_pedal_controller.last_output,
            self.bridge_module.AccelerationPedalOutput(),
        )
        self.assertEqual(
            bridge.acceleration_pedal_controller.integral_error_mps,
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
