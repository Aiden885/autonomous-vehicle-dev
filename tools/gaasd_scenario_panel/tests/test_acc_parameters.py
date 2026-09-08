import importlib.util
import math
import sys
import unittest
from pathlib import Path


PANEL_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PANEL_DIR.parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class AccParameterValidationTest(unittest.TestCase):
    SCENARIO_PATH = (
        REPO_ROOT
        / "scenarios"
        / "newaccpro4_accel_codegen_carla_20260824"
        / "scenario.yaml"
    )
    RUN_SCRIPT = SCENARIO_PATH.with_name("run.sh")
    EXPECTED_PARAMETERS = {
        "lead_speed_mps": "ACC_LEAD_SPEED_MPS",
        "lead_distance_m": "ACC_LEAD_DISTANCE_M",
        "lead_step_speed_mps": "ACC_LEAD_STEP_SPEED_MPS",
        "lead_step_time_sec": "ACC_LEAD_STEP_TIME_SEC",
        "lead_max_throttle": "ACC_LEAD_MAX_THROTTLE",
        "record_duration_sec": "ACC_RECORD_DURATION_SEC",
        "boost_speed_mps": "BOOST_SPEED_MPS",
        "boost_duration_sec": "BOOST_DURATION_SEC",
        "max_accel_mps2": "ACC_MAX_ACCEL_MPS2",
        "max_brake_mps2": "ACC_MAX_BRAKE_MPS2",
        "max_throttle": "ACC_MAX_THROTTLE",
        "max_brake": "ACC_MAX_BRAKE",
    }

    def setUp(self):
        self.scenario = panel_app.load_yaml_file(self.SCENARIO_PATH)

    def test_acc_scenario_exposes_scene_and_actuator_parameters(self):
        schema = {
            item["key"]: item["env"]
            for item in panel_app.parameter_schema(self.scenario)
        }

        self.assertEqual(schema, self.EXPECTED_PARAMETERS)

    def test_acc_run_script_consumes_every_parameter(self):
        run_script = self.RUN_SCRIPT.read_text(encoding="utf-8")

        for environment_name in self.EXPECTED_PARAMETERS.values():
            self.assertIn(environment_name, run_script)

    def test_acc_parameter_ranges_accept_a_deceleration_step(self):
        valid, values = panel_app.validate_parameter_values(
            self.scenario,
            {
                "lead_speed_mps": 8.0,
                "lead_step_speed_mps": 2.0,
                "lead_step_time_sec": 20.0,
                "lead_distance_m": 35.0,
            },
        )

        self.assertTrue(valid, values)
        self.assertEqual(values["lead_step_speed_mps"], 2.0)


class LeadSpeedScheduleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = load_module(
            "lead_waypoint_pid_controller_test",
            REPO_ROOT / "tools" / "carla_bridge" / "lead-waypoint-pid-controller.py",
        )

    def test_step_is_disabled_when_step_time_is_zero(self):
        self.assertEqual(
            self.controller.target_speed_at_elapsed(30.0, 3.0, 7.0, 0.0),
            3.0,
        )

    def test_target_speed_changes_at_configured_time(self):
        self.assertEqual(
            self.controller.target_speed_at_elapsed(9.9, 3.0, 7.0, 10.0),
            3.0,
        )
        self.assertEqual(
            self.controller.target_speed_at_elapsed(10.0, 3.0, 7.0, 10.0),
            7.0,
        )


class AccResultSummaryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recorder = load_module(
            "record_acc_run_test",
            REPO_ROOT / "tools" / "pangu_acc_closed_loop" / "record_acc_run.py",
        )

    def test_summary_reports_tracking_error_saturation_and_pedal_switches(self):
        samples = [
            {
                "wall_time_sec": 0.0,
                "ego_speed_mps": 1.0,
                "lead_speed_mps": 2.0,
                "target_accel_mps2": 1.5,
                "throttle": 0.3,
                "brake": 0.0,
                "clearance_m": 30.0,
            },
            {
                "wall_time_sec": 1.0,
                "ego_speed_mps": 1.8,
                "lead_speed_mps": 2.0,
                "target_accel_mps2": 0.2,
                "throttle": 0.1,
                "brake": 0.0,
                "clearance_m": 29.0,
            },
            {
                "wall_time_sec": 2.0,
                "ego_speed_mps": 2.0,
                "lead_speed_mps": 2.0,
                "target_accel_mps2": -6.0,
                "throttle": 0.0,
                "brake": 0.4,
                "clearance_m": 28.0,
            },
        ]

        summary = self.recorder.summarize(
            samples,
            100.0,
            102.0,
            analysis_start_sec=0.0,
            max_accel_mps2=1.5,
            max_brake_mps2=6.0,
            speed_tolerance_mps=0.25,
        )

        expected_rms = math.sqrt((1.0 + 0.04 + 0.0) / 3.0)
        self.assertAlmostEqual(summary["speed_error_rms_mps"], expected_rms)
        self.assertEqual(summary["speed_error_max_abs_mps"], 1.0)
        self.assertAlmostEqual(summary["accel_saturation_ratio"], 2.0 / 3.0)
        self.assertEqual(summary["throttle_brake_switch_count"], 1)
        self.assertEqual(summary["speed_settling_time_sec"], 1.0)


class AccParameterUiTest(unittest.TestCase):
    def test_parameter_card_is_profile_independent_and_acc_results_are_available(self):
        html = (PANEL_DIR / "templates" / "index.html").read_text(encoding="utf-8")
        javascript = (PANEL_DIR / "static" / "app.js").read_text(encoding="utf-8")

        self.assertIn('id="scenarioParameterCard"', html)
        self.assertIn('id="scenarioParameterTitle"', html)
        self.assertIn('id="accResultBtn"', html)
        self.assertIn("scenarioParameters", javascript)
        self.assertIn("loadScenarioParameters", javascript)
        self.assertIn("readAccResult", javascript)


class AccelerationLimitPrecedenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_module(
            "carla_bridge_acc_limit_test",
            REPO_ROOT / "tools" / "carla_bridge" / "carla_bridge.py",
        )

    def test_scenario_can_explicitly_override_message_acceleration_limits(self):
        limits = self.bridge.resolve_acceleration_limits(
            {
                "override_command_accel_limits": True,
                "max_accel_mps2": 2.0,
                "max_brake_mps2": 7.0,
            },
            {"max_accel_mps2": 1.0, "max_decel_mps2": 3.0},
        )

        self.assertEqual(limits, (2.0, 7.0))

    def test_message_limits_remain_default_precedence(self):
        limits = self.bridge.resolve_acceleration_limits(
            {
                "override_command_accel_limits": False,
                "max_accel_mps2": 2.0,
                "max_brake_mps2": 7.0,
            },
            {"max_accel_mps2": 1.0, "max_decel_mps2": 3.0},
        )

        self.assertEqual(limits, (1.0, 3.0))


if __name__ == "__main__":
    unittest.main()
