import sys
import unittest
from pathlib import Path


PANEL_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PANEL_DIR.parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


class LksParameterValidationTest(unittest.TestCase):
    SCENARIO_PATH = (
        REPO_ROOT / "scenarios" / "lks3_codegen_carla_20260817" / "scenario.yaml"
    )
    RUNTIME_PARAMETER_SOURCE = (
        REPO_ROOT
        / "scenarios"
        / "lks3_codegen_carla_20260817"
        / "runtime_project_src"
        / "modules"
        / "module_empty"
        / "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d"
        / "module_lib"
        / "lks_runtime_parameters.cpp"
    )
    RUN_SCRIPT = (
        REPO_ROOT / "scenarios" / "lks3_codegen_carla_20260817" / "run.sh"
    )
    EXPECTED_PARAMETERS = {
        "target_speed_mps": "LKS_TARGET_SPEED_MPS",
        "initial_lateral_offset_m": "LKS_INITIAL_OFFSET_M",
        "record_duration_sec": "LKS_RECORD_DURATION_SEC",
        "boost_speed_mps": "LKS_BOOST_SPEED_MPS",
        "boost_duration_sec": "LKS_BOOST_DURATION_SEC",
        "l0": "LKS_PARAM_L0",
        "rt": "LKS_PARAM_RT",
        "r_alpha": "LKS_PARAM_R_ALPHA",
        "curvature_threshold": "LKS_PARAM_CURVATURE_THRESHOLD",
        "near_preview_distance": "LKS_PARAM_NEAR_PREVIEW_DISTANCE",
        "w1": "LKS_PARAM_W1",
        "w2": "LKS_PARAM_W2",
        "w3": "LKS_PARAM_W3",
        "kp": "LKS_PARAM_KP",
        "steer_scale": "LKS_PARAM_STEER_SCALE",
        "ay_max": "LKS_PARAM_AY_MAX",
        "v_min": "LKS_PARAM_V_MIN",
        "driver_steer_threshold": "LKS_PARAM_DRIVER_STEER_THRESHOLD",
        "wheel_base": "LKS_PARAM_WHEEL_BASE",
        "front_wheel_max_rad": "LKS_PARAM_FRONT_WHEEL_MAX_RAD",
    }

    def setUp(self):
        self.scenario = panel_app.load_yaml_file(self.SCENARIO_PATH)

    def test_lks3_accepts_one_hundred_forty_kilometers_per_hour(self):
        valid, values = panel_app.validate_parameter_values(
            self.scenario,
            {"target_speed_mps": 38.9},
        )

        self.assertTrue(valid, values)
        self.assertEqual(values["target_speed_mps"], 38.9)

    def test_lks3_exposes_all_scene_and_algorithm_parameters(self):
        schema = {
            item["key"]: item["env"]
            for item in panel_app.parameter_schema(self.scenario)
        }

        self.assertEqual(schema, self.EXPECTED_PARAMETERS)

    def test_lks3_rejects_preview_weights_that_do_not_sum_to_one(self):
        valid, message = panel_app.validate_parameter_values(
            self.scenario,
            {"w1": 0.4, "w2": 0.4, "w3": 0.4},
        )

        self.assertFalse(valid)
        self.assertIn("preview_weights", message)

    def test_lks3_accepts_vehicle_calibration_parameters(self):
        valid, values = panel_app.validate_parameter_values(
            self.scenario,
            {"wheel_base": 2.8, "front_wheel_max_rad": 0.55},
        )

        self.assertTrue(valid, values)
        self.assertEqual(values["wheel_base"], 2.8)
        self.assertEqual(values["front_wheel_max_rad"], 0.55)

    def test_lks3_run_script_forwards_every_runtime_parameter(self):
        run_script = self.RUN_SCRIPT.read_text(encoding="utf-8")

        for environment_name in self.EXPECTED_PARAMETERS.values():
            self.assertIn(environment_name, run_script)

    def test_lks3_runtime_loader_reads_every_algorithm_parameter(self):
        self.assertTrue(self.RUNTIME_PARAMETER_SOURCE.exists())
        source = self.RUNTIME_PARAMETER_SOURCE.read_text(encoding="utf-8")

        for environment_name in list(self.EXPECTED_PARAMETERS.values())[5:]:
            self.assertIn(environment_name, source)


if __name__ == "__main__":
    unittest.main()
