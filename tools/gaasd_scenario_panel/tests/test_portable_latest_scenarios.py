import unittest
from pathlib import Path

import yaml


ROOT_DIR = Path(__file__).resolve().parents[3]
ACC_SCENARIO_DIR = ROOT_DIR / "scenarios/newaccpro4_accel_codegen_carla_20260824"
LKS_SCENARIO_DIR = ROOT_DIR / "scenarios/lks3_codegen_carla_20260817"
BUILD_SCRIPT = ROOT_DIR / "tools/pangu_generated_closed_loop/build_generated_project.sh"


class PortableLatestScenariosTest(unittest.TestCase):
    def test_latest_scenarios_define_maps_and_editable_parameters(self):
        expected = (
            (ACC_SCENARIO_DIR, "Town01", "lead_speed_mps"),
            (LKS_SCENARIO_DIR, "Town04", "target_speed_mps"),
        )

        for scenario_dir, map_name, parameter_key in expected:
            data = yaml.safe_load(
                (scenario_dir / "scenario.yaml").read_text(encoding="utf-8")
            )
            keys = {item["key"] for item in data["parameters"]}
            self.assertEqual(data["carla_scene"]["map"], map_name)
            self.assertIn(parameter_key, keys)

    def test_acc_scenario_carries_verified_runtime_source(self):
        module_root = (
            ACC_SCENARIO_DIR
            / "runtime_project_src/modules/module_empty"
            / "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43"
        )

        bridge_source = (module_root / "module_lib/acc_zmq_bridge.cpp").read_text(
            encoding="utf-8"
        )
        output_source = (
            module_root
            / "src/AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.cc"
        ).read_text(encoding="utf-8")
        self.assertIn("target_accel_mps2", bridge_source)
        self.assertIn("PublishAccelerationControl", output_source)

    def test_lks_scenario_carries_runtime_parameter_overlay(self):
        module_root = (
            LKS_SCENARIO_DIR
            / "runtime_project_src/modules/module_empty"
            / "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d"
        )

        runtime_source = (module_root / "module_lib/lks_runtime_parameters.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("LKS_PARAM_FRONT_WHEEL_MAX_RAD", runtime_source)

    def test_build_and_run_scripts_use_scenario_source_override(self):
        build_script = BUILD_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("PANGU_PROJECT_SOURCE", build_script)

        for scenario_dir in (ACC_SCENARIO_DIR, LKS_SCENARIO_DIR):
            run_script = (scenario_dir / "run.sh").read_text(encoding="utf-8")
            self.assertIn("runtime_project_src", run_script)
            self.assertIn("PANGU_PROJECT_SOURCE", run_script)

    def test_latest_scenarios_do_not_embed_packager_home_path(self):
        checked_files = [BUILD_SCRIPT, ROOT_DIR / "tools/gaasd_scenario_panel/app.py"]
        for scenario_dir in (ACC_SCENARIO_DIR, LKS_SCENARIO_DIR):
            checked_files.extend(
                [
                    scenario_dir / "run.sh",
                    scenario_dir / "stop.sh",
                    scenario_dir / "scenario.yaml",
                    scenario_dir / "bridge_config.json",
                ]
            )

        for path in checked_files:
            self.assertNotIn(
                "/home/aiden",
                path.read_text(encoding="utf-8"),
                str(path),
            )


if __name__ == "__main__":
    unittest.main()
