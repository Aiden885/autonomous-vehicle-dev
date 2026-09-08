import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PANEL_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PANEL_DIR.parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


def load_bridge_module():
    module_path = REPO_ROOT / "tools" / "carla_bridge" / "carla_bridge.py"
    spec = importlib.util.spec_from_file_location("carla_bridge_runtime_test", module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class CarlaFrameModeApiTest(unittest.TestCase):
    def setUp(self):
        self.client = panel_app.app.test_client()

    def start_environment(self, mode, quality_mode="low"):
        with mock.patch.object(panel_app, "run_script_job", return_value=(True, "queued")) as runner:
            response = self.client.post(
                "/api/scenarios/lks3_codegen_carla_20260817/start",
                json={"carla_frame_mode": mode, "carla_quality_mode": quality_mode},
            )
        return response, runner

    def test_stable_mode_uses_twenty_fps_and_005_second_step(self):
        response, runner = self.start_environment("stable")

        self.assertEqual(response.status_code, 202)
        environment = runner.call_args.kwargs["env_overrides"]
        self.assertIn("-fps=20", environment["CARLA_EXTRA_ARGS"])
        self.assertEqual(environment["CARLA_FIXED_DELTA_SECONDS"], "0.05")
        self.assertEqual(environment["CARLA_QUALITY_MODE"], "low")

    def test_normal_mode_uses_thirty_fps_and_matching_step(self):
        response, runner = self.start_environment("normal")

        self.assertEqual(response.status_code, 202)
        environment = runner.call_args.kwargs["env_overrides"]
        self.assertIn("-fps=30", environment["CARLA_EXTRA_ARGS"])
        self.assertEqual(environment["CARLA_FIXED_DELTA_SECONDS"], "0.03333333333333333")

    def test_normal_quality_is_forwarded_to_launcher(self):
        response, runner = self.start_environment("stable", "normal")

        self.assertEqual(response.status_code, 202)
        environment = runner.call_args.kwargs["env_overrides"]
        self.assertEqual(environment["CARLA_QUALITY_MODE"], "normal")

    def test_unknown_mode_is_rejected(self):
        response, runner = self.start_environment("turbo")

        self.assertEqual(response.status_code, 400)
        runner.assert_not_called()

    def test_unknown_quality_mode_is_rejected(self):
        response, runner = self.start_environment("stable", "cinema")

        self.assertEqual(response.status_code, 400)
        runner.assert_not_called()


class CarlaFrameModeBridgeTest(unittest.TestCase):
    def test_bridge_applies_fixed_delta_from_environment(self):
        bridge = load_bridge_module()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as config_file:
            json.dump({}, config_file)
            config_file.flush()
            with mock.patch.dict(
                bridge.os.environ,
                {"CARLA_FIXED_DELTA_SECONDS": "0.03333333333333333"},
                clear=False,
            ), mock.patch.object(bridge, "CarlaGaasdBridge") as bridge_class:
                bridge.main(["--config", config_file.name])

        config = bridge_class.call_args.args[0]
        self.assertAlmostEqual(config["carla"]["fixed_delta_seconds"], 1.0 / 30.0)


class CarlaQualityLauncherTest(unittest.TestCase):
    WRAPPERS = (
        REPO_ROOT / "tools" / "carla_bridge" / "start-carla.sh",
        REPO_ROOT
        / "scenarios"
        / "acc_carla_phase2_20260513"
        / "bridge_snapshot"
        / "tools"
        / "carla_bridge"
        / "start-carla.sh",
    )

    def run_wrapper(self, wrapper, quality_mode):
        with tempfile.TemporaryDirectory() as temp_dir:
            fake_root = Path(temp_dir)
            capture_path = fake_root / "arguments.txt"
            package_launcher = fake_root / "start-carla.sh"
            package_launcher.write_text(
                '#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "$CAPTURE_PATH"\n',
                encoding="utf-8",
            )
            package_launcher.chmod(0o755)
            environment = os.environ.copy()
            environment["CAPTURE_PATH"] = str(capture_path)
            environment["CARLA_QUALITY_MODE"] = quality_mode
            subprocess.run(
                ["bash", str(wrapper), "--root", str(fake_root)],
                check=True,
                env=environment,
                capture_output=True,
                text=True,
            )
            return capture_path.read_text(encoding="utf-8").splitlines()

    def available_wrappers(self):
        wrappers = tuple(wrapper for wrapper in self.WRAPPERS if wrapper.exists())
        self.assertTrue(wrappers)
        return wrappers

    def test_normal_quality_invokes_package_high_mode(self):
        for wrapper in self.available_wrappers():
            with self.subTest(wrapper=wrapper):
                self.assertIn("--high", self.run_wrapper(wrapper, "normal"))

    def test_low_quality_keeps_package_default_mode(self):
        for wrapper in self.available_wrappers():
            with self.subTest(wrapper=wrapper):
                self.assertNotIn("--high", self.run_wrapper(wrapper, "low"))


class CarlaFrameModeUiTest(unittest.TestCase):
    def test_ui_exposes_frame_modes_and_posts_selection(self):
        html = (PANEL_DIR / "templates" / "index.html").read_text(encoding="utf-8")
        javascript = (PANEL_DIR / "static" / "app.js").read_text(encoding="utf-8")

        self.assertIn('name="carlaFrameMode"', html)
        self.assertIn('value="stable"', html)
        self.assertIn('value="normal"', html)
        self.assertIn("carla_frame_mode", javascript)
        self.assertIn('name="carlaQualityMode"', html)
        self.assertIn('value="low"', html)
        self.assertIn("carla_quality_mode", javascript)


if __name__ == "__main__":
    unittest.main()
