import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


PANEL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


class PanelShutdownApiTest(unittest.TestCase):
    def test_shutdown_endpoint_rejects_non_loopback_clients(self):
        with mock.patch.object(panel_app, "start_panel_shutdown") as shutdown:
            response = panel_app.app.test_client().post(
                "/api/panel/shutdown",
                environ_base={"REMOTE_ADDR": "192.0.2.10"},
            )

        self.assertEqual(response.status_code, 403)
        shutdown.assert_not_called()

    def test_shutdown_endpoint_schedules_global_cleanup(self):
        with mock.patch.object(
            panel_app, "start_panel_shutdown", return_value=True
        ) as shutdown:
            response = panel_app.app.test_client().post("/api/panel/shutdown")

        self.assertEqual(response.status_code, 202)
        self.assertTrue(response.get_json()["ok"])
        shutdown.assert_called_once_with()

    def test_repeated_shutdown_request_is_idempotent(self):
        with mock.patch.object(
            panel_app, "start_panel_shutdown", return_value=False
        ) as shutdown:
            response = panel_app.app.test_client().post("/api/panel/shutdown")

        self.assertEqual(response.status_code, 202)
        self.assertTrue(response.get_json()["ok"])
        self.assertTrue(response.get_json()["already_stopping"])
        shutdown.assert_called_once_with()


class PanelShutdownCleanupTest(unittest.TestCase):
    def create_scenario(self, root: Path, scenario_id: str) -> Path:
        scenario_dir = root / scenario_id
        scenario_dir.mkdir()
        (scenario_dir / "scenario.yaml").write_text(
            f"name: {scenario_id}\n",
            encoding="utf-8",
        )
        stop_script = scenario_dir / "stop.sh"
        stop_script.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
        return stop_script

    def test_all_scenario_stop_scripts_run_before_panel_exit(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            scenarios_root = Path(temp_dir)
            first = self.create_scenario(scenarios_root, "first")
            second = self.create_scenario(scenarios_root, "second")
            completed = subprocess.CompletedProcess([], 0, "stopped")

            with mock.patch.object(
                panel_app, "SCENARIOS_DIR", scenarios_root
            ), mock.patch.object(
                panel_app.subprocess, "run", return_value=completed
            ) as runner:
                failures = panel_app.stop_all_scenario_processes()

        self.assertEqual(failures, [])
        commands = [call.args[0] for call in runner.call_args_list]
        self.assertEqual(commands, [["bash", str(first)], ["bash", str(second)]])


class PanelShutdownUiTest(unittest.TestCase):
    def test_ui_exposes_confirmed_shutdown_action(self):
        html = (PANEL_DIR / "templates" / "index.html").read_text(encoding="utf-8")
        javascript = (PANEL_DIR / "static" / "app.js").read_text(encoding="utf-8")

        self.assertIn('id="shutdownPanelBtn"', html)
        self.assertIn("/api/panel/shutdown", javascript)
        self.assertIn("window.confirm", javascript)


if __name__ == "__main__":
    unittest.main()
