import sys
import unittest
from pathlib import Path
from unittest import mock


PANEL_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PANEL_DIR.parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


class AccDriverCommandTest(unittest.TestCase):
    def test_w_is_sent_as_active_level_command(self):
        with mock.patch.object(
            panel_app, "publish_control_message", return_value=(True, "sent")
        ) as publish:
            ok, _message = panel_app.send_driver_command(
                "test_acc", {"bridge": {}}, "w", write_log=False
            )

        self.assertTrue(ok)
        call = publish.call_args.kwargs
        self.assertEqual(call["topic"], panel_app.DRIVER_COMMAND_TOPIC)
        self.assertEqual(call["payload"]["command_type"], 5)
        self.assertEqual(call["payload"]["mode"], "level")
        self.assertTrue(call["payload"]["active"])

    def test_e_is_sent_as_single_pulse_with_state_dependent_description(self):
        with mock.patch.object(
            panel_app, "publish_control_message", return_value=(True, "sent")
        ) as publish:
            ok, _message = panel_app.send_driver_command(
                "test_acc", {"bridge": {}}, "e", write_log=False
            )

        self.assertTrue(ok)
        call = publish.call_args.kwargs
        self.assertEqual(call["payload"]["command_type"], 1)
        self.assertEqual(call["payload"]["mode"], "pulse")
        self.assertEqual(call["repeat_count"], 1)
        self.assertIn("待命", call["payload"]["description"])
        self.assertIn("在控", call["payload"]["description"])


class AccDriverUiTest(unittest.TestCase):
    def test_acc_level_keys_have_heartbeat_and_release_cleanup(self):
        javascript = (PANEL_DIR / "static" / "app.js").read_text(encoding="utf-8")

        self.assertIn("accLevelTimer", javascript)
        self.assertIn("setInterval", javascript)
        self.assertIn("queueDriverCommand(key, false)", javascript)
        self.assertIn("clearInterval(state.accLevelTimer)", javascript)

    def test_e_label_explains_both_state_dependent_actions(self):
        html = (PANEL_DIR / "templates" / "index.html").read_text(encoding="utf-8")

        self.assertIn("待命启控 / 在控降速", html)


class AccScenarioStartupTest(unittest.TestCase):
    RUN_SCRIPT = (
        REPO_ROOT
        / "scenarios"
        / "newaccpro4_accel_codegen_carla_20260824"
        / "run.sh"
    )

    def test_manual_test_scenario_does_not_auto_engage_acc(self):
        script = self.RUN_SCRIPT.read_text(encoding="utf-8")

        self.assertIn('AUTO_START_COMMAND="${AUTO_START_COMMAND:-}"', script)
        self.assertIn("ready: ACC is in standby", script)


if __name__ == "__main__":
    unittest.main()
