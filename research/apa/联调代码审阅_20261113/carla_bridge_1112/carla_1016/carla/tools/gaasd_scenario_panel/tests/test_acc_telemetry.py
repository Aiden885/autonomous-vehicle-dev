import sys
import unittest
from pathlib import Path
from unittest import mock


PANEL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PANEL_DIR))

import app as panel_app  # noqa: E402


class AccTelemetrySnapshotTest(unittest.TestCase):
    def test_bridge_messages_are_merged_into_acc_dashboard_values(self):
        now = 1000.0
        messages = {
            panel_app.EGO_STATE_TOPIC: {
                "received_at": now - 0.05,
                "payload": {"velocity": {"speed_mps": 12.5}},
            },
            panel_app.LEAD_VEHICLE_TOPIC: {
                "received_at": now - 0.04,
                "payload": {
                    "valid": True,
                    "lead_speed_mps": 11.0,
                    "relative_speed_mps": 1.5,
                    "clearance_m": 27.5,
                    "time_gap_sec": 2.2,
                    "ttc_sec": 18.3,
                },
            },
            panel_app.CHASSIS_FEEDBACK_TOPIC: {
                "received_at": now - 0.03,
                "payload": {
                    "speed_mps": 12.4,
                    "target_accel_mps2": -0.35,
                    "actual_accel_mps2": -0.31,
                    "filtered_accel_mps2": -0.28,
                    "accel_feedforward_mps2": 0.12,
                    "accel_feedback_mps2": -0.19,
                    "accel_integral_error_mps": -0.24,
                    "accel_controller_saturated": False,
                    "throttle": 0.0,
                    "brake": 0.18,
                    "control_enable": True,
                },
            },
        }
        diagnostic = {
            "desired_time_gap_s": 1.8,
            "last_command": {"key": "t", "label": "T 减小时距", "timestamp": now - 0.2},
        }

        result = panel_app.build_acc_telemetry_snapshot(
            messages, diagnostic, now=now
        )

        self.assertTrue(result["online"])
        self.assertEqual(result["control_state"], "active")
        self.assertAlmostEqual(result["ego_speed_mps"], 12.4)
        self.assertAlmostEqual(result["lead_speed_mps"], 11.0)
        self.assertAlmostEqual(result["clearance_m"], 27.5)
        self.assertAlmostEqual(result["actual_time_gap_s"], 2.2)
        self.assertAlmostEqual(result["desired_time_gap_s"], 1.8)
        self.assertAlmostEqual(result["desired_distance_m"], 22.32)
        self.assertAlmostEqual(result["distance_error_m"], 5.18)
        self.assertAlmostEqual(result["target_accel_mps2"], -0.35)
        self.assertAlmostEqual(result["actual_accel_mps2"], -0.31)
        self.assertAlmostEqual(result["filtered_accel_mps2"], -0.28)
        self.assertAlmostEqual(result["accel_feedforward_mps2"], 0.12)
        self.assertAlmostEqual(result["accel_feedback_mps2"], -0.19)
        self.assertAlmostEqual(result["accel_integral_error_mps"], -0.24)
        self.assertFalse(result["accel_controller_saturated"])
        self.assertEqual(result["last_command"]["key"], "t")

    def test_stale_messages_are_reported_offline(self):
        messages = {
            panel_app.CHASSIS_FEEDBACK_TOPIC: {
                "received_at": 10.0,
                "payload": {"speed_mps": 3.0, "control_enable": True},
            }
        }

        result = panel_app.build_acc_telemetry_snapshot(
            messages, {"desired_time_gap_s": 1.8}, now=20.0
        )

        self.assertFalse(result["online"])
        self.assertEqual(result["control_state"], "offline")

    def test_acc_telemetry_endpoint_uses_scenario_bridge_monitor(self):
        now = 1000.0
        monitor = mock.Mock()
        monitor.snapshot.return_value = {
            panel_app.CHASSIS_FEEDBACK_TOPIC: {
                "received_at": now,
                "payload": {"speed_mps": 5.0, "control_enable": False},
            }
        }
        diagnostic = mock.Mock()
        diagnostic.snapshot.return_value = {"desired_time_gap_s": 1.8}
        scenario_id = "newaccpro4_accel_codegen_carla_20260824"

        with mock.patch.object(panel_app, "get_telemetry_monitor", return_value=monitor), mock.patch.object(
            panel_app, "get_acc_diagnostic_state", return_value=diagnostic
        ), mock.patch.object(panel_app.time, "time", return_value=now):
            response = panel_app.app.test_client().get(
                f"/api/scenarios/{scenario_id}/telemetry"
            )

        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["online"])
        self.assertEqual(response.get_json()["control_state"], "standby")


class AccDiagnosticStateTest(unittest.TestCase):
    def test_time_gap_estimate_follows_active_t_and_r_commands(self):
        state = panel_app.AccDiagnosticState(
            initial_time_gap_s=1.8,
            time_gap_step_s=0.2,
            min_time_gap_s=1.0,
            max_time_gap_s=2.0,
        )

        state.record_command("t", control_active=True)
        self.assertAlmostEqual(state.snapshot()["desired_time_gap_s"], 1.6)

        state.record_command("r", control_active=True)
        state.record_command("r", control_active=True)
        state.record_command("r", control_active=True)
        self.assertAlmostEqual(state.snapshot()["desired_time_gap_s"], 2.0)

    def test_standby_gap_command_is_sent_but_does_not_change_estimate(self):
        state = panel_app.AccDiagnosticState()

        state.record_command("t", control_active=False)
        result = state.snapshot()

        self.assertAlmostEqual(result["desired_time_gap_s"], 1.8)
        self.assertFalse(result["last_command"]["estimated_applied"])

    def test_driver_command_reads_control_state_from_bridge_pub_endpoint(self):
        diagnostic = mock.Mock()
        scenario = {"bridge": {}}

        with mock.patch.object(
            panel_app, "bridge_control_endpoint", return_value="tcp://127.0.0.1:5702"
        ), mock.patch.object(
            panel_app, "bridge_publish_endpoint", return_value="tcp://127.0.0.1:5701"
        ), mock.patch.object(
            panel_app, "publish_control_message", return_value=(True, "sent")
        ), mock.patch.object(
            panel_app, "current_acc_control_active", return_value=True
        ) as active_mock, mock.patch.object(
            panel_app, "get_acc_diagnostic_state", return_value=diagnostic
        ):
            ok, _ = panel_app.send_driver_command(
                "acc_scenario", scenario, "r", write_log=False
            )

        self.assertTrue(ok)
        active_mock.assert_called_once_with("tcp://127.0.0.1:5701")
        diagnostic.record_command.assert_called_once_with("r", True)


class AccTelemetryUiTest(unittest.TestCase):
    def test_acc_dashboard_and_command_ack_feedback_are_present(self):
        html = (PANEL_DIR / "templates" / "index.html").read_text(encoding="utf-8")
        javascript = (PANEL_DIR / "static" / "app.js").read_text(encoding="utf-8")
        css = (PANEL_DIR / "static" / "style.css").read_text(encoding="utf-8")

        self.assertIn('id="accTelemetryCard"', html)
        self.assertIn('id="accEgoSpeed"', html)
        self.assertIn('id="accLeadDistance"', html)
        self.assertIn('id="accDesiredTimeGap"', html)
        self.assertIn('id="accDesiredDistance"', html)
        self.assertIn('id="accActualAccel"', html)
        self.assertIn('id="accAccelFeedforward"', html)
        self.assertIn('id="accAccelFeedback"', html)
        self.assertIn('id="accAccelSaturation"', html)
        self.assertIn("pollAccTelemetry", javascript)
        self.assertIn("command-ack", javascript)
        self.assertIn(".dashboard .acc-telemetry-card", css)


if __name__ == "__main__":
    unittest.main()
