#!/usr/bin/env python3

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "tools" / "carla_bridge"))
sys.path.insert(0, str(REPO_ROOT / "tools" / "pangu_acc_closed_loop"))

from carla_bridge import (  # noqa: E402
    AccelerationPedalController,
    CarlaGaasdBridge,
    LastCommand,
)
from record_acc_run import summarize  # noqa: E402


class FakeEgo:
    def get_velocity(self):
        return SimpleNamespace(x=3.0, y=4.0)

    def get_control(self):
        return SimpleNamespace(steer=0.1, throttle=0.2, brake=0.0, gear=1)


class AccelerationDiagnosticsTest(unittest.TestCase):
    def test_chassis_feedback_contains_acceleration_target(self):
        bridge = CarlaGaasdBridge.__new__(CarlaGaasdBridge)
        bridge.ego = FakeEgo()
        bridge.cfg = {"control": {"max_abs_steer_rad": 0.6, "timeout_sec": 1.0}}
        bridge.acceleration_pedal_controller = AccelerationPedalController({})
        bridge.last_command = LastCommand(
            timestamp=time.time(),
            command_id=7,
            throttle=0.2,
            brake=0.0,
            steer_norm=0.1,
            target_accel_mps2=-1.25,
            enable=True,
        )
        captured = {}
        bridge.publish = lambda topic, payload, snapshot: captured.update(
            {"topic": topic, "payload": payload}
        )

        bridge.publish_chassis_feedback(object())

        self.assertEqual(captured["topic"], "chassis_feedback")
        self.assertEqual(captured["payload"]["target_accel_mps2"], -1.25)

    def test_summary_reports_acceleration_target_range(self):
        samples = [
            {"target_accel_mps2": -2.0},
            {"target_accel_mps2": 0.5},
            {"target_accel_mps2": 1.25},
        ]

        summary = summarize(samples, 10.0, 11.0)

        self.assertEqual(summary["target_accel_min_mps2"], -2.0)
        self.assertEqual(summary["target_accel_max_mps2"], 1.25)
        self.assertEqual(summary["target_accel_last_mps2"], 1.25)


if __name__ == "__main__":
    unittest.main()
