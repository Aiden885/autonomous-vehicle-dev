#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "set-spectator-follow.py"
SPEC = importlib.util.spec_from_file_location("set_spectator_follow", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class FakeWorld:
    def __init__(self) -> None:
        self.wait_timeouts = []
        self.tick_calls = 0

    def wait_for_tick(self, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
        return object()

    def tick(self):
        self.tick_calls += 1


class SpectatorFollowTests(unittest.TestCase):
    def test_follow_loop_updates_once_per_passive_world_tick(self) -> None:
        world = FakeWorld()
        updates = []

        MODULE.follow_world_ticks(
            world,
            lambda: updates.append("updated"),
            timeout_sec=1.5,
            max_updates=3,
        )

        self.assertEqual(world.wait_timeouts, [1.5, 1.5, 1.5])
        self.assertEqual(updates, ["updated", "updated", "updated"])
        self.assertEqual(world.tick_calls, 0)


if __name__ == "__main__":
    unittest.main()
