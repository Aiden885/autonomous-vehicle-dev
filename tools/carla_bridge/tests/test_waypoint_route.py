import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


BRIDGE_DIR = Path(__file__).resolve().parents[1]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def make_waypoint(road_id: int, yaw_deg: float, x_m: float, y_m: float):
    return SimpleNamespace(
        road_id=road_id,
        section_id=0,
        lane_id=1,
        s=0.0,
        transform=SimpleNamespace(
            location=SimpleNamespace(x=x_m, y=y_m, z=0.0),
            rotation=SimpleNamespace(yaw=yaw_deg),
        ),
    )


class DeterministicWaypointSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = load_module(
            "lead_waypoint_pid_route_test",
            BRIDGE_DIR / "lead-waypoint-pid-controller.py",
        )

    def test_equal_heading_candidates_use_stable_road_tie_break(self):
        high_road = make_waypoint(20, 10.0, 6.0, 1.0)
        low_road = make_waypoint(10, -10.0, 6.0, -1.0)

        first = self.controller.choose_forward_waypoint(
            [high_road, low_road], 0.0
        )
        second = self.controller.choose_forward_waypoint(
            [low_road, high_road], 0.0
        )

        self.assertEqual(first.road_id, 10)
        self.assertEqual(second.road_id, 10)

    def test_lead_route_uses_lane_heading_instead_of_biased_actor_heading(self):
        straight = make_waypoint(10, 0.0, 6.0, 0.0)
        turning = make_waypoint(20, 30.0, 5.2, 3.0)
        lane_waypoint = make_waypoint(1, 0.0, 0.0, 0.0)
        lane_waypoint.next = lambda _distance: [straight, turning]

        actor_transform = SimpleNamespace(
            location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(yaw=25.0),
        )
        actor = SimpleNamespace(get_transform=lambda: actor_transform)
        world_map = SimpleNamespace(get_waypoint=lambda *_args, **_kwargs: lane_waypoint)
        carla = SimpleNamespace(LaneType=SimpleNamespace(Driving=1))

        _steer, _error, selected = self.controller.compute_steer(
            carla,
            actor,
            world_map,
            lookahead_m=6.0,
            steer_kp=1.0,
            steer_kd=0.0,
            max_steer=1.0,
            previous_error=0.0,
            dt=0.05,
        )

        self.assertEqual(selected.road_id, 10)


class EgoWaypointSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_module = load_module(
            "carla_bridge_route_test",
            BRIDGE_DIR / "carla_bridge.py",
        )

    def test_ego_route_uses_lane_heading_instead_of_biased_actor_heading(self):
        straight = make_waypoint(10, 0.0, 6.0, 0.0)
        turning = make_waypoint(20, 30.0, 5.2, 3.0)
        lane_waypoint = make_waypoint(1, 0.0, 0.0, 0.0)
        lane_waypoint.next = lambda _distance: [straight, turning]

        actor_transform = SimpleNamespace(
            location=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(yaw=25.0),
        )
        actor = SimpleNamespace(
            get_transform=lambda: actor_transform,
            get_velocity=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )
        world_map = SimpleNamespace(get_waypoint=lambda *_args, **_kwargs: lane_waypoint)

        bridge = object.__new__(self.bridge_module.CarlaGaasdBridge)
        bridge.ego = actor
        bridge.world = SimpleNamespace(get_map=lambda: world_map)
        bridge.carla = SimpleNamespace(LaneType=SimpleNamespace(Driving=1))
        bridge.cfg = {
            "control": {
                "lane_keep_waypoint_lookahead_base_m": 6.0,
                "lane_keep_waypoint_lookahead_gain_s": 0.0,
                "lane_keep_waypoint_lookahead_min_m": 6.0,
                "lane_keep_waypoint_lookahead_max_m": 6.0,
                "lane_keep_waypoint_steer_kp": 1.0,
                "lane_keep_waypoint_steer_kd": 0.0,
                "lane_keep_max_steer": 1.0,
            }
        }
        bridge.lane_keep_prev_time = 0.0
        bridge.lane_keep_prev_error = 0.0

        steer = bridge.compute_waypoint_lane_keep_steer_norm()

        self.assertLess(steer, 0.0)


if __name__ == "__main__":
    unittest.main()
