import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from apa_parkin_goal import fit_goal, parallel_centered_goal, parking_footprint


class GoalGeometryTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[5]
        self.vehicle = json.loads((root/'pangu_gaasd_base/modules/apa/apa/conf/apa_vehicle_profile.json').read_text())
        self.slot = dict(slot_kind='parallel', slot_entry=1)
        for i, (x, y) in enumerate(((-3, -1.2), (3, -1.2), (3, 1.2), (-3, 1.2)), 1):
            self.slot.update({f'slot_pt{i}_x': x, f'slot_pt{i}_y': y})
        self.obstacle = dict(id='occupancy:neighbor', velocity_x=0, velocity_y=0,
            footprint=[dict(x=x, y=y) for x, y in ((2.75, -1), (4, -1), (4, 1), (2.75, 1))])

    def test_asymmetric_intrusion_moves_the_goal_away(self):
        nominal = parallel_centered_goal(self.slot, self.vehicle, 0)
        adjusted = parallel_centered_goal(self.slot, self.vehicle, 0, [self.obstacle])
        self.assertLess(adjusted[0], nominal[0])
        self.assertAlmostEqual(adjusted[1], nominal[1])
        body = parking_footprint(self.vehicle, adjusted)
        self.assertGreater(2.75-max(p['x'] for p in body), self.vehicle['parkin_physical_margin_m'])
        self.assertGreater(min(p['x'] for p in body), -3)

    def test_target_is_invariant_under_rigid_coordinate_changes(self):
        original = parallel_centered_goal(self.slot, self.vehicle, 0, [self.obstacle])
        angle = 1.1
        def transform(x, y):
            return 17.3+math.cos(angle)*x-math.sin(angle)*y, -43.8+math.sin(angle)*x+math.cos(angle)*y
        slot = dict(self.slot)
        for i in range(1, 5):
            slot[f'slot_pt{i}_x'], slot[f'slot_pt{i}_y'] = transform(slot[f'slot_pt{i}_x'], slot[f'slot_pt{i}_y'])
        polygon = [dict(zip(('x', 'y'), transform(p['x'], p['y']))) for p in self.obstacle['footprint']]
        result = parallel_centered_goal(slot, self.vehicle, angle, [dict(self.obstacle, footprint=polygon)])
        expected = transform(*original[:2])
        self.assertAlmostEqual(result[0], expected[0], places=6)
        self.assertAlmostEqual(result[1], expected[1], places=6)

    def test_parallel_physical_inside_edge_keeps_four_centimetres(self):
        preferred = (-1.0, 1.2-self.vehicle['width_m']/2-0.03)
        with self.assertRaises(ValueError):
            fit_goal(self.slot, self.vehicle, preferred, 0, adjust=False)

    def test_body_support_excludes_the_rectangle_corner(self):
        body = parking_footprint(self.vehicle, (0, 0, 0))
        self.assertEqual(len(body), 8)
        self.assertLess(max(p['x']+p['y'] for p in body), max(p['x'] for p in body)+max(p['y'] for p in body))


if __name__ == '__main__':
    unittest.main()
