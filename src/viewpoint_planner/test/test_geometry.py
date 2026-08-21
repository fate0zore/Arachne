"""Offline unit tests for viewpoint_planner.geometry (pure numpy)."""

import unittest

import numpy as np

from arachne_operator.kinematics import AuboI5Kinematics
from viewpoint_planner.geometry import (
    CollisionBox,
    arm_collision_clearance_base,
    box_box_clearance,
    box_from_center_size,
    box_from_min_max,
    default_collision_boxes,
    point_box_clearance,
    self_collision_clearance,
)


class BoxTest(unittest.TestCase):
    def test_box_from_min_max(self):
        box = box_from_min_max("b", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), margin=0.1)
        self.assertIsNotNone(box)
        self.assertEqual(box.name, "b")
        # center unchanged, size padded by 2*margin on each axis
        np.testing.assert_allclose(box.center, (0.5, 0.5, 0.5), atol=1e-9)
        np.testing.assert_allclose(box.size, (1.2, 1.2, 1.2), atol=1e-9)
        np.testing.assert_allclose(box.minimum, (-0.1, -0.1, -0.1), atol=1e-9)

    def test_box_from_min_max_rejects_degenerate(self):
        self.assertIsNone(box_from_min_max("b", (1, 1, 1), (0, 0, 0)))

    def test_box_box_clearance_separated_and_overlapping(self):
        a_min = np.array([0.0, 0.0, 0.0])
        a_max = np.array([1.0, 1.0, 1.0])
        b_min = np.array([3.0, 0.0, 0.0])
        b_max = np.array([4.0, 1.0, 1.0])
        self.assertAlmostEqual(box_box_clearance(a_min, a_max, b_min, b_max), 2.0)
        # overlapping: negative penetration depth
        b_min = np.array([0.8, 0.0, 0.0])
        b_max = np.array([2.0, 1.0, 1.0])
        self.assertAlmostEqual(box_box_clearance(a_min, a_max, b_min, b_max), -0.2)

    def test_point_box_clearance(self):
        box = CollisionBox("b", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        # inside -> negative (penetration)
        self.assertAlmostEqual(point_box_clearance(np.array([0.0, 0.0, 0.0]), box), -0.5)
        # outside along x
        self.assertAlmostEqual(point_box_clearance(np.array([2.0, 0.0, 0.0]), box), 1.5)

    def test_default_collision_boxes_contains_4(self):
        boxes = default_collision_boxes()
        names = [box.name for box in boxes]
        self.assertIn("scout_base_main", names)
        self.assertIn("scout_base_center_ridge", names)
        self.assertIn("front_basket_keepout", names)
        self.assertIn("rear_sensor_rack", names)
        self.assertEqual(len(boxes), 4)

    def test_default_collision_boxes_padding_applied(self):
        baskets = [b for b in default_collision_boxes() if b.name == "front_basket_keepout"]
        self.assertEqual(len(baskets), 1)
        # nominal basket size ~(0.244, 0.22, 0.187); margin 0.015 pads each side
        padded = baskets[0].size
        self.assertGreater(padded[0], 0.24)
        self.assertGreater(padded[2], 0.18)


class ArmClearanceTest(unittest.TestCase):
    def setUp(self):
        self.kinematics = AuboI5Kinematics()
        self.identity = np.eye(4)

    def test_free_pose_collision_free(self):
        boxes = []  # no obstacles
        ok, clearance, hit = arm_collision_clearance_base(
            np.zeros(6),
            self.identity,
            kinematics=self.kinematics,
            boxes=boxes,
            ground_min_z_base=-10.0,  # ground far below
        )
        self.assertTrue(ok)
        # hit_name records the closest surface (may be ground); ok is what matters

    def test_box_hit_detected(self):
        # place a small box at a middle link so the arm actually hits it
        points = [
            t[:3, 3]
            for _name, t in self.kinematics.link_transforms(np.zeros(6))
        ]
        middle = tuple(float(v) for v in points[3])
        box = CollisionBox("probe", middle, (0.06, 0.06, 0.06))
        ok, _clearance, hit = arm_collision_clearance_base(
            np.zeros(6),
            self.identity,
            kinematics=self.kinematics,
            boxes=[box],
            ground_min_z_base=-10.0,
        )
        self.assertFalse(ok)
        self.assertEqual(hit, "probe")

    def test_ground_violation(self):
        # ground threshold raised far above the arm -> collision on ground
        ok, _clearance, hit = arm_collision_clearance_base(
            np.zeros(6),
            self.identity,
            kinematics=self.kinematics,
            boxes=[],
            ground_min_z_base=5.0,
        )
        self.assertFalse(ok)
        self.assertEqual(hit, "ground/arm")


class SelfCollisionTest(unittest.TestCase):
    def setUp(self):
        self.kinematics = AuboI5Kinematics()
        self.identity = np.eye(4)

    def test_adjacent_links_exempt(self):
        # With a huge radius, only non-adjacent pairs can ever violate; if the
        # adjacency exemption were broken, adjacent segments (which share a
        # joint) would be flagged even at zero radius.
        ok, clearance, hit = self_collision_clearance(
            np.zeros(6),
            self.identity,
            kinematics=self.kinematics,
            link_radius=0.0,
        )
        self.assertTrue(ok, f"zero-radius should be collision-free, got {hit}")

    def test_large_radius_triggers_self_collision(self):
        ok, clearance, hit = self_collision_clearance(
            np.zeros(6),
            self.identity,
            kinematics=self.kinematics,
            link_radius=0.5,
        )
        self.assertFalse(ok)
        self.assertIsNotNone(hit)
        self.assertLess(clearance, 0.0)


if __name__ == "__main__":
    unittest.main()
