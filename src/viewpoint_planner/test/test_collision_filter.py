"""Offline unit tests for viewpoint_planner.collision_filter."""

import unittest

import numpy as np

from arachne_operator.kinematics import AuboI5Kinematics
from viewpoint_planner.collision_filter import (
    CollisionFilter,
    CollisionParams,
    CollisionResult,
)
from viewpoint_planner.geometry import CollisionBox


class _FakeIkResult:
    def __init__(self, ok, q):
        self.ok = ok
        self.q = None if q is None else np.array(q, dtype=float)


class CollisionFilterTest(unittest.TestCase):
    def setUp(self):
        self.kinematics = AuboI5Kinematics()
        self.identity = np.eye(4)

    def test_self_pose_collision_free(self):
        filt = CollisionFilter(
            kinematics=self.kinematics, boxes=[]
        )
        result = filt.check(np.zeros(6), self.identity)
        self.assertTrue(result.collision_free, f"clearance={result.clearance} hit={result.hit_name}")

    def test_box_hit_reported(self):
        points = [
            t[:3, 3]
            for _name, t in self.kinematics.link_transforms(np.zeros(6))
        ]
        middle = tuple(float(v) for v in points[3])
        box = CollisionBox("probe", middle, (0.06, 0.06, 0.06))
        filt = CollisionFilter(kinematics=self.kinematics, boxes=[box])
        result = filt.check(np.zeros(6), self.identity)
        self.assertFalse(result.collision_free)
        self.assertEqual(result.hit_name, "probe")

    def test_ground_violation_reported(self):
        params = CollisionParams(ground_min_z_base=5.0)
        filt = CollisionFilter(kinematics=self.kinematics, boxes=[], params=params)
        result = filt.check(np.zeros(6), self.identity)
        self.assertFalse(result.collision_free)
        self.assertEqual(result.hit_name, "ground/arm")

    def test_unreachable_point_skips_collision(self):
        filt = CollisionFilter(kinematics=self.kinematics, boxes=[])
        ik = [_FakeIkResult(True, np.zeros(6)), _FakeIkResult(False, None)]
        results = filt.filter_joints(ik, self.identity)
        self.assertTrue(results[0].collision_free)
        self.assertFalse(results[1].collision_free)
        self.assertEqual(results[1].reason, "ik_unreachable")

    def test_budget_exhaustion_marks_collision_false(self):
        filt = CollisionFilter(kinematics=self.kinematics, boxes=[])
        ik = [_FakeIkResult(True, np.zeros(6))] * 5
        results = filt.filter_joints(ik, self.identity, time_budget_sec=0.0)
        # a zero budget should exhaust immediately: everything marked false
        self.assertTrue(all(not r.collision_free for r in results))
        self.assertTrue(any(r.reason == "budget" for r in results))

    def test_switch_combination_reachable_nonzero(self):
        # filter_collision gate lives at the node layer; here we just confirm
        # the filter always produces per-point flags (never deletes points).
        filt = CollisionFilter(kinematics=self.kinematics, boxes=[])
        ik = [_FakeIkResult(True, np.zeros(6))] * 3
        results = filt.filter_joints(ik, self.identity)
        self.assertEqual(len(results), 3)
        self.assertTrue(all(isinstance(r, CollisionResult) for r in results))


if __name__ == "__main__":
    unittest.main()
