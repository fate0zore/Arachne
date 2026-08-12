"""Offline unit tests for IK reachability filter and tf_utils (pure numpy)."""

import time
import unittest

import numpy as np

from arachne_operator.kinematics import AuboI5Kinematics
from viewpoint_planner.ik_filter import IkParams, IkReachabilityFilter, IkResult
from viewpoint_planner.tf_utils import (
    compose_tool0_from_camera,
    invert_rigid,
    mat_to_quat_xyzw,
    quat_to_matrix_xyzw,
    to_aubo_frame,
    transform_from_parts,
)


def pose4(translation, rotation=None):
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    if rotation is not None:
        transform[:3, :3] = np.asarray(rotation, dtype=float)
    return transform


class IkFilterTest(unittest.TestCase):
    def setUp(self):
        self.kinematics = AuboI5Kinematics()
        self.filter = IkReachabilityFilter(kinematics=self.kinematics)

    def test_reachable_pose(self):
        q_known = np.array([0.2, -0.4, 0.6, -0.3, 0.1, 0.4])
        target = self.kinematics.fk(q_known)
        result = self.filter.check(target, q_seed=q_known)
        self.assertTrue(result.ok)
        np.testing.assert_allclose(result.q, q_known, atol=0.01)

    def test_unreachable_pose(self):
        target = pose4([5.0, 5.0, 5.0])
        result = self.filter.check(target, q_seed=np.zeros(6))
        self.assertFalse(result.ok)
        self.assertIsNone(result.q)

    def test_joint_limits_reject_out_of_range_solution(self):
        # narrow limits: IK converges to 0.5 rad which exceeds (0.1, 0.2)
        narrow = tuple((0.1, 0.2) for _ in range(6))
        tight = IkReachabilityFilter(
            kinematics=self.kinematics, joint_limits=narrow
        )
        q_known = np.full(6, 0.5)
        target = self.kinematics.fk(q_known)
        result = tight.check(target, q_seed=q_known)
        self.assertFalse(result.ok)

    def test_within_limits_pose(self):
        q_known = np.full(6, 0.15)
        target = self.kinematics.fk(q_known)
        result = self.filter.check(target, q_seed=q_known)
        self.assertTrue(result.ok)
        np.testing.assert_allclose(result.q, q_known, atol=0.01)

    def test_serial_seed_filtering(self):
        # smoothly varying poses should all be reachable with serial seeds
        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        poses = [
            self.kinematics.fk(q0 + 0.02 * i * np.ones(6)) for i in range(4)
        ]
        results = self.filter.filter_poses(poses, q_seed=q0)
        self.assertEqual(len(results), 4)
        self.assertTrue(all(r.ok for r in results))


class TfUtilsTest(unittest.TestCase):
    def test_quat_matrix_roundtrip(self):
        for quat in ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.70710678, 0.70710678)):
            matrix = quat_to_matrix_xyzw(quat)
            np.testing.assert_allclose(mat_to_quat_xyzw(matrix), quat, atol=1e-9)

    def test_matrix_quat_roundtrip(self):
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        quat = mat_to_quat_xyzw(rotation)
        np.testing.assert_allclose(quat_to_matrix_xyzw(quat), rotation, atol=1e-9)

    def test_transform_from_parts(self):
        transform = transform_from_parts((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
        np.testing.assert_allclose(transform[:3, 3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(transform[:3, :3], np.eye(3))

    def test_invert_rigid(self):
        transform = transform_from_parts((1.0, -2.0, 0.5), (0.0, 0.0, 0.70710678, 0.70710678))
        product = transform @ invert_rigid(transform)
        np.testing.assert_allclose(product, np.eye(4), atol=1e-7)

    def test_compose_tool0_from_camera(self):
        camera_pose = pose4([0.4, 0.1, 0.6], np.eye(3))
        tool0_in_camera = pose4([0.02, 0.0, 0.05], np.eye(3))
        tool0_pose = compose_tool0_from_camera(camera_pose, tool0_in_camera)
        # reconstruction: camera_pose == tool0_pose @ tool0_in_camera
        np.testing.assert_allclose(tool0_pose @ tool0_in_camera, camera_pose, atol=1e-9)
        np.testing.assert_allclose(tool0_pose[:3, 3], [0.38, 0.1, 0.55], atol=1e-9)

    def test_to_aubo_frame(self):
        base_in_aubo = pose4([0.1, 0.0, 0.2], np.eye(3))
        pose_base = pose4([0.3, 0.0, 0.1], np.eye(3))
        pose_aubo = to_aubo_frame(pose_base, base_in_aubo)
        np.testing.assert_allclose(pose_aubo[:3, 3], [0.4, 0.0, 0.3], atol=1e-9)


if __name__ == "__main__":
    unittest.main()


class FilterPosesBudgetTest(unittest.TestCase):
    """filter_poses time budget: exhausted budget marks remaining poses."""

    def _slow_filter(self, delay: float = 0.2):
        class SlowFilter(IkReachabilityFilter):
            def check(self, pose, q_seed):  # noqa: ANN001
                time.sleep(delay)
                return IkResult(True, np.asarray(q_seed, dtype=float), 0.0, 0.0)

        return SlowFilter(kinematics=AuboI5Kinematics())

    def test_budget_marks_remaining_unreachable(self):
        f = self._slow_filter(delay=0.15)
        poses = [pose4([0.3, 0.0, 0.5])] * 5
        results = f.filter_poses(poses, np.zeros(6), time_budget_sec=0.25)
        self.assertEqual(len(results), 5)
        self.assertTrue(results[0].ok)
        self.assertFalse(results[-1].ok)
        solved = sum(1 for r in results if r.ok)
        self.assertLess(solved, 5)  # 至少一个被预算跳过
        self.assertGreaterEqual(solved, 1)  # 至少一个被真实求解

    def test_no_budget_solves_all(self):
        f = self._slow_filter(delay=0.0)
        poses = [pose4([0.3, 0.0, 0.5])] * 3
        results = f.filter_poses(poses, np.zeros(6))
        self.assertTrue(all(r.ok for r in results))
