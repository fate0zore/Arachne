"""Offline unit tests for viewpoint strategies (pure math, no ROS)."""

import math
import unittest

import numpy as np

from viewpoint_planner.strategies.base import ViewpointStrategy, get_strategy
from viewpoint_planner.strategies.fibonacci_sphere import (
    FibonacciSphereStrategy,
    _look_at_quat,
)
from viewpoint_planner.tf_utils import mat_to_quat_xyzw


def quat_rotate(quat, vec):
    """Rotate vector by (x, y, z, w) quaternion."""
    x, y, z, w = quat
    qv = np.array([x, y, z])
    vec = np.asarray(vec, dtype=float)
    return vec + 2.0 * np.cross(qv, np.cross(qv, vec) + w * vec)


class FibonacciSphereDistributionTest(unittest.TestCase):
    TARGET = (0.3, -0.2, 0.15)
    RADIUS = 0.5
    COUNT = 32

    def samples(self, **kwargs):
        strategy = FibonacciSphereStrategy()
        return strategy.plan(
            self.TARGET,
            self.RADIUS,
            self.COUNT,
            up_vector_base=kwargs.pop("up", (0.0, 0.0, 1.0)),
            **kwargs,
        )

    def test_count_and_radius(self):
        samples = self.samples()
        self.assertEqual(len(samples), self.COUNT)
        for sample in samples:
            point = np.asarray(sample.camera_xyz_base)
            dist = float(np.linalg.norm(point - np.asarray(self.TARGET)))
            self.assertAlmostEqual(dist, self.RADIUS, places=9)

    def test_optical_axis_points_at_target(self):
        samples = self.samples()
        for sample in samples:
            direction = np.asarray(self.TARGET) - np.asarray(sample.camera_xyz_base)
            direction = direction / np.linalg.norm(direction)
            rotated_z = quat_rotate(sample.camera_quat_xyzw, [0.0, 0.0, 1.0])
            rotated_z = rotated_z / np.linalg.norm(rotated_z)
            np.testing.assert_allclose(rotated_z, direction, atol=1e-9)

    def test_no_duplicate_points(self):
        samples = self.samples()
        points = [np.asarray(s.camera_xyz_base) for s in samples]
        min_chord = min(
            float(np.linalg.norm(points[i] - points[j]))
            for i in range(len(points))
            for j in range(i + 1, len(points))
        )
        # fibonacci sphere 32 pts: min chord ~0.55*radius; loose floor catches clustering
        self.assertGreater(min_chord, 0.3 * self.RADIUS)

    def test_quaternion_is_unit(self):
        for sample in self.samples():
            q = np.asarray(sample.camera_quat_xyzw)
            self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=9)

    def test_hemisphere_upper_only(self):
        samples = self.samples(hemisphere=True)
        for sample in samples:
            # z coordinate of sampled position, in up-aligned coordinates,
            # must be >= 0; with up=(0,0,1) that is the point z itself.
            self.assertGreaterEqual(sample.camera_xyz_base[2] - self.TARGET[2], -1e-9)

    def test_custom_up_vector(self):
        up = (0.0, 1.0, 0.0)
        samples = self.samples(up=up)
        self.assertEqual(len(samples), self.COUNT)
        for sample in samples:
            point = np.asarray(sample.camera_xyz_base)
            self.assertAlmostEqual(
                float(np.linalg.norm(point - np.asarray(self.TARGET))), self.RADIUS, places=9
            )
            direction = np.asarray(self.TARGET) - point
            direction = direction / np.linalg.norm(direction)
            rotated_z = quat_rotate(sample.camera_quat_xyzw, [0.0, 0.0, 1.0])
            rotated_z = rotated_z / np.linalg.norm(rotated_z)
            np.testing.assert_allclose(rotated_z, direction, atol=1e-9)

    def test_pole_degeneracy_no_crash(self):
        # hemisphere sampling puts a sample near the pole where the look-at
        # direction is (anti-)parallel to the up vector; must not crash.
        samples = self.samples(hemisphere=True, up=(0.0, 0.0, 1.0))
        for sample in samples:
            q = np.asarray(sample.camera_quat_xyzw)
            self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=9)

    def test_invalid_arguments(self):
        strategy = FibonacciSphereStrategy()
        with self.assertRaises(ValueError):
            strategy.plan(self.TARGET, self.RADIUS, 0)
        with self.assertRaises(ValueError):
            strategy.plan(self.TARGET, 0.0, self.COUNT)
        with self.assertRaises(ValueError):
            strategy.plan(self.TARGET, self.RADIUS, self.COUNT, up_vector_base=(0.0, 0.0, 0.0))


class LookAtQuaternionTest(unittest.TestCase):
    def test_look_at_orientation(self):
        direction = np.array([1.0, 0.0, 0.0])
        quat = _look_at_quat(direction, np.array([0.0, 0.0, 1.0]))
        rotated_z = quat_rotate(quat, [0.0, 0.0, 1.0])
        np.testing.assert_allclose(rotated_z, direction, atol=1e-9)

    def test_degenerate_parallel_direction(self):
        up = np.array([0.0, 0.0, 1.0])
        for direction in (up, -up):
            quat = _look_at_quat(direction, up)
            rotated_z = quat_rotate(quat, [0.0, 0.0, 1.0])
            rotated_z = rotated_z / np.linalg.norm(rotated_z)
            np.testing.assert_allclose(rotated_z, direction, atol=1e-9)

    def test_matrix_quaternion_roundtrip(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        )  # 90 deg about z
        quat = mat_to_quat_xyzw(rotation)
        rotated_z = quat_rotate(quat, [0.0, 0.0, 1.0])
        np.testing.assert_allclose(rotated_z, [0.0, 0.0, 1.0], atol=1e-9)
        rotated_x = quat_rotate(quat, [1.0, 0.0, 0.0])
        np.testing.assert_allclose(rotated_x, [0.0, 1.0, 0.0], atol=1e-9)


class RegistryTest(unittest.TestCase):
    def test_get_strategy_instantiates(self):
        strategy = get_strategy("fibonacci_sphere")
        self.assertIsInstance(strategy, ViewpointStrategy)

    def test_unknown_strategy_error_lists_available(self):
        with self.assertRaises(ValueError) as ctx:
            get_strategy("does_not_exist")
        message = str(ctx.exception)
        self.assertIn("does_not_exist", message)
        self.assertIn("fibonacci_sphere", message)

    def test_register_duplicate_rejected(self):
        class Duplicate(ViewpointStrategy):
            name = "fibonacci_sphere"

            def plan(self, *args, **kwargs):  # pragma: no cover
                return []

        from viewpoint_planner.strategies.base import register_strategy

        with self.assertRaises(ValueError):
            register_strategy(Duplicate)


if __name__ == "__main__":
    unittest.main()
