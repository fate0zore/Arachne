"""Fibonacci sphere (golden-angle) uniform sampling strategy.

Pure math, no ROS dependency: unit-testable offline.
"""

from __future__ import annotations

import math

import numpy as np

from viewpoint_planner.strategies.base import (
    ViewpointSample,
    ViewpointStrategy,
    register_strategy,
)
from viewpoint_planner.tf_utils import mat_to_quat_xyzw

# golden angle in radians = 2*pi / phi^2 = pi * (3 - sqrt(5))
GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))

_EPS = 1e-9


def _rotation_from_z_to(up: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping the +z axis onto unit vector `up`."""
    z = np.array([0.0, 0.0, 1.0])
    dot = float(np.clip(np.dot(z, up), -1.0, 1.0))
    if dot > 1.0 - _EPS:
        return np.eye(3)
    if dot < -1.0 + _EPS:
        # opposite: rotate pi around any axis perpendicular to z
        axis = np.array([1.0, 0.0, 0.0])
        return _axis_rotation(axis, math.pi)
    axis = np.cross(z, up)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(dot)
    return _axis_rotation(axis, angle)


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation matrix."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    )


def _any_perpendicular(vec: np.ndarray) -> np.ndarray:
    """A unit vector perpendicular to unit `vec` (degenerate-case fallback)."""
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, vec))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    perp = np.cross(ref, vec)
    return perp / np.linalg.norm(perp)


def _look_at_quat(
    direction: np.ndarray, up: np.ndarray
) -> tuple[float, float, float, float]:
    """Orientation whose +z is `direction` and +y is as close to `up` as possible.

    `direction` and `up` are unit vectors. Falls back to an arbitrary
    perpendicular frame when direction is (anti-)parallel to `up`.
    """
    z = direction / np.linalg.norm(direction)
    x = np.cross(up, z)
    norm = float(np.linalg.norm(x))
    if norm < _EPS:
        x = _any_perpendicular(z)
    else:
        x = x / norm
    y = np.cross(z, x)
    rotation = np.column_stack((x, y, z))
    return mat_to_quat_xyzw(rotation)


@register_strategy
class FibonacciSphereStrategy(ViewpointStrategy):
    """Uniform fibonacci-sphere sampling around the target.

    Full-sphere (default) uses z_i = 1 - 2*(i + 0.5)/N; pass
    hemisphere=True (via kwargs) for the upper half z_i = (i + 0.5)/N,
    still with `up_vector_base` as the polar axis.
    """

    name = "fibonacci_sphere"

    def plan(
        self,
        target_xyz_base: tuple[float, float, float],
        radius_m: float,
        num_viewpoints: int,
        up_vector_base: tuple[float, float, float] = (0.0, 0.0, 1.0),
        **kwargs,
    ) -> list[ViewpointSample]:
        hemisphere = bool(kwargs.get("hemisphere", False))
        count = int(num_viewpoints)
        if count <= 0:
            raise ValueError(f"num_viewpoints must be positive, got {num_viewpoints}")
        if radius_m <= 0.0:
            raise ValueError(f"radius_m must be positive, got {radius_m}")

        target = np.asarray(target_xyz_base, dtype=float)
        up = np.asarray(up_vector_base, dtype=float)
        up_norm = np.linalg.norm(up)
        if up_norm < _EPS:
            raise ValueError("up_vector_base must be a non-zero vector")
        up = up / up_norm

        rotate_to_up = _rotation_from_z_to(up)

        samples: list[ViewpointSample] = []
        for index in range(count):
            if hemisphere:
                z = (index + 0.5) / count
            else:
                z = 1.0 - 2.0 * (index + 0.5) / count
            radial = math.sqrt(max(0.0, 1.0 - z * z))
            phi = index * GOLDEN_ANGLE
            point_up = np.array(
                [radial * math.cos(phi), radial * math.sin(phi), z]
            )
            point = target + radius_m * (rotate_to_up @ point_up)

            direction = target - point
            quat = _look_at_quat(direction, up)
            samples.append(
                ViewpointSample(
                    camera_xyz_base=tuple(float(v) for v in point),
                    camera_quat_xyzw=quat,
                )
            )
        return samples
