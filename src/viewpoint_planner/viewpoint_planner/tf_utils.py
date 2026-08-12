"""Pure-numpy helpers for composing tf2 transform results.

The viewpoint_planner node performs the tf2 lookups (tool0 <- camera,
base_link <- aubo_base_link); these helpers convert and compose the resulting
4x4 matrices without a ROS dependency, keeping unit tests offline.
"""

from __future__ import annotations

import math

import numpy as np


def quat_to_matrix_xyzw(
    quat_xyzw: tuple[float, float, float, float],
) -> np.ndarray:
    """(x, y, z, w) quaternion -> 3x3 rotation matrix."""
    x, y, z, w = quat_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def mat_to_quat_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> (x, y, z, w) quaternion (Shepperd's method)."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def transform_from_parts(
    translation: tuple[float, float, float],
    quat_xyzw: tuple[float, float, float, float],
) -> np.ndarray:
    """4x4 rigid transform from translation and (x, y, z, w) quaternion."""
    transform = np.eye(4)
    transform[:3, :3] = quat_to_matrix_xyzw(quat_xyzw)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def invert_rigid(transform: np.ndarray) -> np.ndarray:
    """Analytic inverse of a rigid 4x4 transform (R^T, -R^T t)."""
    inverse = np.eye(4)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -(transform[:3, :3].T @ transform[:3, 3])
    return inverse


def compose_tool0_from_camera(
    camera_pose_base: np.ndarray, tool0_in_camera: np.ndarray
) -> np.ndarray:
    """tool0_pose_base = camera_pose_base @ (tool0 -> camera)^-1.

    `tool0_in_camera` is the full transform resolved from TF
    (target=tool0, source=camera_color_optical_frame), which already includes
    the URDF fixed chain plus the hand-eye extrinsics static TF.
    """
    return camera_pose_base @ invert_rigid(tool0_in_camera)


def to_aubo_frame(pose_base: np.ndarray, base_in_aubo: np.ndarray) -> np.ndarray:
    """Express `pose_base` (a pose in base_link) in aubo_base_link.

    `base_in_aubo` is the 4x4 transform of base_link in aubo_base_link
    (resolved from TF). Full pose transform, including orientation.
    """
    return base_in_aubo @ pose_base
