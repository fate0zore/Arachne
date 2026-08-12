"""Aubo i5 FK/IK kinematics (aubo_base_link -> tool0).

Extracted from real_hardware_acceptance_test.py so planning/filtering nodes
can reuse the same kinematics without depending on the acceptance-test module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Joint names as published on /joint_states (no aubo_ prefix).
REAL_ARM_JOINT_NAMES = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)

# Aubo i5 nominal joint position limits (rad), in AuboI5Kinematics.JOINTS order
# (aubo_ prefix). The URDF vendor xacro only declares a loose +/-2*pi; these
# are the official nominal ranges used for IK reachability checks.
AUBO_I5_JOINT_LIMITS = (
    (-2.618, 2.618),  # aubo_shoulder_joint
    (-2.079, 2.079),  # aubo_upperArm_joint
    (-3.142, 3.142),  # aubo_foreArm_joint
    (-2.443, 2.443),  # aubo_wrist1_joint
    (-2.443, 2.443),  # aubo_wrist2_joint
    (-3.142, 3.142),  # aubo_wrist3_joint
)


@dataclass(frozen=True)
class RevoluteJoint:
    name: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


class AuboI5Kinematics:
    """FK/IK for the Aubo i5 chain from aubo_base_link to tool0."""

    LINK_NAMES = (
        "aubo_shoulder_Link",
        "aubo_upperArm_Link",
        "aubo_foreArm_Link",
        "aubo_wrist1_Link",
        "aubo_wrist2_Link",
        "aubo_wrist3_Link",
    )

    JOINTS = (
        RevoluteJoint(
            "aubo_shoulder_joint",
            (0.0, 0.0, 0.122),
            (0.0, 0.0, math.pi),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_upperArm_joint",
            (0.0, 0.1215, 0.0),
            (-math.pi / 2.0, -math.pi / 2.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_foreArm_joint",
            (0.408, 0.0, 0.0),
            (-math.pi, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_wrist1_joint",
            (0.376, 0.0, 0.0),
            (math.pi, 0.0, math.pi / 2.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_wrist2_joint",
            (0.0, 0.1025, 0.0),
            (-math.pi / 2.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        RevoluteJoint(
            "aubo_wrist3_joint",
            (0.0, -0.094, 0.0),
            (math.pi / 2.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )

    TOOL0_FIXED_RPY = (0.0, 0.0, math.pi / 2.0)

    def fk(self, q: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        for joint, angle in zip(self.JOINTS, q):
            transform = transform @ self._origin(joint.xyz, joint.rpy)
            transform = transform @ self._axis_rotation(joint.axis, float(angle))
        return transform @ self._origin((0.0, 0.0, 0.0), self.TOOL0_FIXED_RPY)

    def link_transforms(self, q: np.ndarray) -> list[tuple[str, np.ndarray]]:
        transform = np.eye(4)
        transforms: list[tuple[str, np.ndarray]] = [("aubo_base_link", np.array(transform))]
        for joint, link_name, angle in zip(self.JOINTS, self.LINK_NAMES, q):
            transform = transform @ self._origin(joint.xyz, joint.rpy)
            transform = transform @ self._axis_rotation(joint.axis, float(angle))
            transforms.append((link_name, np.array(transform)))
        transforms.append(
            (
                "tool0",
                np.array(transform @ self._origin((0.0, 0.0, 0.0), self.TOOL0_FIXED_RPY)),
            )
        )
        return transforms

    def solve_position(
        self,
        q_start: np.ndarray,
        target_position: np.ndarray,
        *,
        tolerance: float,
        damping: float,
        max_iterations: int,
        max_step: float,
    ) -> tuple[bool, np.ndarray, float, int]:
        q = np.array(q_start, dtype=float)
        best_q = np.array(q_start, dtype=float)
        best_error = float(np.linalg.norm(self.fk(q)[:3, 3] - target_position))

        for iteration in range(max_iterations):
            current = self.fk(q)[:3, 3]
            error = target_position - current
            norm = float(np.linalg.norm(error))
            if norm < best_error:
                best_q = np.array(q, dtype=float)
                best_error = norm
            if norm <= tolerance:
                return True, q, norm, iteration

            jacobian = self._numeric_position_jacobian(q)
            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(3)
            step = jacobian.T @ np.linalg.solve(lhs, error)
            step = np.clip(step, -max_step, max_step)
            q = self._wrap(q + step)

        return False, best_q, best_error, max_iterations

    def solve_pose(
        self,
        q_start: np.ndarray,
        target_transform: np.ndarray,
        *,
        position_tolerance: float,
        orientation_tolerance: float,
        damping: float,
        max_iterations: int,
        max_step: float,
        orientation_weight: float = 0.5,
    ) -> tuple[bool, np.ndarray, float, float, int]:
        q = np.array(q_start, dtype=float)
        best_q = np.array(q_start, dtype=float)
        best_position_error, best_orientation_error = self._pose_error_norms(q, target_transform)
        best_error = best_position_error + orientation_weight * best_orientation_error

        for iteration in range(max_iterations):
            current_transform = self.fk(q)
            position_error = target_transform[:3, 3] - current_transform[:3, 3]
            orientation_error = self._rotation_vector(
                target_transform[:3, :3] @ current_transform[:3, :3].T
            )
            position_norm = float(np.linalg.norm(position_error))
            orientation_norm = float(np.linalg.norm(orientation_error))
            combined_norm = position_norm + orientation_weight * orientation_norm
            if combined_norm < best_error:
                best_q = np.array(q, dtype=float)
                best_position_error = position_norm
                best_orientation_error = orientation_norm
                best_error = combined_norm
            if position_norm <= position_tolerance and orientation_norm <= orientation_tolerance:
                return True, q, position_norm, orientation_norm, iteration

            error = np.concatenate((position_error, orientation_weight * orientation_error))
            jacobian = self._numeric_pose_jacobian(q, orientation_weight)
            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(6)
            step = jacobian.T @ np.linalg.solve(lhs, error)
            step = np.clip(step, -max_step, max_step)
            q = self._wrap(q + step)

        return False, best_q, best_position_error, best_orientation_error, max_iterations

    def _numeric_position_jacobian(self, q: np.ndarray) -> np.ndarray:
        eps = 1e-5
        base = self.fk(q)[:3, 3]
        jacobian = np.zeros((3, len(q)))
        for index in range(len(q)):
            q_eps = np.array(q, dtype=float)
            q_eps[index] += eps
            jacobian[:, index] = (self.fk(q_eps)[:3, 3] - base) / eps
        return jacobian

    def _numeric_pose_jacobian(self, q: np.ndarray, orientation_weight: float) -> np.ndarray:
        eps = 1e-5
        base_transform = self.fk(q)
        base_position = base_transform[:3, 3]
        base_rotation = base_transform[:3, :3]
        jacobian = np.zeros((6, len(q)))
        for index in range(len(q)):
            q_eps = np.array(q, dtype=float)
            q_eps[index] += eps
            next_transform = self.fk(q_eps)
            jacobian[:3, index] = (next_transform[:3, 3] - base_position) / eps
            jacobian[3:, index] = (
                orientation_weight
                * self._rotation_vector(next_transform[:3, :3] @ base_rotation.T)
                / eps
            )
        return jacobian

    def _pose_error_norms(
        self, q: np.ndarray, target_transform: np.ndarray
    ) -> tuple[float, float]:
        current_transform = self.fk(q)
        position_error = float(
            np.linalg.norm(target_transform[:3, 3] - current_transform[:3, 3])
        )
        orientation_error = float(
            np.linalg.norm(
                self._rotation_vector(target_transform[:3, :3] @ current_transform[:3, :3].T)
            )
        )
        return position_error, orientation_error

    def _rotation_vector(self, rotation: np.ndarray) -> np.ndarray:
        cos_angle = max(min((float(np.trace(rotation)) - 1.0) * 0.5, 1.0), -1.0)
        angle = math.acos(cos_angle)
        vector = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=float,
        )
        if angle < 1e-8:
            return 0.5 * vector
        return angle / (2.0 * math.sin(angle)) * vector

    def _origin(
        self, xyz: tuple[float, float, float], rpy: tuple[float, float, float]
    ) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = self._rpy_matrix(*rpy)
        transform[:3, 3] = np.array(xyz, dtype=float)
        return transform

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
        return rz @ ry @ rx

    def _axis_rotation(self, axis: tuple[float, float, float], angle: float) -> np.ndarray:
        vector = np.array(axis, dtype=float)
        vector = vector / np.linalg.norm(vector)
        x, y, z = vector
        c = math.cos(angle)
        s = math.sin(angle)
        c1 = 1.0 - c
        rotation = np.array(
            [
                [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
                [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
                [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
            ],
            dtype=float,
        )
        transform = np.eye(4)
        transform[:3, :3] = rotation
        return transform

    def _wrap(self, q: np.ndarray) -> np.ndarray:
        return np.array([math.atan2(math.sin(value), math.cos(value)) for value in q])
