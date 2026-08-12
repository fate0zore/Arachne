"""IK reachability filtering for sampled viewpoints.

Pure numpy (no ROS) so it can be unit-tested offline. TF lookups
(tool0 <- camera, base_link <- aubo_base_link) happen in the node; this
module works on tool0 poses already expressed in the aubo_base_link frame.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from arachne_operator.kinematics import AUBO_I5_JOINT_LIMITS, AuboI5Kinematics


@dataclass(frozen=True)
class IkParams:
    """Numerical IK parameters, mirroring grasp_preview_pipeline constants."""

    position_tolerance: float = 0.005
    orientation_tolerance: float = 0.01
    damping: float = 0.08
    # 100 (not 180): unreachable targets iterate the full budget per pose
    # (~0.6-0.9 s each at this tolerance), so 32 unreachable poses would
    # exceed the service timeout at 180. Reachable poses converge well below 100.
    max_iterations: int = 100
    max_step: float = 0.05
    orientation_weight: float = 0.5


@dataclass
class IkResult:
    """Result of an IK reachability check for one tool0 pose."""

    ok: bool
    q: np.ndarray | None  # 6 joint angles when reachable, else None
    position_error: float
    orientation_error: float


class IkReachabilityFilter:
    """Solves IK for tool0 poses (aubo_base_link) and checks joint limits.

    `AuboI5Kinematics.solve_pose` is a numerical IK that does not enforce
    joint limits, so reachability also requires the solution to fall inside
    the nominal Aubo i5 limits (AUBO_I5_JOINT_LIMITS by default; override
    via the node parameter).
    """

    def __init__(
        self,
        kinematics: AuboI5Kinematics | None = None,
        joint_limits: tuple[tuple[float, float], ...] = AUBO_I5_JOINT_LIMITS,
        params: IkParams | None = None,
    ) -> None:
        self.kinematics = kinematics or AuboI5Kinematics()
        self.joint_limits = tuple(tuple(limit) for limit in joint_limits)
        self.params = params or IkParams()

    def check(self, tool0_pose_aubo: np.ndarray, q_seed: np.ndarray) -> IkResult:
        """Solve IK for one tool0 pose; success requires limits to hold."""
        ok, q, position_error, orientation_error, _ = self.kinematics.solve_pose(
            np.asarray(q_seed, dtype=float),
            np.asarray(tool0_pose_aubo, dtype=float),
            position_tolerance=self.params.position_tolerance,
            orientation_tolerance=self.params.orientation_tolerance,
            damping=self.params.damping,
            max_iterations=self.params.max_iterations,
            max_step=self.params.max_step,
            orientation_weight=self.params.orientation_weight,
        )
        if ok and self._within_limits(q):
            return IkResult(True, np.array(q, dtype=float), position_error, orientation_error)
        if ok:
            # IK converged but the joint solution is out of limits: not reachable.
            return IkResult(False, None, position_error, orientation_error)
        return IkResult(False, None, position_error, orientation_error)

    def filter_poses(
        self,
        tool0_poses_aubo: list[np.ndarray],
        q_seed: np.ndarray,
        time_budget_sec: float | None = None,
    ) -> list[IkResult]:
        """Serial-seed filtering: each reachable solution seeds the next pose
        (same pattern as grasp_preview_pipeline L5347), keeping consecutive
        solutions continuous.

        ``time_budget_sec`` caps the total solve time: when exhausted, the
        remaining poses are marked unreachable without solving (unreachable
        targets are slow to reject — each runs the full iteration budget).
        """
        seed = np.asarray(q_seed, dtype=float)
        deadline = (
            time.monotonic() + max(float(time_budget_sec), 0.0)
            if time_budget_sec is not None
            else None
        )
        results: list[IkResult] = []
        for pose in tool0_poses_aubo:
            if deadline is not None and time.monotonic() >= deadline:
                results.append(
                    IkResult(False, None, float("inf"), float("inf"))
                )
                continue
            result = self.check(pose, seed)
            if result.ok:
                seed = result.q
            results.append(result)
        return results

    def _within_limits(self, q: np.ndarray) -> bool:
        for angle, (lower, upper) in zip(q, self.joint_limits):
            if angle < lower or angle > upper:
                return False
        return True
