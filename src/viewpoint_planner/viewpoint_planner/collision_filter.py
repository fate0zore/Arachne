"""Collision filtering for sampled viewpoints (pure numpy, no ROS).

Runs after IK reachability: given the per-pose joint solutions from the IK
filter, checks each reachable pose against the scout keepout boxes, the
ground, the end-effector camera envelope and the arm's own links. Mirrors the
``ik_filter`` design so it can be unit-tested offline.

Semantics (see phase2 plan §2.4): collision checks only run for points that
have a valid ``q`` (IK-reachable). Unreachable points get ``collision_free``
False with a sentinel clearance of 0.0 (no collision info).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from arachne_operator.ee_collision import ee_camera_boxes_tool0
from arachne_operator.kinematics import AuboI5Kinematics
from viewpoint_planner.geometry import (
    CollisionBox,
    arm_collision_clearance_base,
    default_collision_boxes,
    self_collision_clearance,
)


@dataclass(frozen=True)
class CollisionParams:
    """Collision-filter parameters, mirroring grasp_preview_pipeline defaults."""

    arm_collision_radius: float = 0.035
    arm_collision_samples_per_link: int = 6
    ground_min_z_base: float = -0.22
    ground_clearance: float = 0.02
    ee_camera_collision_enabled: bool = True
    ee_camera_collision_margin: float = 0.01
    self_collision_enabled: bool = True
    self_collision_radius: float = 0.035
    collision_margin: float = 0.015
    rear_rack_collision_margin: float = 0.005
    front_basket_keepout_min: Optional[tuple[float, float, float]] = None
    front_basket_keepout_max: Optional[tuple[float, float, float]] = None
    rear_rack_keepout_min: Optional[tuple[float, float, float]] = None
    rear_rack_keepout_max: Optional[tuple[float, float, float]] = None


@dataclass
class CollisionResult:
    """Outcome of one collision check for one viewpoint."""

    collision_free: bool
    clearance: float
    hit_name: Optional[str]
    reason: str = "checked"


class CollisionFilter:
    """Checks joint solutions against environment/ground/self geometry."""

    def __init__(
        self,
        kinematics: AuboI5Kinematics | None = None,
        params: CollisionParams | None = None,
        boxes: Optional[list[CollisionBox]] = None,
        ee_camera_boxes_tool0_fn: Callable[[float], list[CollisionBox]] | None = None,
    ) -> None:
        self.kinematics = kinematics or AuboI5Kinematics()
        self.params = params or CollisionParams()
        self.boxes = (
            boxes
            if boxes is not None
            else default_collision_boxes(
                collision_margin=self.params.collision_margin,
                rear_rack_collision_margin=self.params.rear_rack_collision_margin,
                front_basket_keepout_min=self.params.front_basket_keepout_min,
                front_basket_keepout_max=self.params.front_basket_keepout_max,
                rear_rack_keepout_min=self.params.rear_rack_keepout_min,
                rear_rack_keepout_max=self.params.rear_rack_keepout_max,
            )
        )
        self.ee_camera_boxes_tool0_fn = ee_camera_boxes_tool0_fn or ee_camera_boxes_tool0

    def check(self, q: Any, base_from_aubo: Any) -> CollisionResult:
        """Run the full A-level check for one joint solution ``q``.

        ``q`` is a 6-vector in ``aubo_base_link``; ``base_from_aubo`` is
        ``base_link <- aubo_base_link``. Boxes/ground are in ``base_link``.
        """
        arm_ok, arm_clearance, arm_hit = arm_collision_clearance_base(
            q,
            base_from_aubo,
            kinematics=self.kinematics,
            boxes=self.boxes,
            arm_collision_radius=self.params.arm_collision_radius,
            arm_collision_samples_per_link=self.params.arm_collision_samples_per_link,
            ground_min_z_base=self.params.ground_min_z_base,
            ground_clearance=self.params.ground_clearance,
            ee_camera_collision_enabled=self.params.ee_camera_collision_enabled,
            ee_camera_collision_margin=self.params.ee_camera_collision_margin,
            ee_camera_boxes_tool0_fn=self.ee_camera_boxes_tool0_fn,
        )
        self_ok = True
        self_clearance = float("inf")
        self_hit: Optional[tuple[str, str]] = None
        if self.params.self_collision_enabled:
            self_ok, self_clearance, self_hit = self_collision_clearance(
                q,
                base_from_aubo,
                kinematics=self.kinematics,
                link_radius=self.params.self_collision_radius,
            )
        clearance = float(min(arm_clearance, self_clearance))
        collision_free = bool(arm_ok and self_ok)
        if self_hit is not None and self_clearance < arm_clearance:
            hit_name: Optional[str] = f"self:{self_hit[0]}/{self_hit[1]}"
        else:
            hit_name = arm_hit
        return CollisionResult(collision_free, clearance, hit_name)

    def filter_joints(
        self,
        ik_results: list[Any],
        base_from_aubo: Any,
        *,
        time_budget_sec: float | None = None,
    ) -> list[CollisionResult]:
        """Collision-check each IK result; only points with a valid ``q`` run.

        Points that were not IK-reachable get ``collision_free=False`` with no
        collision info (sentinel clearance 0.0). ``time_budget_sec`` caps the
        total check time; when exhausted the remaining points are marked
        ``collision_free=False`` (reason "budget").
        """
        deadline = (
            time.monotonic() + max(float(time_budget_sec), 0.0)
            if time_budget_sec is not None
            else None
        )
        results: list[CollisionResult] = []
        for result in ik_results:
            if not getattr(result, "ok", False) or getattr(result, "q", None) is None:
                results.append(
                    CollisionResult(False, 0.0, None, reason="ik_unreachable")
                )
                continue
            if deadline is not None and time.monotonic() >= deadline:
                results.append(CollisionResult(False, 0.0, None, reason="budget"))
                continue
            results.append(self.check(result.q, base_from_aubo))
        return results
