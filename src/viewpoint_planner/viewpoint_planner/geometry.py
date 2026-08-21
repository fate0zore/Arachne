"""Pure-numpy collision geometry for viewpoint planning (no ROS).

Parameterized extraction of the box/ground/end-effector collision math that
teach_panel and grasp_preview_pipeline perform inline, plus a new link-pair
self-collision check for the Aubo i5. Everything here is a plain function of
its inputs so it can be unit-tested offline.

Coordinate conventions
----------------------
- ``CollisionBox`` centres/sizes and ground are in ``base_link``.
- The arm joint vector ``q`` is in ``aubo_base_link``; caller supplies
  ``base_from_aubo`` (``base_link <- aubo_base_link``) to move link/tool0
  frames into ``base_link`` before testing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class CollisionBox:
    """Axis-aligned box (``base_link``) for point/box and box/box tests."""

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]

    @property
    def half(self) -> tuple[float, float, float]:
        return tuple(0.5 * value for value in self.size)  # type: ignore[return-value]

    @property
    def minimum(self) -> tuple[float, float, float]:
        return tuple(c - h for c, h in zip(self.center, self.half))

    @property
    def maximum(self) -> tuple[float, float, float]:
        return tuple(c + h for c, h in zip(self.center, self.half))


def _padded(size: tuple[float, float, float], margin: float) -> tuple[float, float, float]:
    pad = max(float(margin), 0.0)
    return tuple(float(value) + 2.0 * pad for value in size)  # type: ignore[return-value]


def box_from_center_size(
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    margin: float = 0.0,
) -> CollisionBox:
    return CollisionBox(name, tuple(float(v) for v in center), _padded(size, margin))


def box_from_min_max(
    name: str,
    minimum: tuple[float, float, float],
    maximum: tuple[float, float, float],
    margin: float = 0.0,
) -> Optional[CollisionBox]:
    """Build a box from min/max corners after padding ``margin`` on every side."""
    mins = np.asarray(minimum, dtype=np.float64)
    maxs = np.asarray(maximum, dtype=np.float64)
    if bool(np.any(maxs <= mins)):
        return None
    center = tuple(float(value) for value in (mins + maxs) * 0.5)
    size = tuple(float(value) for value in (maxs - mins))
    return CollisionBox(name, center, _padded(size, margin))


# Scout vehicle default keepout boxes (``base_link``), mirroring
# grasp_preview_pipeline._make_collision_boxes (margins included).
DEFAULT_SCOUT_BOXES = (
    ("scout_base_main", (0.0, 0.0, 0.008), (0.925, 0.380, 0.210)),
    (
        "scout_base_center_ridge",
        (0.0, 0.0, 0.210 / 6.0),
        (0.925 / 6.0, 0.380 * 1.65, 0.210 / 3.0),
    ),
)

# Default keepout min/max (``base_link``), aligned with
# teach_panel.py:73-77 and grasp_preview_pipeline.py:451-454.
DEFAULT_FRONT_BASKET_KEEPOUT_MIN = (0.4215, -0.11, -0.1235)
DEFAULT_FRONT_BASKET_KEEPOUT_MAX = (0.6655, 0.11, 0.0635)
DEFAULT_REAR_RACK_KEEPOUT_MIN = (-0.41, -0.22, 0.04)
DEFAULT_REAR_RACK_KEEPOUT_MAX = (0.09, 0.22, 0.82)


def default_collision_boxes(
    collision_margin: float = 0.015,
    rear_rack_collision_margin: float = 0.005,
    front_basket_keepout_min: Optional[tuple[float, float, float]] = None,
    front_basket_keepout_max: Optional[tuple[float, float, float]] = None,
    rear_rack_keepout_min: Optional[tuple[float, float, float]] = None,
    rear_rack_keepout_max: Optional[tuple[float, float, float]] = None,
) -> list[CollisionBox]:
    """Scout vehicle collision boxes (base_link): 2 body + basket + rear rack."""
    boxes: list[CollisionBox] = []
    for name, center, size in DEFAULT_SCOUT_BOXES:
        boxes.append(box_from_center_size(name, center, size, collision_margin))

    basket_min = (
        tuple(float(v) for v in front_basket_keepout_min)
        if front_basket_keepout_min is not None
        else DEFAULT_FRONT_BASKET_KEEPOUT_MIN
    )
    basket_max = (
        tuple(float(v) for v in front_basket_keepout_max)
        if front_basket_keepout_max is not None
        else DEFAULT_FRONT_BASKET_KEEPOUT_MAX
    )
    basket = box_from_min_max("front_basket_keepout", basket_min, basket_max, collision_margin)
    if basket is not None:
        boxes.append(basket)

    rack_min = (
        tuple(float(v) for v in rear_rack_keepout_min)
        if rear_rack_keepout_min is not None
        else DEFAULT_REAR_RACK_KEEPOUT_MIN
    )
    rack_max = (
        tuple(float(v) for v in rear_rack_keepout_max)
        if rear_rack_keepout_max is not None
        else DEFAULT_REAR_RACK_KEEPOUT_MAX
    )
    rack = box_from_min_max("rear_sensor_rack", rack_min, rack_max, rear_rack_collision_margin)
    if rack is not None:
        boxes.append(rack)
    return boxes


def box_box_clearance(
    minimum_a: np.ndarray,
    maximum_a: np.ndarray,
    minimum_b: np.ndarray,
    maximum_b: np.ndarray,
) -> float:
    """AABB-AABB distance: positive when separated, negative penetration when overlapping."""
    overlap_min = np.maximum(minimum_a, minimum_b)
    overlap_max = np.minimum(maximum_a, maximum_b)
    if bool(np.all(overlap_min <= overlap_max)):
        return float(-np.min(overlap_max - overlap_min))
    separation = np.maximum(
        np.maximum(minimum_b - maximum_a, minimum_a - maximum_b), 0.0
    )
    return float(np.linalg.norm(separation))


def point_box_clearance(point: np.ndarray, box: CollisionBox) -> float:
    """Distance from a point to an AABB: positive outside, negative penetration inside."""
    center = np.asarray(box.center, dtype=float)
    half = np.asarray(box.half, dtype=float)
    delta = np.abs(np.asarray(point, dtype=float) - center) - half
    outside = np.maximum(delta, 0.0)
    outside_distance = float(np.linalg.norm(outside))
    if outside_distance > 0.0:
        return outside_distance
    return float(np.max(delta))


def ee_boxes_to_base_aabbs(
    ee_boxes_tool0: Iterable[CollisionBox],
    tool_in_base: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Rotate tool0-frame AABBs into base_link and return world min/max corners.

    The tool0 boxes are only axis-aligned in tool0; after rotation they are
    converted back to AABBs (conservative), matching grasp_preview's
    corner-sweep approach that catches slanted penetration.
    """
    rotation = tool_in_base[:3, :3]
    origin = tool_in_base[:3, 3]
    result: list[tuple[np.ndarray, np.ndarray]] = []
    for actor in ee_boxes_tool0:
        center = np.asarray(actor.center, dtype=np.float64)
        half = np.asarray(actor.half, dtype=np.float64)
        corners = np.array(
            [
                [sx * half[0], sy * half[1], sz * half[2]]
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ],
            dtype=np.float64,
        )
        points = corners @ rotation.T + origin
        result.append((points.min(axis=0), points.max(axis=0)))
    return result


def arm_collision_clearance_base(
    q: np.ndarray,
    base_from_aubo: np.ndarray,
    *,
    kinematics,
    boxes: list[CollisionBox],
    arm_collision_radius: float = 0.035,
    arm_collision_samples_per_link: int = 6,
    ground_min_z_base: float = -0.22,
    ground_clearance: float = 0.02,
    ee_camera_collision_enabled: bool = True,
    ee_camera_collision_margin: float = 0.01,
    ee_camera_boxes_tool0_fn: Optional[Callable[[float], list[CollisionBox]]] = None,
) -> tuple[bool, float, Optional[str]]:
    """Arm-vs-environment clearance in base_link.

    Checks every sampled point along the arm links against the ground and each
    box, plus the end-effector eccentric camera box via box/box tests. Returns
    ``(ok, min_clearance, hit_name)`` where ``ok`` is False when any clearance
    goes negative (penetration).
    """
    link_points = [
        (base_from_aubo @ np.append(transform[:3, 3], 1.0))[:3]
        for _name, transform in kinematics.link_transforms(np.asarray(q, dtype=float))
    ]
    tool_in_aubo = kinematics.fk(np.asarray(q, dtype=float))
    tool_in_base = np.asarray(base_from_aubo, dtype=float) @ tool_in_aubo

    samples_per_link = max(int(arm_collision_samples_per_link), 2)
    radius = max(float(arm_collision_radius), 0.0)
    ground_threshold = (
        float(ground_min_z_base)
        + max(float(ground_clearance), 0.0)
        + radius
    )
    min_clearance = float("inf")
    hit_name: Optional[str] = None

    def check_point(point: np.ndarray) -> bool:
        """Ground/box check for one link point; updates minima, False on hit."""
        nonlocal min_clearance, hit_name
        ground_clearance_value = float(point[2] - ground_threshold)
        if ground_clearance_value < min_clearance:
            min_clearance = ground_clearance_value
            hit_name = "ground/arm"
        if ground_clearance_value < 0.0:
            return False
        for actor in boxes:
            clearance = point_box_clearance(point, actor) - radius
            if clearance < min_clearance:
                min_clearance = float(clearance)
                hit_name = actor.name
            if clearance < 0.0:
                return False
        return True

    # Arm links (skip base link index 0): consecutive link origins form the
    # arm chain, sampled along each segment.
    for start, end in zip(link_points[1:], link_points[2:]):
        a = np.asarray(start, dtype=float)
        b = np.asarray(end, dtype=float)
        for t in np.linspace(0.0, 1.0, samples_per_link):
            if not check_point(a * (1.0 - t) + b * t):
                return False, float(min_clearance), hit_name

    # End-effector camera/support eccentric envelope (tool0 frame): rotate to
    # base_link and test box-vs-box (point sampling misses slanted penetration).
    if ee_camera_collision_enabled:
        ee_margin = max(float(ee_camera_collision_margin), 0.0)
        ee_boxes = (
            ee_camera_boxes_tool0_fn(ee_margin)
            if ee_camera_boxes_tool0_fn is not None
            else []
        )
        for ee_minimum, ee_maximum in ee_boxes_to_base_aabbs(ee_boxes, tool_in_base):
            ground_clearance_value = float(ee_minimum[2] - ground_threshold)
            if ground_clearance_value < min_clearance:
                min_clearance = ground_clearance_value
                hit_name = "ground/arm"
            if ground_clearance_value < 0.0:
                return False, float(min_clearance), hit_name
            for actor in boxes:
                box_center = np.asarray(actor.center, dtype=np.float64)
                box_half = np.asarray(actor.half, dtype=np.float64)
                clearance = box_box_clearance(
                    ee_minimum,
                    ee_maximum,
                    box_center - box_half,
                    box_center + box_half,
                )
                if clearance < min_clearance:
                    min_clearance = float(clearance)
                    hit_name = actor.name
                if clearance < 0.0:
                    return False, float(min_clearance), hit_name
    return True, float(min_clearance if np.isfinite(min_clearance) else 0.0), hit_name


def _closest_distance_between_segments(
    p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray
) -> float:
    """Min distance between two finite 3D line segments (p1-p2, p3-p4)."""
    d1 = p2 - p1
    d2 = p4 - p3
    r = p1 - p3
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(p1 - p3))
    if a <= 1e-12:
        s = 0.0
        t = float(np.clip(f / e, 0.0, 1.0))
    else:
        c = float(np.dot(d1, r))
        if e <= 1e-12:
            t = 0.0
            s = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            s = (
                float(np.clip((b * f - c * e) / denom, 0.0, 1.0))
                if denom > 1e-12
                else 0.0
            )
            t = float(np.clip((b * s + f) / e, 0.0, 1.0))
            # clamp again and recompute s per standard algorithm
            if t < 0.0:
                t = 0.0
                s = float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t = 1.0
                s = float(np.clip((b - c) / a, 0.0, 1.0))
    return float(np.linalg.norm(p1 + d1 * s - (p3 + d2 * t)))


def self_collision_clearance(
    q: np.ndarray,
    base_from_aubo: np.ndarray,
    *,
    kinematics,
    link_radius: float = 0.035,
    min_clearance_threshold: float = 0.0,
) -> tuple[bool, float, Optional[tuple[str, str]]]:
    """Link-link self-collision check (Aubo i5).

    Builds the 7-segment arm chain from ``link_transforms`` (base->L1->
    ...->L6->tool0) and tests every non-adjacent pair (chain index gap >= 2)
    for segment distance < ``2 * link_radius``. Adjacent segments share a
    joint and are exempt. Returns ``(ok, min_clearance, (n1, n2))`` where
    min_clearance is the closest pair distance reduced by 2*radius (negative =
    self-collision).
    """
    names, transforms = zip(
        *kinematics.link_transforms(np.asarray(q, dtype=float))
    )  # base, L1..L6, tool0
    points_world = [
        (np.asarray(base_from_aubo, dtype=float) @ np.append(transform[:3, 3], 1.0))[:3]
        for transform in transforms
    ]
    radius = max(float(link_radius), 0.0)
    min_distance = float("inf")
    hit_pair: Optional[tuple[str, str]] = None
    count = len(points_world)
    # segments[gap] spans points[gap] -> points[gap+1] (gap in [0, count-2])
    for i in range(count - 2):
        for j in range(i + 2, count - 1):
            if j - i < 2:
                continue
            distance = _closest_distance_between_segments(
                points_world[i], points_world[i + 1],
                points_world[j], points_world[j + 1],
            )
            clearance = distance - 2.0 * radius
            if clearance < min_distance:
                min_distance = clearance
                hit_pair = (names[i + 1], names[j + 1])
    ok = min_distance >= float(min_clearance_threshold)
    return ok, float(min_distance), hit_pair
