"""Viewpoint strategy abstraction and registry.

Strategies are pure functions of the target geometry: no ROS, no hardware.
They produce camera poses (position + orientation) in the base_link frame.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar


@dataclass(frozen=True)
class ViewpointSample:
    """A sampled camera pose in the base_link frame.

    camera_quat_xyzw is the orientation such that the camera optical axis
    (+z of camera_color_optical_frame) points at the target.
    """

    camera_xyz_base: tuple[float, float, float]
    camera_quat_xyzw: tuple[float, float, float, float]


class ViewpointStrategy(ABC):
    """Base class for viewpoint sampling strategies."""

    name: ClassVar[str] = ""

    @abstractmethod
    def plan(
        self,
        target_xyz_base: tuple[float, float, float],
        radius_m: float,
        num_viewpoints: int,
        up_vector_base: tuple[float, float, float] = (0.0, 0.0, 1.0),
        **kwargs,
    ) -> list[ViewpointSample]:
        """Sample `num_viewpoints` camera poses on a sphere of `radius_m`
        centered on `target_xyz_base`, with `up_vector_base` as the polar
        reference axis."""
        raise NotImplementedError


STRATEGIES: dict[str, type[ViewpointStrategy]] = {}


def register_strategy(cls: type[ViewpointStrategy]) -> type[ViewpointStrategy]:
    """Register a strategy class by its `name` class attribute."""
    if not cls.name:
        raise ValueError(f"strategy {cls.__name__} must define a non-empty `name`")
    if cls.name in STRATEGIES:
        raise ValueError(f"strategy already registered: {cls.name}")
    STRATEGIES[cls.name] = cls
    return cls


def get_strategy(name: str) -> ViewpointStrategy:
    """Instantiate the registered strategy named `name`."""
    try:
        return STRATEGIES[name]()
    except KeyError:
        available = ", ".join(sorted(STRATEGIES)) or "(none)"
        raise ValueError(f"unknown strategy {name!r}; available strategies: {available}")
