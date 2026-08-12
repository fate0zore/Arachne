"""Viewpoint sampling strategies. Importing this package registers built-ins."""

from viewpoint_planner.strategies.base import (
    STRATEGIES,
    ViewpointSample,
    ViewpointStrategy,
    get_strategy,
    register_strategy,
)
from viewpoint_planner.strategies import fibonacci_sphere  # noqa: F401  (registration)

__all__ = [
    "STRATEGIES",
    "ViewpointSample",
    "ViewpointStrategy",
    "get_strategy",
    "register_strategy",
]
