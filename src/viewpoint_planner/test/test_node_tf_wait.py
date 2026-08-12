"""Offline tests for the TF startup race guard in viewpoint_planner_node.

Covers _lookup_transform_wait polling logic only (no ROS graph / hardware):
immediate success, transient failure then success, and timeout behaviour.
"""

import time

import pytest
from rclpy.duration import Duration
from tf2_ros import TransformException

from viewpoint_planner.viewpoint_planner_node import ViewpointPlannerNode


class _FakeBuffer:
    def __init__(self, fail_times: int) -> None:
        self.fail_times = fail_times
        self.calls = 0

    def lookup_transform(self, target_frame, source_frame, _time):  # noqa: ANN001
        self.calls += 1
        if self.calls <= self.fail_times:
            raise TransformException("frame does not exist")
        return f"{target_frame}<-{source_frame}"


def _node_with(buffer: _FakeBuffer) -> ViewpointPlannerNode:
    node = ViewpointPlannerNode.__new__(ViewpointPlannerNode)
    node._tf_buffer = buffer
    return node


def test_lookup_waits_then_succeeds() -> None:
    node = _node_with(_FakeBuffer(fail_times=2))
    result = node._lookup_transform_wait("tool0", "camera", Duration(seconds=2.0))
    assert result == "tool0<-camera"
    assert node._tf_buffer.calls == 3  # noqa: SLF001


def test_lookup_immediate_success() -> None:
    node = _node_with(_FakeBuffer(fail_times=0))
    result = node._lookup_transform_wait(
        "aubo_base_link", "base_link", Duration(seconds=2.0)
    )
    assert result == "aubo_base_link<-base_link"
    assert node._tf_buffer.calls == 1  # noqa: SLF001


def test_lookup_raises_after_timeout() -> None:
    node = _node_with(_FakeBuffer(fail_times=10**6))
    start = time.monotonic()
    with pytest.raises(TransformException):
        node._lookup_transform_wait("tool0", "camera", Duration(seconds=0.15))
    assert time.monotonic() - start < 2.0  # bounded wait, not a hang
