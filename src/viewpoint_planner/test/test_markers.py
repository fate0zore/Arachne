"""Offline tests for the RViz MarkerArray builder (no ROS graph / hardware)."""

from types import SimpleNamespace as NS
from unittest.mock import MagicMock

import pytest
from visualization_msgs.msg import Marker, MarkerArray

from viewpoint_planner.markers import (
    ARROW_HEAD_LENGTH,
    ARROW_LENGTH_FRACTION,
    LABEL_NAMESPACE,
    NAMESPACE,
    TARGET_SPHERE_DIAMETER,
    build_viewpoint_marker_array,
    quat_to_matrix_xyzw,
)
from viewpoint_planner.viewpoint_planner_node import ViewpointPlannerNode


def _viewpoint(
    camera_xyz,
    quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    ik_reachable=True,
    collision_free=True,
):
    return NS(
        camera_xyz_base=list(camera_xyz),
        camera_quat_xyzw=list(quat_xyzw),
        ik_reachable=ik_reachable,
        collision_free=collision_free,
        collision_clearance=0.02,
    )


def _request():
    return NS(
        target_xyz_base=[0.3, 0.0, 0.2],
        radius_m=0.5,
    )


def _response(success=True, viewpoints=None):
    return NS(success=success, viewpoints=viewpoints or [])


def _arrows(markers):
    # The DELETEALL marker shares ns=viewpoint_plan and its default type is the
    # ARROW enum value (0), so require action == ADD as well.
    return [
        m
        for m in markers.markers
        if m.action == Marker.ADD and m.ns == NAMESPACE and m.type == Marker.ARROW
    ]


def _labels(markers):
    return [m for m in markers.markers if m.ns == LABEL_NAMESPACE]


def test_first_marker_is_deleteall():
    markers = build_viewpoint_marker_array(_request(), _response(), "base_link")
    assert markers.markers[0].action == Marker.DELETEALL
    assert markers.markers[0].ns == NAMESPACE
    assert markers.markers[0].header.frame_id == "base_link"
    # -1 keeps DELETEALL from colliding with ADD markers (target id=0, arrows 1..N)
    assert markers.markers[0].id == -1


def test_no_duplicate_ns_id_in_array():
    vps = [
        _viewpoint([0.1, 0.0, 0.5]),
        _viewpoint([0.2, 0.1, 0.5], ik_reachable=False),
    ]
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=vps), "base_link"
    )
    pairs = [(m.ns, m.id) for m in markers.markers]
    assert len(pairs) == len(set(pairs))  # RViz "Duplicate Marker Check" would error


def test_success_target_sphere():
    markers = build_viewpoint_marker_array(_request(), _response(), "base_link")
    target = markers.markers[1]
    assert target.type == Marker.SPHERE
    assert target.id == 0
    assert (target.pose.position.x, target.pose.position.y, target.pose.position.z) == (
        0.3,
        0.0,
        0.2,
    )
    assert target.scale.x == TARGET_SPHERE_DIAMETER
    assert target.color.a == pytest.approx(0.9)


def test_arrow_count_and_ids():
    vps = [
        _viewpoint([0.1, 0.0, 0.5]),
        _viewpoint([0.2, 0.1, 0.5], ik_reachable=False),
        _viewpoint([-0.1, 0.2, 0.4]),
    ]
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=vps), "base_link"
    )
    arrows = _arrows(markers)
    assert len(arrows) == 3
    assert [m.id for m in arrows] == [1, 2, 3]
    assert all(m.ns == NAMESPACE for m in arrows)


def test_label_count_ids_and_text():
    vps = [
        _viewpoint([0.1, 0.0, 0.5]),
        _viewpoint([0.2, 0.1, 0.5], ik_reachable=False),
        _viewpoint([-0.1, 0.2, 0.4]),
    ]
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=vps), "base_link"
    )
    labels = _labels(markers)
    assert len(labels) == 3
    assert [m.id for m in labels] == [0, 1, 2]
    assert [m.text for m in labels] == ["0", "1", "2"]
    assert all(m.type == Marker.TEXT_VIEW_FACING for m in labels)
    assert all(m.ns == LABEL_NAMESPACE for m in labels)


def test_label_position_offset_from_camera():
    vp = _viewpoint([0.3, 0.0, 0.5])
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=[vp]), "base_link", label_offset_z=0.08
    )
    label = _labels(markers)[0]
    pos = label.pose.position
    assert (pos.x, pos.y) == (0.3, 0.0)
    assert pos.z == pytest.approx(0.5 + 0.08)
    assert label.scale.z == pytest.approx(0.06)  # default font height


def test_label_color_follows_viewpoint():
    vps = [
        _viewpoint([0.1, 0.0, 0.5], ik_reachable=True, collision_free=True),
        _viewpoint([0.2, 0.1, 0.5], ik_reachable=True, collision_free=False),
        _viewpoint([-0.1, 0.2, 0.4], ik_reachable=False),
    ]
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=vps), "base_link"
    )
    green, amber, red = _labels(markers)
    assert green.color.g > 0.9 and green.color.r < 0.2
    assert amber.color.r > 0.9 and amber.color.g > 0.7  # orange-ish
    assert red.color.r > 0.9 and red.color.g < 0.2


def test_labels_disabled_omits_labels():
    vps = [_viewpoint([0.1, 0.0, 0.5]), _viewpoint([0.2, 0.1, 0.5])]
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=vps), "base_link", label_enabled=False
    )
    assert _labels(markers) == []
    assert len(_arrows(markers)) == 2


def test_arrow_points_along_optical_axis():
    # identity orientation -> optical axis is +z
    vp = _viewpoint([0.3, 0.0, 0.5], quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=[vp]), "base_link"
    )
    arrow = _arrows(markers)[0]
    start, end = arrow.points[0], arrow.points[1]
    assert (start.x, start.y, start.z) == (0.3, 0.0, 0.5)
    length = 0.5 * ARROW_LENGTH_FRACTION
    assert end.x == pytest.approx(0.3)
    assert end.y == pytest.approx(0.0)
    assert end.z == pytest.approx(0.5 + length)


def test_arrow_direction_from_quaternion():
    # quat (0, 1, 0, 0) rotates 180 deg about y: optical +z -> -z
    vp = _viewpoint([0.0, 0.0, 0.0], quat_xyzw=(0.0, 1.0, 0.0, 0.0))
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=[vp]), "base_link"
    )
    arrow = _arrows(markers)[0]
    end = arrow.points[1]
    assert end.z == pytest.approx(-0.5 * ARROW_LENGTH_FRACTION)
    rot = quat_to_matrix_xyzw((0.0, 1.0, 0.0, 0.0))
    assert rot[2, 2] == pytest.approx(-1.0)


def test_colors_follow_reachability():
    vps = [
        _viewpoint([0.1, 0.0, 0.5], ik_reachable=True),
        _viewpoint([0.2, 0.1, 0.5], ik_reachable=False),
    ]
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=vps), "base_link"
    )
    green, red = _arrows(markers)[:2]
    assert green.color.g > 0.9 and green.color.r < 0.2
    assert red.color.r > 0.9 and red.color.g < 0.2


def test_failed_response_only_clears():
    markers = build_viewpoint_marker_array(
        _request(), _response(success=False), "base_link"
    )
    assert len(markers.markers) == 1
    assert markers.markers[0].action == Marker.DELETEALL


def test_empty_viewpoints_keeps_target():
    markers = build_viewpoint_marker_array(
        _request(), _response(success=True), "base_link"
    )
    assert len(markers.markers) == 2  # DELETEALL + target


def test_arrow_head_scale():
    vp = _viewpoint([0.1, 0.0, 0.5])
    markers = build_viewpoint_marker_array(
        _request(), _response(viewpoints=[vp]), "base_link"
    )
    arrow = _arrows(markers)[0]
    assert arrow.scale.z == ARROW_HEAD_LENGTH


def _node_with_params(params):
    defaults = {
        "publish_markers": True,
        "markers_frame": "base_link",
        "marker_label_enabled": True,
        "marker_label_offset_z": 0.08,
        "marker_label_size": 0.06,
    }
    node = ViewpointPlannerNode.__new__(ViewpointPlannerNode)
    node.get_parameter = lambda name: (
        NS(value=params[name]) if name in params else NS(value=defaults[name])
    )
    node._marker_pub = MagicMock()  # noqa: SLF001
    return node


def test_publish_markers_disabled_skips_publish():
    node = _node_with_params({"publish_markers": False, "markers_frame": "base_link"})
    node._publish_markers(_request(), _response(success=True))  # noqa: SLF001
    node._marker_pub.publish.assert_not_called()  # noqa: SLF001


def test_publish_markers_enabled_publishes_array():
    node = _node_with_params({"publish_markers": True, "markers_frame": "base_link"})
    node._publish_markers(_request(), _response(success=True))  # noqa: SLF001
    node._marker_pub.publish.assert_called_once()  # noqa: SLF001
    payload = node._marker_pub.publish.call_args.args[0]
    assert isinstance(payload, MarkerArray)
    assert payload.markers[0].action == Marker.DELETEALL
    assert len(_labels(payload)) == 0  # empty plan -> target + deleteall only


def test_republish_cb_sends_cached_array():
    node = _node_with_params({"publish_markers": True, "markers_frame": "base_link"})
    node._last_marker_array = None
    node._marker_republish_cb()  # noqa: SLF001
    node._marker_pub.publish.assert_not_called()  # nothing planned yet
    node._publish_markers(_request(), _response(success=True))  # noqa: SLF001
    node._marker_pub.reset_mock()
    node._marker_republish_cb()  # noqa: SLF001
    assert node._marker_pub.publish.call_count == 1
    assert node._marker_pub.publish.call_args.args[0] is node._last_marker_array
