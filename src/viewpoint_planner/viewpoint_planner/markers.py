"""Build RViz MarkerArray for viewpoint plans (pure functions, no ROS node).

The array always starts with a DELETEALL marker so any new plan — successful
or failed — clears the previous visualization. On success it adds a target
sphere and, per viewpoint, one arrow plus one index label. Arrow/label color
is green (IK-reachable & collision-free), amber (IK-reachable but colliding)
or red (not IK-reachable), and every label shows the viewpoint's 0-based index
matching the teach_panel list row.
"""

from __future__ import annotations

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from viewpoint_planner_msgs.msg import Viewpoint
from viewpoint_planner_msgs.srv import PlanViewpoints
from visualization_msgs.msg import Marker, MarkerArray

from viewpoint_planner.tf_utils import quat_to_matrix_xyzw

NAMESPACE = "viewpoint_plan"
LABEL_NAMESPACE = "viewpoint_labels"

# Arrow shaft is drawn from the camera position along the optical axis.
ARROW_LENGTH_FRACTION = 0.35  # arrow length = radius * this
TARGET_SPHERE_DIAMETER = 0.08  # m (visible at default RViz zoom)
ARROW_SHAFT_DIAMETER = 0.025  # m
ARROW_HEAD_DIAMETER = 0.06  # m
ARROW_HEAD_LENGTH = 0.1  # m
LABEL_DEFAULT_OFFSET_Z = 0.08  # m, index text above the camera origin
LABEL_DEFAULT_SIZE = 0.06  # m, TEXT_VIEW_FACING font height

_COLOR_REACHABLE = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.9)
_COLOR_UNREACHABLE = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)
_COLOR_COLLISION = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)
_COLOR_TARGET = ColorRGBA(r=1.0, g=0.9, b=0.1, a=0.9)


def build_viewpoint_marker_array(
    request: PlanViewpoints.Request,
    response: PlanViewpoints.Response,
    frame_id: str,
    label_enabled: bool = True,
    label_offset_z: float = LABEL_DEFAULT_OFFSET_Z,
    label_size: float = LABEL_DEFAULT_SIZE,
) -> MarkerArray:
    """Build the marker array for one plan result in ``frame_id``.

    ``label_enabled`` adds one TEXT_VIEW_FACING index marker per viewpoint
    (0-based, matching the teach_panel list row).
    """
    markers = MarkerArray()
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.ns = NAMESPACE
    clear.action = Marker.DELETEALL
    # -1: DELETEALL ignores its id, and -1 never collides with the ADD
    # markers below (target id=0, arrows 1..N, labels 0..N-1 in a separate
    # namespace). A duplicate (ns, id) inside one MarkerArray makes RViz
    # raise a "Duplicate Marker Check" error.
    clear.id = -1
    markers.markers.append(clear)

    if not response.success:
        return markers

    markers.markers.append(_target_marker(frame_id, request.target_xyz_base))
    radius = max(float(request.radius_m), 0.05)
    arrow_length = radius * ARROW_LENGTH_FRACTION
    for index, viewpoint in enumerate(response.viewpoints, start=1):
        markers.markers.append(
            _viewpoint_arrow(frame_id, index, viewpoint, arrow_length)
        )
        if label_enabled:
            markers.markers.append(
                _viewpoint_label(
                    frame_id,
                    index - 1,
                    viewpoint,
                    label_offset_z,
                    label_size,
                )
            )
    return markers


def _target_marker(frame_id: str, target_xyz: tuple[float, float, float]) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = NAMESPACE
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = 0
    marker.pose.position = _point_from_xyz(target_xyz)
    marker.pose.orientation.w = 1.0
    marker.scale = Vector3(
        x=TARGET_SPHERE_DIAMETER, y=TARGET_SPHERE_DIAMETER, z=TARGET_SPHERE_DIAMETER
    )
    marker.color = _COLOR_TARGET
    return marker


def _viewpoint_arrow(
    frame_id: str, marker_id: int, viewpoint: Viewpoint, length: float
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = NAMESPACE
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.id = marker_id
    origin = _point_from_xyz(viewpoint.camera_xyz_base)
    direction = _optical_axis(viewpoint.camera_quat_xyzw)
    marker.points.append(origin)
    marker.points.append(
        Point(
            x=origin.x + direction[0] * length,
            y=origin.y + direction[1] * length,
            z=origin.z + direction[2] * length,
        )
    )
    marker.scale = Vector3(
        x=ARROW_SHAFT_DIAMETER, y=ARROW_HEAD_DIAMETER, z=ARROW_HEAD_LENGTH
    )
    marker.color = _viewpoint_color(viewpoint)
    return marker


def _viewpoint_label(
    frame_id: str,
    label_id: int,
    viewpoint: Viewpoint,
    offset_z: float,
    size: float,
) -> Marker:
    """Index text (0-based) above the camera origin, colored like the arrow."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = LABEL_NAMESPACE
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.id = label_id
    marker.pose.position = _point_from_xyz(viewpoint.camera_xyz_base)
    marker.pose.position.z += float(offset_z)
    marker.pose.orientation.w = 1.0
    marker.scale.z = float(size)  # TEXT_VIEW_FACING font height in meters
    marker.color = _viewpoint_color(viewpoint)
    marker.text = str(label_id)
    return marker


def _viewpoint_color(viewpoint: Viewpoint) -> ColorRGBA:
    """Green = reachable & collision-free; amber = ik-reachable but colliding;
    red = not ik-reachable."""
    if not viewpoint.ik_reachable:
        return _COLOR_UNREACHABLE
    if not viewpoint.collision_free:
        return _COLOR_COLLISION
    return _COLOR_REACHABLE


def _optical_axis(
    quat_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Camera optical axis (+z of camera_color_optical_frame) in the marker frame."""
    rotation = quat_to_matrix_xyzw(quat_xyzw)
    return float(rotation[0, 2]), float(rotation[1, 2]), float(rotation[2, 2])


def _point_from_xyz(xyz: tuple[float, float, float]) -> Point:
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
