"""Launch the viewpoint planner service node.

Parameters (all optional, overriding the node defaults):
  radius_m, num_viewpoints, strategy, filter_ik_reachable, joint_states_topic
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    declare_radius = DeclareLaunchArgument(
        "radius_m", default_value="0.5", description="viewpoint sampling radius (m)"
    )
    declare_count = DeclareLaunchArgument(
        "num_viewpoints",
        default_value="32",
        description="number of viewpoints to sample",
    )
    declare_strategy = DeclareLaunchArgument(
        "strategy",
        default_value="fibonacci_sphere",
        description="sampling strategy name (see strategy registry)",
    )
    declare_filter = DeclareLaunchArgument(
        "filter_ik_reachable",
        default_value="true",
        description="keep only IK-reachable viewpoints",
    )
    declare_joint_topic = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/joint_states",
        description="joint state topic used as IK seed",
    )
    declare_publish_markers = DeclareLaunchArgument(
        "publish_markers",
        default_value="true",
        description="publish RViz MarkerArray for each plan",
    )
    declare_markers_topic = DeclareLaunchArgument(
        "markers_topic",
        default_value="/arachne/viewpoint/markers",
        description="MarkerArray topic for viewpoint visualization",
    )
    declare_markers_frame = DeclareLaunchArgument(
        "markers_frame",
        default_value="base_link",
        description="frame_id used for the viewpoint markers",
    )
    declare_label_enabled = DeclareLaunchArgument(
        "marker_label_enabled",
        default_value="true",
        description="show per-viewpoint index labels in RViz",
    )
    declare_label_offset = DeclareLaunchArgument(
        "marker_label_offset_z",
        default_value="0.08",
        description="index label height above the camera origin (m)",
    )
    declare_label_size = DeclareLaunchArgument(
        "marker_label_size",
        default_value="0.06",
        description="index label font height (m)",
    )
    declare_republish_period = DeclareLaunchArgument(
        "markers_republish_period_sec",
        default_value="1.0",
        description="periodic republish of the last marker array (0 = off)",
    )
    declare_ik_budget = DeclareLaunchArgument(
        "ik_time_budget_sec",
        default_value="12.0",
        description="max seconds for IK reachability filtering (hard timeout guard)",
    )

    node = Node(
        package="viewpoint_planner",
        executable="viewpoint_planner",
        name="viewpoint_planner",
        output="screen",
        parameters=[
            {
                "radius_m": LaunchConfiguration("radius_m"),
                "num_viewpoints": LaunchConfiguration("num_viewpoints"),
                "strategy": LaunchConfiguration("strategy"),
                "filter_ik_reachable": LaunchConfiguration("filter_ik_reachable"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "publish_markers": LaunchConfiguration("publish_markers"),
                "markers_topic": LaunchConfiguration("markers_topic"),
                "markers_frame": LaunchConfiguration("markers_frame"),
                "marker_label_enabled": LaunchConfiguration("marker_label_enabled"),
                "marker_label_offset_z": LaunchConfiguration("marker_label_offset_z"),
                "marker_label_size": LaunchConfiguration("marker_label_size"),
                "markers_republish_period_sec": LaunchConfiguration(
                    "markers_republish_period_sec"
                ),
                "ik_time_budget_sec": LaunchConfiguration("ik_time_budget_sec"),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_radius,
            declare_count,
            declare_strategy,
            declare_filter,
            declare_joint_topic,
            declare_publish_markers,
            declare_markers_topic,
            declare_markers_frame,
            declare_label_enabled,
            declare_label_offset,
            declare_label_size,
            declare_republish_period,
            declare_ik_budget,
            node,
        ]
    )
