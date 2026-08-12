"""Launch the viewpoint planner service node.

Parameters (all optional, overriding the node defaults):
  radius_m, num_viewpoints, strategy, filter_ik_reachable, joint_states_topic
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            declare_ik_budget,
            node,
        ]
    )
