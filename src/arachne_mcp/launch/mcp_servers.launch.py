"""Standalone launch for the Arachne MCP aggregate server (no teach panel GUI).

Brings up the execution-chain dependencies (real hardware bridge, MoveIt
planning, camera) plus ONE MCP endpoint surface on a single port with two
readable paths:

- /arachne/viewpoint/plan     -> read-only planning tools
- /arachne/viewpoint/acquire  -> motion-gated acquire-chain tools

The same capabilities are also exposed as plain-JSON HTTP endpoints under the
same per-capability bearer tokens (see arachne_mcp/json_api.py), e.g.:

    POST /arachne/json/viewpoint/plan/plan_viewpoints          (planner token)
    POST /arachne/json/viewpoint/acquire/acquire_chain         (execution token)
    GET  /arachne/json/viewpoint/acquire/status                (execution token)

Example:
    source scripts/env/arachne_env.sh && source install/setup.bash
    export ARACHNE_MCP_PLANNER_TOKEN=change-me
    export ARACHNE_MCP_EXECUTION_TOKEN=change-me
    ros2 launch arachne_mcp mcp_servers.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DEFAULT_ARM_STATE_JOINTS = (
    "aubo_shoulder_joint,aubo_upperArm_joint,aubo_foreArm_joint,"
    "aubo_wrist1_joint,aubo_wrist2_joint,aubo_wrist3_joint"
)
DEFAULT_ARM_COMMAND_JOINTS = (
    "shoulder_joint,upperArm_joint,foreArm_joint,wrist1_joint,wrist2_joint,wrist3_joint"
)


def _include_real_bringup():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arachne_hardware"),
                    "launch",
                    "real_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "aubo_robot_ip": LaunchConfiguration("aubo_sdk_ip"),
            "aubo_teach_command_topic": LaunchConfiguration("aubo_teach_command_topic"),
            "aubo_sdk_velocity_command_topic": LaunchConfiguration(
                "arm_velocity_command_topic"
            ),
            "aubo_rpc_port": LaunchConfiguration("aubo_sdk_rpc_port"),
            "aubo_rpc_timeout_sec": LaunchConfiguration("aubo_sdk_rpc_timeout_sec"),
            "aubo_teach_flag_path": LaunchConfiguration("aubo_sdk_teach_flag_path"),
            "aubo_control_owner_path": LaunchConfiguration(
                "aubo_sdk_control_owner_path"
            ),
            "aubo_teach_control_owner_name": LaunchConfiguration(
                "aubo_sdk_control_owner_name"
            ),
            "aubo_sdk_velocity_control_owner_name": LaunchConfiguration(
                "aubo_sdk_control_owner_name"
            ),
            "aubo_move_joint_action_name": LaunchConfiguration(
                "aubo_move_joint_action_name"
            ),
            "aubo_sdk_move_joint_speed_rad_sec": LaunchConfiguration(
                "aubo_sdk_move_speed_rad_sec"
            ),
            "aubo_sdk_move_joint_accel_rad_sec2": LaunchConfiguration(
                "aubo_sdk_move_accel_rad_sec2"
            ),
            "aubo_sdk_move_joint_goal_tolerance_rad": LaunchConfiguration(
                "aubo_sdk_goal_tolerance_rad"
            ),
            "aubo_sdk_move_joint_arrival_padding_sec": LaunchConfiguration(
                "aubo_sdk_arrival_timeout_padding_sec"
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_real_bringup")),
    )


def _include_moveit_planning():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("arachne_moveit_config"),
                    "launch",
                    "moveit_planning.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gripper_type": LaunchConfiguration("visualization_gripper_type"),
            "launch_rviz": "false",
            "with_robot_state_publisher": "false",
            "joint_states_topic": LaunchConfiguration("moveit_joint_states_topic"),
            "ee_support_xyz": LaunchConfiguration("visualization_ee_support_xyz"),
            "ee_support_rpy": LaunchConfiguration("visualization_ee_support_rpy"),
            "tool_adapter_xyz": LaunchConfiguration("visualization_tool_adapter_xyz"),
            "tool_adapter_rpy": LaunchConfiguration("visualization_tool_adapter_rpy"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_moveit_planning")),
    )


def _include_camera():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arachne_sensors"), "launch", "gemini335.launch.py"]
            )
        ),
        launch_arguments={
            "color_device": LaunchConfiguration("camera_color_device"),
            "color_capture_mode": LaunchConfiguration("camera_color_capture_mode"),
            "color_fourcc": LaunchConfiguration("camera_color_fourcc"),
            "depth_device": LaunchConfiguration("camera_depth_device"),
            "with_color_view": LaunchConfiguration("camera_with_color_view"),
            "with_depth_view": LaunchConfiguration("camera_with_depth_view"),
            "publish_depth": LaunchConfiguration("camera_publish_depth"),
            "publish_depth_color": LaunchConfiguration("camera_publish_depth_color"),
            "publish_pointcloud": LaunchConfiguration("camera_publish_pointcloud"),
            "camera_parent_frame": LaunchConfiguration("camera_parent_frame"),
            "projection_flip_x": LaunchConfiguration("camera_projection_flip_x"),
            "projection_flip_y": LaunchConfiguration("camera_projection_flip_y"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_camera")),
    )


def _mcp_servers_node():
    return Node(
        package="arachne_mcp",
        executable="mcp_servers",
        name="arachne_execution_mcp",
        output="screen",
        parameters=[
            {
                "arm_state_joint_names": LaunchConfiguration("arm_state_joint_names"),
                "arm_command_joint_names": LaunchConfiguration(
                    "arm_command_joint_names"
                ),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "moveit_plan_service": LaunchConfiguration("moveit_plan_service"),
                "moveit_joint_states_topic": LaunchConfiguration(
                    "moveit_joint_states_topic"
                ),
                "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                "aubo_teach_command_topic": LaunchConfiguration(
                    "aubo_teach_command_topic"
                ),
                "aubo_sdk_ip": LaunchConfiguration("aubo_sdk_ip"),
                "aubo_sdk_rpc_port": ParameterValue(
                    LaunchConfiguration("aubo_sdk_rpc_port"), value_type=int
                ),
                "aubo_sdk_rpc_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_rpc_timeout_sec"), value_type=float
                ),
                "aubo_sdk_move_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_move_speed_rad_sec"), value_type=float
                ),
                "aubo_sdk_move_accel_rad_sec2": ParameterValue(
                    LaunchConfiguration("aubo_sdk_move_accel_rad_sec2"),
                    value_type=float,
                ),
                "aubo_sdk_goal_tolerance_rad": ParameterValue(
                    LaunchConfiguration("aubo_sdk_goal_tolerance_rad"), value_type=float
                ),
                "aubo_sdk_arrival_timeout_padding_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_arrival_timeout_padding_sec"),
                    value_type=float,
                ),
                "aubo_sdk_teach_flag_path": LaunchConfiguration(
                    "aubo_sdk_teach_flag_path"
                ),
                "aubo_sdk_control_owner_path": LaunchConfiguration(
                    "aubo_sdk_control_owner_path"
                ),
                "aubo_sdk_control_owner_name": LaunchConfiguration(
                    "aubo_sdk_control_owner_name"
                ),
                "arm_velocity_command_topic": LaunchConfiguration(
                    "arm_velocity_command_topic"
                ),
                "arm_home_joints_deg": LaunchConfiguration("arm_home_joints_deg"),
                "arm_install_joints_deg": LaunchConfiguration("arm_install_joints_deg"),
                "arm_keepout_enabled": ParameterValue(
                    LaunchConfiguration("arm_keepout_enabled"), value_type=bool
                ),
                "arm_base_xyz": LaunchConfiguration("arm_base_xyz"),
                "arm_base_rpy": LaunchConfiguration("arm_base_rpy"),
                "rear_rack_keepout_min_xyz": LaunchConfiguration(
                    "rear_rack_keepout_min_xyz"
                ),
                "rear_rack_keepout_max_xyz": LaunchConfiguration(
                    "rear_rack_keepout_max_xyz"
                ),
                "front_basket_keepout_min_xyz": LaunchConfiguration(
                    "front_basket_keepout_min_xyz"
                ),
                "front_basket_keepout_max_xyz": LaunchConfiguration(
                    "front_basket_keepout_max_xyz"
                ),
                "moveit_plan_enabled": ParameterValue(
                    LaunchConfiguration("moveit_plan_enabled"), value_type=bool
                ),
                "moveit_refuse_on_plan_failure": ParameterValue(
                    LaunchConfiguration("moveit_refuse_on_plan_failure"),
                    value_type=bool,
                ),
                "moveit_planning_time": ParameterValue(
                    LaunchConfiguration("moveit_planning_time"), value_type=float
                ),
                "moveit_planning_attempts": ParameterValue(
                    LaunchConfiguration("moveit_planning_attempts"), value_type=int
                ),
                "moveit_velocity_scale": ParameterValue(
                    LaunchConfiguration("moveit_velocity_scale"), value_type=float
                ),
                "moveit_accel_scale": ParameterValue(
                    LaunchConfiguration("moveit_accel_scale"), value_type=float
                ),
                "camera_image_topic": LaunchConfiguration("camera_image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "points_topic": LaunchConfiguration("points_topic"),
                "color_camera_info_topic": LaunchConfiguration(
                    "color_camera_info_topic"
                ),
                "depth_camera_info_topic": LaunchConfiguration(
                    "depth_camera_info_topic"
                ),
                "viewpoint_acquisition_dir_name": LaunchConfiguration(
                    "viewpoint_acquisition_dir_name"
                ),
                "pointcloud_decimation": ParameterValue(
                    LaunchConfiguration("pointcloud_decimation"), value_type=int
                ),
                "depth_scale": ParameterValue(
                    LaunchConfiguration("depth_scale"), value_type=float
                ),
                # Headless-constraint params (plan §2.4): the execution MCP
                # instance must not pull managed services or GUI teach config.
                "teach_config_autoload": "false",
                "autostart_managed_processes": "",
                "runtime_log_root": LaunchConfiguration(
                    "mcp_execution_runtime_log_root"
                ),
                "workspace_root": LaunchConfiguration("workspace_root"),
                # MCP layer (aggregate listener on one port; per-path tokens)
                "mcp_bind_host": LaunchConfiguration("mcp_bind_host"),
                "mcp_port": ParameterValue(
                    LaunchConfiguration("mcp_port"), value_type=int
                ),
                "mcp_planner_token": LaunchConfiguration("mcp_planner_token"),
                "mcp_execution_token": LaunchConfiguration(
                    "mcp_execution_token"
                ),
                # Optional MCP Host-header allowlist (empty = LAN open)
                "mcp_allowed_hosts": LaunchConfiguration("mcp_allowed_hosts"),
                "motion_enabled": ParameterValue(
                    LaunchConfiguration("mcp_motion_enabled"), value_type=bool
                ),
                "confirm_agent_motion": ParameterValue(
                    LaunchConfiguration("mcp_confirm_agent_motion"), value_type=bool
                ),
                "viewpoint_plan_share_dir": LaunchConfiguration(
                    "viewpoint_plan_share_dir"
                ),
                "viewpoint_plan_share_keep": ParameterValue(
                    LaunchConfiguration("viewpoint_plan_share_keep"), value_type=int
                ),
                "acquire_chain_max_points": ParameterValue(
                    LaunchConfiguration("acquire_chain_max_points"), value_type=int
                ),
                # Planner capability (hosted): its params are declared on the
                # same node; share dir/keep above are already shared.
                "viewpoint_plan_service": LaunchConfiguration(
                    "viewpoint_plan_service"
                ),
                "viewpoint_plan_service_timeout_sec": ParameterValue(
                    LaunchConfiguration("viewpoint_plan_service_timeout_sec"),
                    value_type=float,
                ),
            }
        ],
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument("with_real_bringup", default_value="true"),
        DeclareLaunchArgument("with_moveit_planning", default_value="true"),
        DeclareLaunchArgument("with_camera", default_value="true"),
        # Shared teach/arm params (same defaults as teach_panel.launch.py)
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "moveit_joint_states_topic", default_value="/arachne/moveit/joint_states"
        ),
        DeclareLaunchArgument(
            "moveit_plan_service", default_value="/plan_kinematic_path"
        ),
        DeclareLaunchArgument(
            "arm_state_joint_names", default_value=DEFAULT_ARM_STATE_JOINTS
        ),
        DeclareLaunchArgument(
            "arm_command_joint_names", default_value=DEFAULT_ARM_COMMAND_JOINTS
        ),
        DeclareLaunchArgument(
            "gripper_command_topic", default_value="/arachne/gripper/command"
        ),
        DeclareLaunchArgument(
            "aubo_teach_command_topic", default_value="/arachne/aubo/teach_command"
        ),
        DeclareLaunchArgument(
            "arm_velocity_command_topic",
            default_value="/arachne/aubo/joint_velocity_command",
        ),
        DeclareLaunchArgument(
            "aubo_sdk_ip",
            default_value=EnvironmentVariable(
                "AUBO_ROBOT_IP", default_value="192.168.127.128"
            ),
        ),
        DeclareLaunchArgument("aubo_sdk_rpc_port", default_value="30004"),
        DeclareLaunchArgument("aubo_sdk_rpc_timeout_sec", default_value="3.0"),
        DeclareLaunchArgument("aubo_sdk_move_speed_rad_sec", default_value="1.00"),
        DeclareLaunchArgument("aubo_sdk_move_accel_rad_sec2", default_value="1.8"),
        DeclareLaunchArgument("aubo_sdk_goal_tolerance_rad", default_value="0.04"),
        DeclareLaunchArgument(
            "aubo_sdk_arrival_timeout_padding_sec", default_value="3.0"
        ),
        DeclareLaunchArgument(
            "aubo_sdk_teach_flag_path", default_value="/tmp/arachne_aubo_teach_mode"
        ),
        DeclareLaunchArgument(
            "aubo_sdk_control_owner_path",
            default_value="/tmp/arachne_aubo_control_owner",
        ),
        DeclareLaunchArgument(
            "aubo_sdk_control_owner_name", default_value="teach_panel"
        ),
        DeclareLaunchArgument(
            "aubo_move_joint_action_name", default_value="/arachne/aubo/move_joint"
        ),
        DeclareLaunchArgument(
            "arm_home_joints_deg", default_value="-90.00,11.55,95.09,27.80,96.07,43.79"
        ),
        DeclareLaunchArgument(
            "arm_install_joints_deg",
            default_value="-90.00,11.55,95.09,27.80,96.07,43.79",
        ),
        DeclareLaunchArgument("arm_keepout_enabled", default_value="true"),
        DeclareLaunchArgument("arm_base_xyz", default_value="0.22,0.0,0.105"),
        DeclareLaunchArgument("arm_base_rpy", default_value="0.0,0.0,1.57079632679"),
        DeclareLaunchArgument(
            "rear_rack_keepout_min_xyz", default_value="-0.41,-0.22,0.04"
        ),
        DeclareLaunchArgument(
            "rear_rack_keepout_max_xyz", default_value="0.09,0.22,0.82"
        ),
        DeclareLaunchArgument(
            "front_basket_keepout_min_xyz", default_value="0.4215,-0.11,-0.1235"
        ),
        DeclareLaunchArgument(
            "front_basket_keepout_max_xyz", default_value="0.6655,0.11,0.0635"
        ),
        DeclareLaunchArgument("moveit_plan_enabled", default_value="true"),
        DeclareLaunchArgument("moveit_refuse_on_plan_failure", default_value="true"),
        DeclareLaunchArgument("moveit_planning_time", default_value="2.0"),
        DeclareLaunchArgument("moveit_planning_attempts", default_value="3"),
        DeclareLaunchArgument("moveit_velocity_scale", default_value="0.5"),
        DeclareLaunchArgument("moveit_accel_scale", default_value="0.5"),
        DeclareLaunchArgument(
            "camera_image_topic", default_value="/camera/color/image_raw"
        ),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("points_topic", default_value="/camera/points"),
        DeclareLaunchArgument(
            "color_camera_info_topic", default_value="/camera/color/camera_info"
        ),
        DeclareLaunchArgument(
            "depth_camera_info_topic", default_value="/camera/depth/camera_info"
        ),
        DeclareLaunchArgument(
            "viewpoint_acquisition_dir_name", default_value="viewpoint_acquisition"
        ),
        DeclareLaunchArgument("pointcloud_decimation", default_value="6"),
        DeclareLaunchArgument("depth_scale", default_value="0.001"),
        DeclareLaunchArgument(
            "viewpoint_plan_service", default_value="/arachne/viewpoint/plan"
        ),
        DeclareLaunchArgument(
            "viewpoint_plan_service_timeout_sec", default_value="60.0"
        ),
        DeclareLaunchArgument(
            "viewpoint_plan_share_dir", default_value="log/viewpoint_plans"
        ),
        DeclareLaunchArgument("viewpoint_plan_share_keep", default_value="20"),
        DeclareLaunchArgument("acquire_chain_max_points", default_value="32"),
        DeclareLaunchArgument("workspace_root", default_value=""),
        DeclareLaunchArgument(
            "mcp_execution_runtime_log_root", default_value="log/arachne_execution_mcp"
        ),
        # Camera passthrough args (defaults mirror teach_panel.launch.py)
        DeclareLaunchArgument("camera_color_device", default_value="/dev/video6"),
        DeclareLaunchArgument("camera_color_capture_mode", default_value="v4l2-ctl"),
        DeclareLaunchArgument("camera_color_fourcc", default_value="YUYV"),
        DeclareLaunchArgument("camera_depth_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_with_color_view", default_value="false"),
        DeclareLaunchArgument("camera_with_depth_view", default_value="false"),
        DeclareLaunchArgument("camera_publish_depth", default_value="false"),
        DeclareLaunchArgument("camera_publish_depth_color", default_value="false"),
        DeclareLaunchArgument("camera_publish_pointcloud", default_value="false"),
        DeclareLaunchArgument("camera_parent_frame", default_value="ee_camera_link"),
        DeclareLaunchArgument("camera_projection_flip_x", default_value="true"),
        DeclareLaunchArgument("camera_projection_flip_y", default_value="true"),
        DeclareLaunchArgument("visualization_gripper_type", default_value="ms42dc"),
        DeclareLaunchArgument(
            "visualization_ee_support_xyz", default_value="0.0 0.0 0.0"
        ),
        DeclareLaunchArgument(
            "visualization_ee_support_rpy", default_value="0.0 0.0 2.35619449019"
        ),
        DeclareLaunchArgument(
            "visualization_tool_adapter_xyz",
            default_value="-0.049334103 0.049874070 0.021816675",
        ),
        DeclareLaunchArgument(
            "visualization_tool_adapter_rpy", default_value="1.570796327 0.0 0.0"
        ),
        # MCP endpoints (tokens come from the environment; absent token = path refused)
        DeclareLaunchArgument("mcp_bind_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("mcp_port", default_value="8123"),
        DeclareLaunchArgument(
            "mcp_planner_token",
            default_value=EnvironmentVariable(
                "ARACHNE_MCP_PLANNER_TOKEN", default_value=""
            ),
        ),
        DeclareLaunchArgument(
            "mcp_execution_token",
            default_value=EnvironmentVariable(
                "ARACHNE_MCP_EXECUTION_TOKEN", default_value=""
            ),
        ),
        # Optional MCP Host-header allowlist: comma-separated ``host`` or
        # ``host:*`` patterns (e.g. "192.168.1.50:*"); the wildcard covers
        # the port only. Empty disables the SDK's Host check (LAN serving
        # by design, bearer tokens still apply).
        DeclareLaunchArgument(
            "mcp_allowed_hosts",
            default_value=EnvironmentVariable(
                "ARACHNE_MCP_ALLOWED_HOSTS", default_value=""
            ),
        ),
        # Motion gate (both must be true before acquire_chain may move the arm)
        DeclareLaunchArgument("mcp_motion_enabled", default_value="false"),
        DeclareLaunchArgument("mcp_confirm_agent_motion", default_value="false"),
    ]

    return LaunchDescription(
        [
            *args,
            _include_real_bringup(),
            _include_moveit_planning(),
            _include_camera(),
            _mcp_servers_node(),
        ]
    )
