from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arachne_operator"), "launch", "teach_visualization.launch.py"]
            )
        ),
        launch_arguments={
            "input_joint_states_topic": LaunchConfiguration(
                "visualization_input_joint_states_topic"
            ),
            "with_rviz": LaunchConfiguration("visualization_with_rviz"),
            "gripper_type": LaunchConfiguration("visualization_gripper_type"),
            "with_lidar": LaunchConfiguration("visualization_with_lidar"),
            "with_lslidar_driver": LaunchConfiguration("visualization_with_lslidar_driver"),
            "with_ee_camera": LaunchConfiguration("visualization_with_ee_camera"),
            "ee_support_xyz": LaunchConfiguration("visualization_ee_support_xyz"),
            "ee_support_rpy": LaunchConfiguration("visualization_ee_support_rpy"),
            "tool_adapter_xyz": LaunchConfiguration("visualization_tool_adapter_xyz"),
            "tool_adapter_rpy": LaunchConfiguration("visualization_tool_adapter_rpy"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_visualization")),
    )

    camera = IncludeLaunchDescription(
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

    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arachne_hardware"), "launch", "real_bringup.launch.py"]
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
            "aubo_control_owner_path": LaunchConfiguration("aubo_sdk_control_owner_path"),
            "aubo_teach_control_owner_name": LaunchConfiguration(
                "aubo_sdk_control_owner_name"
            ),
            "aubo_sdk_velocity_control_owner_name": LaunchConfiguration(
                "aubo_sdk_control_owner_name"
            ),
            "aubo_move_joint_action_name": LaunchConfiguration("aubo_move_joint_action_name"),
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

    demo_orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arachne_operator"), "launch", "demo_orchestrator.launch.py"]
            )
        ),
        launch_arguments={
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
            "aubo_teach_command_topic": LaunchConfiguration("aubo_teach_command_topic"),
            "aubo_move_joint_action_name": LaunchConfiguration("aubo_move_joint_action_name"),
            "grasp_task_start_service": LaunchConfiguration("grasp_task_start_service"),
            "grasp_task_stop_service": LaunchConfiguration("grasp_task_stop_service"),
            "grasp_task_preflight_service": LaunchConfiguration("grasp_task_preflight_service"),
            "cleanup_task_start_service": LaunchConfiguration("cleanup_task_start_service"),
            "cleanup_task_stop_service": LaunchConfiguration("cleanup_task_stop_service"),
            "cleanup_task_preflight_service": LaunchConfiguration(
                "cleanup_task_preflight_service"
            ),
            "camera_command": LaunchConfiguration("camera_command"),
            "depth_pointcloud_command": LaunchConfiguration("depth_pointcloud_command"),
            "camera_view_command": LaunchConfiguration("camera_view_command"),
            "grasp_server_command": LaunchConfiguration("grasp_server_command"),
            "cleanup_server_command": LaunchConfiguration("cleanup_server_command"),
            "service_stop_timeout_sec": LaunchConfiguration("service_stop_timeout_sec"),
            "runtime_log_root": LaunchConfiguration("runtime_log_root"),
            "workspace_root": LaunchConfiguration("workspace_root"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_demo_orchestrator")),
    )

    teach_panel_node = Node(
        package="arachne_operator",
        executable="teach_panel",
        name="arachne_teach_panel",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "arm_follow_joint_trajectory_action": LaunchConfiguration(
                    "arm_follow_joint_trajectory_action"
                ),
                "arm_trajectory_topic": LaunchConfiguration("arm_trajectory_topic"),
                "legacy_arm_trajectory_topic": LaunchConfiguration(
                    "legacy_arm_trajectory_topic"
                ),
                "arm_state_joint_names": LaunchConfiguration("arm_state_joint_names"),
                "arm_command_joint_names": LaunchConfiguration("arm_command_joint_names"),
                "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                "aubo_teach_command_topic": LaunchConfiguration(
                    "aubo_teach_command_topic"
                ),
                "aubo_teach_exit_wait_sec": ParameterValue(
                    LaunchConfiguration("aubo_teach_exit_wait_sec"), value_type=float
                ),
                "base_linear_speed": ParameterValue(
                    LaunchConfiguration("base_linear_speed"), value_type=float
                ),
                "base_angular_speed": ParameterValue(
                    LaunchConfiguration("base_angular_speed"), value_type=float
                ),
                "base_replay_linear_speed": ParameterValue(
                    LaunchConfiguration("base_replay_linear_speed"), value_type=float
                ),
                "base_replay_angular_speed": ParameterValue(
                    LaunchConfiguration("base_replay_angular_speed"), value_type=float
                ),
                "base_manual_publish_rate": ParameterValue(
                    LaunchConfiguration("base_manual_publish_rate"), value_type=float
                ),
                "base_motion_max_segment_sec": ParameterValue(
                    LaunchConfiguration("base_motion_max_segment_sec"), value_type=float
                ),
                "base_ignore_spurious_zero_odom": ParameterValue(
                    LaunchConfiguration("base_ignore_spurious_zero_odom"), value_type=bool
                ),
                "arm_jog_step_m": ParameterValue(
                    LaunchConfiguration("arm_jog_step_m"), value_type=float
                ),
                "arm_jog_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_jog_duration_sec"), value_type=float
                ),
                "arm_rotate_step_rad": ParameterValue(
                    LaunchConfiguration("arm_rotate_step_rad"), value_type=float
                ),
                "arm_rotate_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_rotate_duration_sec"), value_type=float
                ),
                "arm_joint_step_rad": ParameterValue(
                    LaunchConfiguration("arm_joint_step_rad"), value_type=float
                ),
                "arm_hold_period_sec": ParameterValue(
                    LaunchConfiguration("arm_hold_period_sec"), value_type=float
                ),
                "arm_manual_prefer_topic": ParameterValue(
                    LaunchConfiguration("arm_manual_prefer_topic"), value_type=bool
                ),
                "arm_velocity_command_topic": LaunchConfiguration(
                    "arm_velocity_command_topic"
                ),
                "arm_velocity_publish_rate": ParameterValue(
                    LaunchConfiguration("arm_velocity_publish_rate"), value_type=float
                ),
                "arm_velocity_watchdog_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_watchdog_sec"), value_type=float
                ),
                "arm_joint_jog_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("arm_joint_jog_speed_rad_sec"), value_type=float
                ),
                "arm_cartesian_jog_speed_m_sec": ParameterValue(
                    LaunchConfiguration("arm_cartesian_jog_speed_m_sec"), value_type=float
                ),
                "arm_cartesian_rotate_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("arm_cartesian_rotate_speed_rad_sec"), value_type=float
                ),
                "arm_velocity_damping": ParameterValue(
                    LaunchConfiguration("arm_velocity_damping"), value_type=float
                ),
                "arm_velocity_max_joint_speed_rad_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_max_joint_speed_rad_sec"), value_type=float
                ),
                "arm_velocity_max_joint_accel_rad_sec2": ParameterValue(
                    LaunchConfiguration("arm_velocity_max_joint_accel_rad_sec2"), value_type=float
                ),
                "arm_velocity_max_joint_jerk_rad_sec3": ParameterValue(
                    LaunchConfiguration("arm_velocity_max_joint_jerk_rad_sec3"), value_type=float
                ),
                "arm_velocity_smoothing_tau_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_smoothing_tau_sec"), value_type=float
                ),
                "arm_velocity_keepout_predict_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_keepout_predict_sec"), value_type=float
                ),
                "arm_velocity_keepout_check_interval_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_keepout_check_interval_sec"), value_type=float
                ),
                "arm_velocity_stream_deadman_sec": ParameterValue(
                    LaunchConfiguration("arm_velocity_stream_deadman_sec"), value_type=float
                ),
                "arm_waypoint_duration_sec": ParameterValue(
                    LaunchConfiguration("arm_waypoint_duration_sec"), value_type=float
                ),
                "arm_replay_backend": LaunchConfiguration("arm_replay_backend"),
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
                    LaunchConfiguration("aubo_sdk_move_accel_rad_sec2"), value_type=float
                ),
                "aubo_sdk_blend_radius": ParameterValue(
                    LaunchConfiguration("aubo_sdk_blend_radius"), value_type=float
                ),
                "aubo_sdk_move_duration_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_move_duration_sec"), value_type=float
                ),
                "aubo_sdk_goal_tolerance_rad": ParameterValue(
                    LaunchConfiguration("aubo_sdk_goal_tolerance_rad"), value_type=float
                ),
                "aubo_sdk_arrival_timeout_padding_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_arrival_timeout_padding_sec"), value_type=float
                ),
                "aubo_sdk_lifecycle_power_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_lifecycle_power_timeout_sec"), value_type=float
                ),
                "aubo_sdk_lifecycle_startup_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_lifecycle_startup_timeout_sec"), value_type=float
                ),
                "aubo_sdk_lifecycle_poll_sec": ParameterValue(
                    LaunchConfiguration("aubo_sdk_lifecycle_poll_sec"), value_type=float
                ),
                "aubo_sdk_teach_flag_path": LaunchConfiguration("aubo_sdk_teach_flag_path"),
                "aubo_sdk_control_owner_path": LaunchConfiguration("aubo_sdk_control_owner_path"),
                "aubo_sdk_control_owner_name": LaunchConfiguration("aubo_sdk_control_owner_name"),
                "aubo_move_joint_action_name": LaunchConfiguration("aubo_move_joint_action_name"),
                "aubo_move_joint_action_timeout_sec": ParameterValue(
                    LaunchConfiguration("aubo_move_joint_action_timeout_sec"),
                    value_type=float,
                ),
                "aubo_move_joint_fallback_internal": ParameterValue(
                    LaunchConfiguration("aubo_move_joint_fallback_internal"),
                    value_type=bool,
                ),
                "demo_orchestrator_enabled": ParameterValue(
                    LaunchConfiguration("demo_orchestrator_enabled"),
                    value_type=bool,
                ),
                "demo_orchestrator_fallback_internal": ParameterValue(
                    LaunchConfiguration("demo_orchestrator_fallback_internal"),
                    value_type=bool,
                ),
                "demo_start_visual_grasp_service": LaunchConfiguration(
                    "demo_start_visual_grasp_service"
                ),
                "demo_start_road_cleanup_service": LaunchConfiguration(
                    "demo_start_road_cleanup_service"
                ),
                "demo_pause_road_cleanup_service": LaunchConfiguration(
                    "demo_pause_road_cleanup_service"
                ),
                "demo_return_home_service": LaunchConfiguration("demo_return_home_service"),
                "demo_stop_service": LaunchConfiguration("demo_stop_service"),
                "demo_preflight_service": LaunchConfiguration("demo_preflight_service"),
                "demo_status_service": LaunchConfiguration("demo_status_service"),
                "demo_state_topic": LaunchConfiguration("demo_state_topic"),
                "gripper_settle_sec": ParameterValue(
                    LaunchConfiguration("gripper_settle_sec"), value_type=float
                ),
                "arm_jog_position_tolerance": ParameterValue(
                    LaunchConfiguration("arm_jog_position_tolerance"), value_type=float
                ),
                "arm_orientation_tolerance": ParameterValue(
                    LaunchConfiguration("arm_orientation_tolerance"), value_type=float
                ),
                "arm_jog_orientation_tolerance": ParameterValue(
                    LaunchConfiguration("arm_jog_orientation_tolerance"), value_type=float
                ),
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
                "arm_keepout_sample_step_m": ParameterValue(
                    LaunchConfiguration("arm_keepout_sample_step_m"), value_type=float
                ),
                "arm_keepout_joint_step_rad": ParameterValue(
                    LaunchConfiguration("arm_keepout_joint_step_rad"), value_type=float
                ),
                "arm_home_joints_deg": LaunchConfiguration("arm_home_joints_deg"),
                "arm_install_joints_deg": LaunchConfiguration("arm_install_joints_deg"),
                "aubo_payload_mass_kg": ParameterValue(
                    LaunchConfiguration("aubo_payload_mass_kg"), value_type=float
                ),
                "aubo_payload_cog": LaunchConfiguration("aubo_payload_cog"),
                "aubo_payload_aom": LaunchConfiguration("aubo_payload_aom"),
                "aubo_payload_inertia": LaunchConfiguration("aubo_payload_inertia"),
                "arm_goal_tolerance": ParameterValue(
                    LaunchConfiguration("arm_goal_tolerance"), value_type=float
                ),
                "recording_dir": LaunchConfiguration("recording_dir"),
                "teach_config_path": LaunchConfiguration("teach_config_path"),
                "teach_config_autoload": ParameterValue(
                    LaunchConfiguration("teach_config_autoload"), value_type=bool
                ),
                "workspace_root": LaunchConfiguration("workspace_root"),
                "runtime_log_root": LaunchConfiguration("runtime_log_root"),
                "autostart_managed_processes": LaunchConfiguration("autostart_managed_processes"),
                "service_stop_timeout_sec": ParameterValue(
                    LaunchConfiguration("service_stop_timeout_sec"), value_type=float
                ),
                "camera_command": LaunchConfiguration("camera_command"),
                "depth_pointcloud_command": LaunchConfiguration("depth_pointcloud_command"),
                "camera_view_command": LaunchConfiguration("camera_view_command"),
                "yolo_view_command": LaunchConfiguration("yolo_view_command"),
                "slam_command": LaunchConfiguration("slam_command"),
                "grasp_server_command": LaunchConfiguration("grasp_server_command"),
                "cleanup_server_command": LaunchConfiguration("cleanup_server_command"),
                "step_cleanup_server_command": LaunchConfiguration("step_cleanup_server_command"),
                "hand_eye_calibrator_command": LaunchConfiguration("hand_eye_calibrator_command"),
                "grasp_task_state_topic": LaunchConfiguration("grasp_task_state_topic"),
                "grasp_task_start_service": LaunchConfiguration("grasp_task_start_service"),
                "grasp_task_stop_service": LaunchConfiguration("grasp_task_stop_service"),
                "grasp_task_restore_service": LaunchConfiguration("grasp_task_restore_service"),
                "grasp_task_status_service": LaunchConfiguration("grasp_task_status_service"),
                "cleanup_task_state_topic": LaunchConfiguration("cleanup_task_state_topic"),
                "cleanup_task_start_service": LaunchConfiguration("cleanup_task_start_service"),
                "cleanup_task_pause_service": LaunchConfiguration("cleanup_task_pause_service"),
                "cleanup_task_return_home_service": LaunchConfiguration(
                    "cleanup_task_return_home_service"
                ),
                "cleanup_task_stop_service": LaunchConfiguration("cleanup_task_stop_service"),
            "cleanup_task_status_service": LaunchConfiguration("cleanup_task_status_service"),
            "cleanup_task_preflight_service": LaunchConfiguration("cleanup_task_preflight_service"),
            "step_cleanup_state_topic": LaunchConfiguration("step_cleanup_state_topic"),
            "step_cleanup_start_service": LaunchConfiguration("step_cleanup_start_service"),
            "step_cleanup_stop_service": LaunchConfiguration("step_cleanup_stop_service"),
            "step_cleanup_status_service": LaunchConfiguration("step_cleanup_status_service"),
            "step_cleanup_preflight_service": LaunchConfiguration("step_cleanup_preflight_service"),
        }
    ],
)

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/arachne/teach_visualization/joint_states",
            ),
            DeclareLaunchArgument(
                "visualization_input_joint_states_topic", default_value="/joint_states"
            ),
            DeclareLaunchArgument("with_visualization", default_value="true"),
            DeclareLaunchArgument("visualization_with_rviz", default_value="true"),
            DeclareLaunchArgument("visualization_gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("visualization_with_lidar", default_value="true"),
            DeclareLaunchArgument("visualization_with_lslidar_driver", default_value="false"),
            DeclareLaunchArgument("visualization_with_ee_camera", default_value="true"),
            DeclareLaunchArgument("visualization_ee_support_xyz", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("visualization_ee_support_rpy", default_value="0.0 0.0 2.35619449019"),
            DeclareLaunchArgument("visualization_tool_adapter_xyz", default_value="-0.049334103 0.049874070 0.021816675"),
            DeclareLaunchArgument(
                "visualization_tool_adapter_rpy",
                default_value="1.570796327 0.0 0.0",
            ),
            DeclareLaunchArgument("with_camera", default_value="false"),
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
            DeclareLaunchArgument("with_real_bringup", default_value="true"),
            DeclareLaunchArgument(
                "arm_follow_joint_trajectory_action",
                default_value="/joint_trajectory_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_trajectory_topic",
                default_value="/joint_trajectory_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "legacy_arm_trajectory_topic",
                default_value="/aubo_arm_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "arm_state_joint_names",
                default_value=(
                    "aubo_shoulder_joint,aubo_upperArm_joint,aubo_foreArm_joint,"
                    "aubo_wrist1_joint,aubo_wrist2_joint,aubo_wrist3_joint"
                ),
            ),
            DeclareLaunchArgument(
                "arm_command_joint_names",
                default_value=(
                    "shoulder_joint,upperArm_joint,foreArm_joint,"
                    "wrist1_joint,wrist2_joint,wrist3_joint"
                ),
            ),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument(
                "aubo_teach_command_topic",
                default_value="/arachne/aubo/teach_command",
            ),
            DeclareLaunchArgument("base_linear_speed", default_value="0.16"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.60"),
            DeclareLaunchArgument("base_replay_linear_speed", default_value="0.40"),
            DeclareLaunchArgument("base_replay_angular_speed", default_value="0.48"),
            DeclareLaunchArgument("base_manual_publish_rate", default_value="12.0"),
            DeclareLaunchArgument("base_motion_max_segment_sec", default_value="20.0"),
            DeclareLaunchArgument("base_ignore_spurious_zero_odom", default_value="true"),
            DeclareLaunchArgument("arm_jog_step_m", default_value="0.008"),
            DeclareLaunchArgument("arm_jog_duration_sec", default_value="0.06"),
            DeclareLaunchArgument("arm_rotate_step_rad", default_value="0.0122173"),
            DeclareLaunchArgument("arm_rotate_duration_sec", default_value="0.06"),
            DeclareLaunchArgument("arm_joint_step_rad", default_value="0.00698132"),
            DeclareLaunchArgument("arm_hold_period_sec", default_value="0.10"),
            DeclareLaunchArgument("arm_manual_prefer_topic", default_value="true"),
            DeclareLaunchArgument(
                "arm_velocity_command_topic",
                default_value="/arachne/aubo/joint_velocity_command",
            ),
            DeclareLaunchArgument("arm_velocity_publish_rate", default_value="80.0"),
            DeclareLaunchArgument("arm_velocity_watchdog_sec", default_value="0.20"),
            DeclareLaunchArgument("arm_joint_jog_speed_rad_sec", default_value="0.32"),
            DeclareLaunchArgument("arm_cartesian_jog_speed_m_sec", default_value="0.10"),
            DeclareLaunchArgument("arm_cartesian_rotate_speed_rad_sec", default_value="0.32"),
            DeclareLaunchArgument("arm_velocity_damping", default_value="0.08"),
            DeclareLaunchArgument("arm_velocity_max_joint_speed_rad_sec", default_value="1.00"),
            DeclareLaunchArgument("arm_velocity_max_joint_accel_rad_sec2", default_value="1.60"),
            DeclareLaunchArgument("arm_velocity_max_joint_jerk_rad_sec3", default_value="24.0"),
            DeclareLaunchArgument("arm_velocity_smoothing_tau_sec", default_value="0.08"),
            DeclareLaunchArgument("arm_velocity_keepout_predict_sec", default_value="0.35"),
            DeclareLaunchArgument("arm_velocity_keepout_check_interval_sec", default_value="0.05"),
            DeclareLaunchArgument("arm_velocity_stream_deadman_sec", default_value="0.75"),
            DeclareLaunchArgument("arm_waypoint_duration_sec", default_value="0.9375"),
            DeclareLaunchArgument("arm_replay_backend", default_value="sdk_move_joint"),
            DeclareLaunchArgument(
                "aubo_sdk_ip",
                default_value=EnvironmentVariable("AUBO_ROBOT_IP", default_value="192.168.127.128"),
            ),
            DeclareLaunchArgument("aubo_sdk_rpc_port", default_value="30004"),
            DeclareLaunchArgument("aubo_sdk_rpc_timeout_sec", default_value="3.0"),
            DeclareLaunchArgument("aubo_sdk_move_speed_rad_sec", default_value="1.00"),
            DeclareLaunchArgument("aubo_sdk_move_accel_rad_sec2", default_value="1.8"),
            DeclareLaunchArgument("aubo_sdk_blend_radius", default_value="0.0"),
            DeclareLaunchArgument("aubo_sdk_move_duration_sec", default_value="0.0"),
            DeclareLaunchArgument("aubo_sdk_goal_tolerance_rad", default_value="0.04"),
            DeclareLaunchArgument("aubo_sdk_arrival_timeout_padding_sec", default_value="3.0"),
            DeclareLaunchArgument("aubo_sdk_lifecycle_power_timeout_sec", default_value="45.0"),
            DeclareLaunchArgument("aubo_sdk_lifecycle_startup_timeout_sec", default_value="45.0"),
            DeclareLaunchArgument("aubo_sdk_lifecycle_poll_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "aubo_sdk_teach_flag_path",
                default_value="/tmp/arachne_aubo_teach_mode",
            ),
            DeclareLaunchArgument(
                "aubo_sdk_control_owner_path",
                default_value="/tmp/arachne_aubo_control_owner",
            ),
            DeclareLaunchArgument("aubo_sdk_control_owner_name", default_value="teach_panel"),
            DeclareLaunchArgument(
                "aubo_move_joint_action_name",
                default_value="/arachne/aubo/move_joint",
            ),
            DeclareLaunchArgument("aubo_move_joint_action_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("aubo_move_joint_fallback_internal", default_value="false"),
            DeclareLaunchArgument("with_demo_orchestrator", default_value="true"),
            DeclareLaunchArgument("demo_orchestrator_enabled", default_value="true"),
            DeclareLaunchArgument("demo_orchestrator_fallback_internal", default_value="true"),
            DeclareLaunchArgument(
                "demo_start_visual_grasp_service",
                default_value="/arachne/demo/start_visual_grasp",
            ),
            DeclareLaunchArgument(
                "demo_start_road_cleanup_service",
                default_value="/arachne/demo/start_road_cleanup",
            ),
            DeclareLaunchArgument(
                "demo_pause_road_cleanup_service",
                default_value="/arachne/demo/pause_road_cleanup",
            ),
            DeclareLaunchArgument(
                "demo_return_home_service",
                default_value="/arachne/demo/return_home",
            ),
            DeclareLaunchArgument("demo_stop_service", default_value="/arachne/demo/stop"),
            DeclareLaunchArgument("demo_preflight_service", default_value="/arachne/demo/preflight"),
            DeclareLaunchArgument("demo_status_service", default_value="/arachne/demo/status"),
            DeclareLaunchArgument("demo_state_topic", default_value="/arachne/demo/state"),
            DeclareLaunchArgument("gripper_settle_sec", default_value="2.0"),
            DeclareLaunchArgument("arm_jog_position_tolerance", default_value="0.0008"),
            DeclareLaunchArgument("arm_orientation_tolerance", default_value="0.01"),
            DeclareLaunchArgument("arm_jog_orientation_tolerance", default_value="0.004"),
            DeclareLaunchArgument("arm_keepout_enabled", default_value="true"),
            DeclareLaunchArgument("arm_base_xyz", default_value="0.22,0.0,0.105"),
            DeclareLaunchArgument("arm_base_rpy", default_value="0.0,0.0,1.57079632679"),
            DeclareLaunchArgument("rear_rack_keepout_min_xyz", default_value="-0.41,-0.22,0.04"),
            DeclareLaunchArgument("rear_rack_keepout_max_xyz", default_value="0.09,0.22,0.82"),
            DeclareLaunchArgument(
                "front_basket_keepout_min_xyz", default_value="0.4215,-0.11,-0.1235"
            ),
            DeclareLaunchArgument(
                "front_basket_keepout_max_xyz", default_value="0.6655,0.11,0.0635"
            ),
            DeclareLaunchArgument("arm_keepout_sample_step_m", default_value="0.035"),
            DeclareLaunchArgument("arm_keepout_joint_step_rad", default_value="0.06"),
            DeclareLaunchArgument(
                "arm_home_joints_deg",
                default_value="-90.00,11.55,95.09,27.80,96.07,43.79",
            ),
            DeclareLaunchArgument(
                "arm_install_joints_deg",
                default_value="-90.00,11.55,95.09,27.80,96.07,43.79",
            ),
            DeclareLaunchArgument("aubo_payload_mass_kg", default_value="0.818"),
            DeclareLaunchArgument("aubo_payload_cog", default_value="0.039927,0.045067,0.143233"),
            DeclareLaunchArgument("aubo_payload_aom", default_value="0,0,0"),
            DeclareLaunchArgument("aubo_payload_inertia", default_value="0,0,0,0,0,0"),
            DeclareLaunchArgument("arm_goal_tolerance", default_value="0.04"),
            DeclareLaunchArgument("aubo_teach_exit_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("recording_dir", default_value="recordings/teach"),
            DeclareLaunchArgument(
                "teach_config_path",
                default_value="recordings/teach/teach_panel_config.json",
            ),
            DeclareLaunchArgument("teach_config_autoload", default_value="true"),
            DeclareLaunchArgument("workspace_root", default_value=""),
            DeclareLaunchArgument("runtime_log_root", default_value="log/teach_panel"),
            DeclareLaunchArgument(
                "autostart_managed_processes",
                default_value="camera,depth_pointcloud,viewer,grasp_server,cleanup_server",
            ),
            DeclareLaunchArgument("service_stop_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument(
                "camera_command",
                default_value=(
                    "ros2 launch arachne_sensors gemini335.launch.py "
                    "publish_pointcloud:=false with_color_view:=false with_depth_view:=false "
                    "with_tf:=true camera_parent_frame:=ee_camera_link "
                    "color_width:=640 color_height:=480 color_fps:=30.0 "
                    "depth_width:=640 depth_height:=480 depth_fps:=15.0 "
                    "color_v4l2_controls:=brightness=20,exposure_auto=0,exposure_absolute=45,gain=0 "
                    "camera_optical_x:=0.0201 camera_optical_y:=0.0 "
                    "camera_optical_z:=0.05 camera_optical_roll:=0.0 "
                    "camera_optical_pitch:=0.0 camera_optical_yaw:=-1.570796327 "
                    "projection_flip_x:=true projection_flip_y:=true color_yuv_layout:=YUYV "
                    "color_capture_mode:=v4l2-ctl"
                ),
            ),
            DeclareLaunchArgument(
                "camera_view_command",
                default_value=(
                    "ros2 run arachne_operator raw_image_viewer "
                    "--topic /camera/color/image_raw --window \"Arachne Raw Camera\" --max-fps 30"
                ),
            ),
            DeclareLaunchArgument(
                "yolo_view_command",
                default_value=(
                    "ros2 run arachne_operator raw_image_viewer "
                    "--topic /camera/color/image_raw --window \"智慧出行\" --max-fps 30 "
                    "--yolo-model yolo_workspace/weights/trash_yolo26n_seg_best.onnx "
                    "--yolo-task segment --yolo-imgsz 640 --yolo-conf 0.25 --yolo-every 5 "
                    "--yolo-device cpu --yolo-venv yolo_workspace/.venv"
                ),
            ),
            DeclareLaunchArgument(
                "depth_pointcloud_command",
                default_value=(
                    "ros2 run arachne_sensors depth_to_pointcloud --ros-args "
                    "-p frames:=1 -p stride:=4 -p max_depth_m:=3.0 "
                    "-p projection_flip_x:=false -p projection_flip_y:=false "
                    "-p continuous:=true -p target_frame:=base_link "
                    "-p min_target_z_m:=-10.0 -p max_target_z_m:=0.0 "
                    "-p min_publish_points:=1000 "
                    "-p exit_after_publish:=false "
                    "-p pointcloud_topic:=/arachne/debug/depth_points"
                ),
            ),
            DeclareLaunchArgument(
                "slam_command",
                default_value=(
                    "ros2 launch arachne_nav nav2_lidar.launch.py "
                    "with_lslidar_driver:=true with_pointcloud_to_scan:=true "
                    "with_robot_state_publisher:=false with_rviz:=true"
                ),
            ),
            DeclareLaunchArgument(
                "grasp_server_command",
                default_value=(
                    "ros2 launch arachne_operator grasp_task_server.launch.py "
                    "execute_real:=true confirm_execute_real:=true with_rviz:=false "
                    "confidence:=0.03 "
                    "grasp_base_offset:=0.03,0.0,0.0 "
                    "real_fixed_post_grasp:=true "
                    "release_point:=basket_a "
                    "release_point_b_joints:=-1.407594293659515,0.07478614140534845,2.611345219296526,1.0028015483718502,1.6581842928233566,0.8873273897703009 "
                    "real_fixed_search_joints:=-1.611779,-0.457910,1.071527,-0.044520,1.575231,0.771459 "
                    "real_sdk_move_speed:=0.18 "
                    "real_sdk_move_accel:=0.25 "
                    "real_gripper_settle_sec:=2.0 "
                    "real_preopen_gripper:=true "
                    "prefer_aubo_move_joint_action:=false "
                    "aubo_move_joint_fallback_internal:=false "
                    "extra_args:='--planner-backend local --imgsz 640 --inference-period 0.5 --min-detection-mask-area-px 0 "
                    "--reject-label-keywords=person,kite --planning-key-waypoints grasp "
                    "--detection-min-center-y-ratio 0.38 "
                    "--preferred-label-keywords bottle,carton,can,cup,container,jar,box "
                    "--arm-collision-samples-per-link 1 --arm-collision-radius 0.018 "
                    "--collision-margin 0.0 --rear-rack-collision-margin 0.0 "
                    "--trajectory-max-duration 8 --max-grasp-orientation-candidates 1 "
                    "--local-planning-timeout-sec 4.0 --local-ik-max-iterations 120 "
                    "--lock-grasp-orientation --grasp-topdown-max-tilt-deg 20 "
                    "--grasp-orientation-yaw-offsets-deg 0 --grasp-orientation-tilt-offsets-deg 0 "
                    "--local-position-tolerance 0.010 --local-orientation-tolerance 0.35 "
                    "--real-sdk-arrival-timeout-padding 10 "
                    "--real-sdk-max-targets 4 --real-sdk-semantic-targets-only' "
                    "preview_on_start:=true warm_execute_preview:=true planning_recovery_base_enabled:=false skip_preflight:=true "
                    "preflight_timeout_sec:=0.5 require_odom:=false require_joint_states:=false require_camera_topics:=false "
                    "require_aubo_status:=false require_gripper_status:=true "
                    "max_grasp_attempts:=2 retry_on_gripper_miss:=true"
                ),
            ),
            DeclareLaunchArgument(
                "cleanup_server_command",
                default_value=(
                    "ros2 launch arachne_operator road_cleanup_task_server.launch.py "
                    "patrol_pattern:=line patrol_distance_m:=1.0 patrol_step_m:=0.20 "
                    "max_round_trips:=1 loop:=false "
                    "patrol_base_speed_mps:=0.05 base_step_timeout_sec:=120.0 "
                    "grasp_timeout_sec:=180.0 "
                    "candidate_min_base_z_m:=-0.18 candidate_max_reach_m:=1.03 "
                    "reach_recovery_enabled:=false "
                    "scan_warmup_sec:=4.0 initial_detection_wait_sec:=45.0 "
                    "skip_preflight:=true move_to_search_pose_before_start:=true require_search_pose_before_start:=true "
                    "required_search_joints:=-1.611779,-0.457910,1.071527,-0.044520,1.575231,0.771459 "
                    "required_search_tolerance_rad:=0.08 "
                    "detection_confidence:=0.08 detection_timeout_sec:=3.0"
                ),
            ),
            DeclareLaunchArgument(
                "step_cleanup_server_command",
                default_value="ros2 launch arachne_operator step_cleanup_demo.launch.py",
            ),
            DeclareLaunchArgument(
                "hand_eye_calibrator_command",
                default_value="ros2 run arachne_operator apriltag_hand_eye_calibrator",
            ),
            DeclareLaunchArgument("grasp_task_state_topic", default_value="/arachne/grasp_task/state"),
            DeclareLaunchArgument("grasp_task_start_service", default_value="/arachne/grasp_task/start"),
            DeclareLaunchArgument("grasp_task_stop_service", default_value="/arachne/grasp_task/stop"),
            DeclareLaunchArgument(
                "grasp_task_restore_service", default_value="/arachne/grasp_task/restore"
            ),
            DeclareLaunchArgument("grasp_task_status_service", default_value="/arachne/grasp_task/status"),
            DeclareLaunchArgument("grasp_task_preflight_service", default_value="/arachne/grasp_task/preflight"),
            DeclareLaunchArgument("cleanup_task_state_topic", default_value="/arachne/road_cleanup/state"),
            DeclareLaunchArgument("cleanup_task_start_service", default_value="/arachne/road_cleanup/start"),
            DeclareLaunchArgument("cleanup_task_pause_service", default_value="/arachne/road_cleanup/pause"),
            DeclareLaunchArgument(
                "cleanup_task_return_home_service", default_value="/arachne/road_cleanup/return_home"
            ),
            DeclareLaunchArgument("cleanup_task_stop_service", default_value="/arachne/road_cleanup/stop"),
            DeclareLaunchArgument("cleanup_task_status_service", default_value="/arachne/road_cleanup/status"),
            DeclareLaunchArgument("cleanup_task_preflight_service", default_value="/arachne/road_cleanup/preflight"),
            DeclareLaunchArgument("step_cleanup_state_topic", default_value="/arachne/step_cleanup/state"),
            DeclareLaunchArgument("step_cleanup_start_service", default_value="/arachne/step_cleanup/start"),
            DeclareLaunchArgument("step_cleanup_stop_service", default_value="/arachne/step_cleanup/stop"),
            DeclareLaunchArgument("step_cleanup_status_service", default_value="/arachne/step_cleanup/status"),
            DeclareLaunchArgument("step_cleanup_preflight_service", default_value="/arachne/step_cleanup/preflight"),
            visualization,
            camera,
            real_bringup,
            demo_orchestrator,
            teach_panel_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=teach_panel_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="teach panel closed"))],
                )
            ),
        ]
    )
