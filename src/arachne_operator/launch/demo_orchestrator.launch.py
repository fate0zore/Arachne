from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("gripper_command_topic", default_value="/arachne/gripper/command"),
            DeclareLaunchArgument("gripper_status_topic", default_value="/arachne/gripper/status"),
            DeclareLaunchArgument("aubo_teach_command_topic", default_value="/arachne/aubo/teach_command"),
            DeclareLaunchArgument("aubo_move_joint_action_name", default_value="/arachne/aubo/move_joint"),
            DeclareLaunchArgument("camera_color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("camera_depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("grasp_task_start_service", default_value="/arachne/grasp_task/start"),
            DeclareLaunchArgument("grasp_task_stop_service", default_value="/arachne/grasp_task/stop"),
            DeclareLaunchArgument(
                "grasp_task_preflight_service", default_value="/arachne/grasp_task/preflight"
            ),
            DeclareLaunchArgument("cleanup_task_start_service", default_value="/arachne/road_cleanup/start"),
            DeclareLaunchArgument("cleanup_task_stop_service", default_value="/arachne/road_cleanup/stop"),
            DeclareLaunchArgument(
                "cleanup_task_preflight_service",
                default_value="/arachne/road_cleanup/preflight",
            ),
            DeclareLaunchArgument("skip_task_preflight", default_value="true"),
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
                    "camera_optical_z:=0.2196 camera_optical_roll:=0.196 "
                    "camera_optical_pitch:=-0.024 camera_optical_yaw:=-1.570796327 "
                    "projection_flip_x:=true projection_flip_y:=true color_yuv_layout:=YUYV"
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
                "camera_view_command",
                default_value=(
                    "ros2 run arachne_operator raw_image_viewer "
                    "--topic /camera/color/image_raw --window \"Arachne Raw Camera\" --max-fps 30"
                ),
            ),
            DeclareLaunchArgument(
                "grasp_server_command",
                default_value=(
                    "ros2 launch arachne_operator grasp_task_server.launch.py "
                    "execute_real:=true confirm_execute_real:=true with_rviz:=false "
                    "confidence:=0.08 "
                    "real_fixed_post_grasp:=true "
                    "real_fixed_search_joints:=-1.629044,0.031622,1.684745,0.079056,1.575197,0.754000 "
                    "real_sdk_move_speed:=0.36 real_sdk_move_accel:=0.60 "
                    "real_gripper_settle_sec:=2.0 "
                    "real_preopen_gripper:=true "
                    "aubo_move_joint_fallback_internal:=false "
                    "extra_args:='--planner-backend local --imgsz 640 --min-detection-mask-area-px 0 "
                    "--reject-label-keywords= --planning-key-waypoints approach,grasp "
                    "--detection-min-center-y-ratio 0.38 "
                    "--preferred-label-keywords bottle,carton,can,cup,container,jar,box "
                    "--arm-collision-samples-per-link 1 --arm-collision-radius 0.018 "
                    "--collision-margin 0.0 --rear-rack-collision-margin 0.0 "
                    "--trajectory-max-duration 8 --max-grasp-orientation-candidates 1 "
                    "--local-planning-timeout-sec 4.0 --local-ik-max-iterations 120 "
                    "--lock-grasp-orientation --grasp-topdown-max-tilt-deg 20 "
                    "--grasp-orientation-yaw-offsets-deg 0 --grasp-orientation-tilt-offsets-deg 0 "
                    "--local-position-tolerance 0.050 --local-orientation-tolerance 0.35 "
                    "--real-sdk-arrival-timeout-padding 10 "
                    "--real-sdk-max-targets 4 --real-sdk-semantic-targets-only' "
                    "preview_on_start:=false warm_execute_preview:=false planning_recovery_base_enabled:=false skip_preflight:=true "
                    "preflight_timeout_sec:=0.5 require_odom:=false require_joint_states:=false require_camera_topics:=false "
                    "require_aubo_status:=false require_gripper_status:=false "
                    "max_grasp_attempts:=2 retry_on_gripper_miss:=true"
                ),
            ),
            DeclareLaunchArgument(
                "cleanup_server_command",
                default_value=(
                    "ros2 launch arachne_operator road_cleanup_task_server.launch.py "
                    "patrol_pattern:=line patrol_distance_m:=1.5 patrol_step_m:=0.20 "
                    "max_round_trips:=1 loop:=false "
                    "patrol_base_speed_mps:=0.08 base_step_timeout_sec:=5.0 "
                    "grasp_timeout_sec:=25.0 "
                    "candidate_min_base_z_m:=-0.18 candidate_max_reach_m:=1.03 "
                    "reach_recovery_enabled:=false initial_detection_wait_sec:=2.0 "
                    "skip_preflight:=true require_search_pose_before_start:=false "
                    "required_search_joints:=-1.629044,0.031622,1.684745,0.079056,1.575197,0.754000 "
                    "required_search_tolerance_rad:=0.08 "
                    "detection_confidence:=0.08 detection_timeout_sec:=3.0"
                ),
            ),
            DeclareLaunchArgument("service_stop_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument("runtime_log_root", default_value="log/demo_orchestrator"),
            DeclareLaunchArgument("workspace_root", default_value=""),
            Node(
                package="arachne_operator",
                executable="demo_orchestrator",
                name="demo_orchestrator",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                        "gripper_status_topic": LaunchConfiguration("gripper_status_topic"),
                        "aubo_teach_command_topic": LaunchConfiguration("aubo_teach_command_topic"),
                        "aubo_move_joint_action_name": LaunchConfiguration(
                            "aubo_move_joint_action_name"
                        ),
                        "camera_color_topic": LaunchConfiguration("camera_color_topic"),
                        "camera_depth_topic": LaunchConfiguration("camera_depth_topic"),
                        "grasp_task_start_service": LaunchConfiguration("grasp_task_start_service"),
                        "grasp_task_stop_service": LaunchConfiguration("grasp_task_stop_service"),
                        "grasp_task_preflight_service": LaunchConfiguration(
                            "grasp_task_preflight_service"
                        ),
                        "cleanup_task_start_service": LaunchConfiguration(
                            "cleanup_task_start_service"
                        ),
                        "cleanup_task_stop_service": LaunchConfiguration(
                            "cleanup_task_stop_service"
                        ),
                        "cleanup_task_preflight_service": LaunchConfiguration(
                            "cleanup_task_preflight_service"
                        ),
                        "skip_task_preflight": ParameterValue(
                            LaunchConfiguration("skip_task_preflight"), value_type=bool
                        ),
                        "camera_command": LaunchConfiguration("camera_command"),
                        "depth_pointcloud_command": LaunchConfiguration("depth_pointcloud_command"),
                        "camera_view_command": LaunchConfiguration("camera_view_command"),
                        "grasp_server_command": LaunchConfiguration("grasp_server_command"),
                        "cleanup_server_command": LaunchConfiguration("cleanup_server_command"),
                        "service_stop_timeout_sec": ParameterValue(
                            LaunchConfiguration("service_stop_timeout_sec"), value_type=float
                        ),
                        "runtime_log_root": LaunchConfiguration("runtime_log_root"),
                        "workspace_root": LaunchConfiguration("workspace_root"),
                        "autostart": ParameterValue(
                            LaunchConfiguration("autostart"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
