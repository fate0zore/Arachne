from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("execute_real", default_value="false"),
            DeclareLaunchArgument("confirm_execute_real", default_value="false"),
            DeclareLaunchArgument("with_rviz", default_value="false"),
            DeclareLaunchArgument("classes", default_value=""),
            DeclareLaunchArgument("confidence", default_value="0.03"),
            DeclareLaunchArgument("device_id", default_value="0"),
            DeclareLaunchArgument("real_execute_backend", default_value="sdk_move_joint"),
            DeclareLaunchArgument("real_return_home", default_value="true"),
            DeclareLaunchArgument("real_fixed_post_grasp", default_value="true"),
            DeclareLaunchArgument(
                "real_fixed_search_joints",
                default_value="-1.629044,0.031622,1.684745,0.079056,1.575197,0.754000",
            ),
            DeclareLaunchArgument(
                "real_fixed_lift_joints",
                default_value="-1.392228627,-0.587456810,1.402798238,0.420158124,1.570706911,0.178573568",
            ),
            DeclareLaunchArgument(
                "real_fixed_basket_joints",
                default_value="-1.0532545919568441,0.13642934952944055,2.6318805595335437,1.0042082880690795,1.5823432026398125,1.308814683852632",
            ),
            DeclareLaunchArgument("release_point", default_value="basket_a"),
            DeclareLaunchArgument(
                "release_point_a_joints",
                default_value="-1.0532545919568441,0.13642934952944055,2.6318805595335437,1.0042082880690795,1.5823432026398125,1.308814683852632",
            ),
            DeclareLaunchArgument(
                "release_point_b_joints",
                default_value="-1.407594293659515,0.07478614140534845,2.611345219296526,1.0028015483718502,1.6581842928233566,0.8873273897703009",
            ),
            DeclareLaunchArgument("real_sdk_move_speed", default_value="0.18"),
            DeclareLaunchArgument("real_sdk_move_accel", default_value="0.25"),
            DeclareLaunchArgument("real_gripper_settle_sec", default_value="2.0"),
            DeclareLaunchArgument("real_preopen_gripper", default_value="false"),
            DeclareLaunchArgument("real_sdk_prewarm", default_value="true"),
            DeclareLaunchArgument("real_sdk_prewarm_wait_sec", default_value="0.25"),
            DeclareLaunchArgument(
                "aubo_teach_flag_path",
                default_value="/tmp/arachne_aubo_teach_mode",
            ),
            DeclareLaunchArgument(
                "aubo_control_owner_path",
                default_value="/tmp/arachne_aubo_control_owner",
            ),
            DeclareLaunchArgument("aubo_control_owner_name", default_value="grasp_task_server"),
            DeclareLaunchArgument("aubo_move_joint_action_name", default_value="/arachne/aubo/move_joint"),
            DeclareLaunchArgument("prefer_aubo_move_joint_action", default_value="false"),
            DeclareLaunchArgument("aubo_move_joint_fallback_internal", default_value="false"),
            DeclareLaunchArgument("aubo_move_joint_wait_server_sec", default_value="0.5"),
            DeclareLaunchArgument("grasp_base_offset", default_value="-0.23,0.01,0.16"),
            DeclareLaunchArgument(
                "extra_args",
                default_value=(
                    "--planner-backend local "
                    "--planning-key-waypoints grasp "
                    "--vertical-approach --no-lock-grasp-orientation "
                    "--tool-orientation-limit-deg 45 "
                    "--grasp-orientation-yaw-offsets-deg 0 "
                    "--grasp-orientation-tilt-offsets-deg 0,8,-8 "
                    "--local-position-tolerance 0.010 --local-orientation-tolerance 0.50 "
                    "--local-planning-timeout-sec 2.0 --local-ik-max-iterations 90 "
                    "--real-gripper-require-capture "
                    "--real-sdk-semantic-targets-only --real-sdk-max-targets 6"
                ),
            ),
            DeclareLaunchArgument("preview_on_start", default_value="false"),
            DeclareLaunchArgument("warm_execute_preview", default_value="false"),
            DeclareLaunchArgument("preview_extra_args", default_value="--planner-backend none --imgsz 640"),
            DeclareLaunchArgument("planning_recovery_base_enabled", default_value="true"),
            DeclareLaunchArgument(
                "planning_recovery_base_sequence",
                default_value="forward:0.10,forward:0.10,forward:0.10",
            ),
            DeclareLaunchArgument("planning_recovery_restore_on_failure", default_value="true"),
            DeclareLaunchArgument("skip_preflight", default_value="false"),
            DeclareLaunchArgument("log_root", default_value="log/grasp_tasks"),
            DeclareLaunchArgument("require_safety_state_machine", default_value="false"),
            DeclareLaunchArgument("require_aubo_status", default_value="false"),
            DeclareLaunchArgument("require_gripper_status", default_value="true"),
            DeclareLaunchArgument("require_odom", default_value="false"),
            DeclareLaunchArgument("require_camera_topics", default_value="false"),
            DeclareLaunchArgument("max_grasp_attempts", default_value="3"),
            DeclareLaunchArgument("retry_on_gripper_miss", default_value="true"),
            DeclareLaunchArgument("gripper_miss_retry_delay_sec", default_value="0.8"),
            DeclareLaunchArgument("set_safety_autonomous_on_start", default_value="true"),
            DeclareLaunchArgument("set_safety_manual_on_finish", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "base_command_topic", default_value="/arachne/grasp_task/base_command"
            ),
            DeclareLaunchArgument(
                "base_state_topic", default_value="/arachne/grasp_task/base_state"
            ),
            DeclareLaunchArgument("base_linear_speed", default_value="0.08"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.30"),
            DeclareLaunchArgument("base_replay_linear_speed", default_value="0.40"),
            DeclareLaunchArgument("base_replay_angular_speed", default_value="0.18"),
            DeclareLaunchArgument("base_position_tolerance", default_value="0.02"),
            DeclareLaunchArgument("base_yaw_tolerance_deg", default_value="0.5"),
            DeclareLaunchArgument("allow_base_commands_during_grasp", default_value="false"),
            Node(
                package="arachne_operator",
                executable="grasp_task_server",
                name="arachne_grasp_task_server",
                output="screen",
                parameters=[
                    {
                        "execute_real": ParameterValue(
                            LaunchConfiguration("execute_real"), value_type=bool
                        ),
                        "confirm_execute_real": ParameterValue(
                            LaunchConfiguration("confirm_execute_real"), value_type=bool
                        ),
                        "with_rviz": ParameterValue(
                            LaunchConfiguration("with_rviz"), value_type=bool
                        ),
                        "classes": LaunchConfiguration("classes"),
                        "confidence": ParameterValue(
                            LaunchConfiguration("confidence"), value_type=float
                        ),
                        "device_id": ParameterValue(
                            LaunchConfiguration("device_id"), value_type=int
                        ),
                        "real_execute_backend": LaunchConfiguration("real_execute_backend"),
                        "real_return_home": ParameterValue(
                            LaunchConfiguration("real_return_home"), value_type=bool
                        ),
                        "real_fixed_post_grasp": ParameterValue(
                            LaunchConfiguration("real_fixed_post_grasp"), value_type=bool
                        ),
                        "real_fixed_search_joints": LaunchConfiguration(
                            "real_fixed_search_joints"
                        ),
                        "real_fixed_lift_joints": LaunchConfiguration(
                            "real_fixed_lift_joints"
                        ),
                        "real_fixed_basket_joints": LaunchConfiguration(
                            "real_fixed_basket_joints"
                        ),
                        "release_point": LaunchConfiguration("release_point"),
                        "release_point_a_joints": LaunchConfiguration(
                            "release_point_a_joints"
                        ),
                        "release_point_b_joints": LaunchConfiguration(
                            "release_point_b_joints"
                        ),
                        "real_sdk_move_speed": ParameterValue(
                            LaunchConfiguration("real_sdk_move_speed"), value_type=float
                        ),
                        "real_sdk_move_accel": ParameterValue(
                            LaunchConfiguration("real_sdk_move_accel"), value_type=float
                        ),
                        "real_gripper_settle_sec": ParameterValue(
                            LaunchConfiguration("real_gripper_settle_sec"), value_type=float
                        ),
                        "real_preopen_gripper": ParameterValue(
                            LaunchConfiguration("real_preopen_gripper"), value_type=bool
                        ),
                        "real_sdk_prewarm": ParameterValue(
                            LaunchConfiguration("real_sdk_prewarm"), value_type=bool
                        ),
                        "real_sdk_prewarm_wait_sec": ParameterValue(
                            LaunchConfiguration("real_sdk_prewarm_wait_sec"), value_type=float
                        ),
                        "aubo_teach_flag_path": LaunchConfiguration("aubo_teach_flag_path"),
                        "aubo_control_owner_path": LaunchConfiguration("aubo_control_owner_path"),
                        "aubo_control_owner_name": LaunchConfiguration("aubo_control_owner_name"),
                        "aubo_move_joint_action_name": LaunchConfiguration(
                            "aubo_move_joint_action_name"
                        ),
                        "prefer_aubo_move_joint_action": ParameterValue(
                            LaunchConfiguration("prefer_aubo_move_joint_action"),
                            value_type=bool,
                        ),
                        "aubo_move_joint_fallback_internal": ParameterValue(
                            LaunchConfiguration("aubo_move_joint_fallback_internal"),
                            value_type=bool,
                        ),
                        "aubo_move_joint_wait_server_sec": ParameterValue(
                            LaunchConfiguration("aubo_move_joint_wait_server_sec"),
                            value_type=float,
                        ),
                        "grasp_base_offset": LaunchConfiguration("grasp_base_offset"),
                        "extra_args": LaunchConfiguration("extra_args"),
                        "preview_on_start": ParameterValue(
                            LaunchConfiguration("preview_on_start"), value_type=bool
                        ),
                        "warm_execute_preview": ParameterValue(
                            LaunchConfiguration("warm_execute_preview"), value_type=bool
                        ),
                        "preview_extra_args": LaunchConfiguration("preview_extra_args"),
                        "planning_recovery_base_enabled": ParameterValue(
                            LaunchConfiguration("planning_recovery_base_enabled"),
                            value_type=bool,
                        ),
                        "planning_recovery_base_sequence": LaunchConfiguration(
                            "planning_recovery_base_sequence"
                        ),
                        "planning_recovery_restore_on_failure": ParameterValue(
                            LaunchConfiguration("planning_recovery_restore_on_failure"),
                            value_type=bool,
                        ),
                        "skip_preflight": ParameterValue(
                            LaunchConfiguration("skip_preflight"), value_type=bool
                        ),
                        "log_root": LaunchConfiguration("log_root"),
                        "require_safety_state_machine": ParameterValue(
                            LaunchConfiguration("require_safety_state_machine"), value_type=bool
                        ),
                        "require_aubo_status": ParameterValue(
                            LaunchConfiguration("require_aubo_status"), value_type=bool
                        ),
                        "require_gripper_status": ParameterValue(
                            LaunchConfiguration("require_gripper_status"), value_type=bool
                        ),
                        "require_odom": ParameterValue(
                            LaunchConfiguration("require_odom"), value_type=bool
                        ),
                        "require_camera_topics": ParameterValue(
                            LaunchConfiguration("require_camera_topics"), value_type=bool
                        ),
                        "max_grasp_attempts": ParameterValue(
                            LaunchConfiguration("max_grasp_attempts"), value_type=int
                        ),
                        "retry_on_gripper_miss": ParameterValue(
                            LaunchConfiguration("retry_on_gripper_miss"), value_type=bool
                        ),
                        "gripper_miss_retry_delay_sec": ParameterValue(
                            LaunchConfiguration("gripper_miss_retry_delay_sec"), value_type=float
                        ),
                        "set_safety_autonomous_on_start": ParameterValue(
                            LaunchConfiguration("set_safety_autonomous_on_start"),
                            value_type=bool,
                        ),
                        "set_safety_manual_on_finish": ParameterValue(
                            LaunchConfiguration("set_safety_manual_on_finish"),
                            value_type=bool,
                        ),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "base_command_topic": LaunchConfiguration("base_command_topic"),
                        "base_state_topic": LaunchConfiguration("base_state_topic"),
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
                        "base_position_tolerance": ParameterValue(
                            LaunchConfiguration("base_position_tolerance"), value_type=float
                        ),
                        "base_yaw_tolerance_deg": ParameterValue(
                            LaunchConfiguration("base_yaw_tolerance_deg"), value_type=float
                        ),
                        "allow_base_commands_during_grasp": ParameterValue(
                            LaunchConfiguration("allow_base_commands_during_grasp"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
