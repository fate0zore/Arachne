from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/arachne/debug/depth_points"),
            DeclareLaunchArgument("stride", default_value="1"),
            DeclareLaunchArgument("max_depth_m", default_value="5.0"),
            DeclareLaunchArgument("frames", default_value="1"),
            DeclareLaunchArgument("continuous", default_value="false"),
            DeclareLaunchArgument("projection_flip_x", default_value="false"),
            DeclareLaunchArgument("projection_flip_y", default_value="false"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("min_target_z_m", default_value="-10.0"),
            DeclareLaunchArgument("max_target_z_m", default_value="0.0"),
            DeclareLaunchArgument("min_publish_points", default_value="1000"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("with_camera", default_value="true"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("aubo_robot_ip", default_value="192.168.127.128"),
            DeclareLaunchArgument("camera_parent_frame", default_value="ee_camera_link"),
            DeclareLaunchArgument("camera_optical_x", default_value="0.0201"),
            DeclareLaunchArgument("camera_optical_y", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_z", default_value="0.05"),
            DeclareLaunchArgument("camera_optical_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_yaw", default_value="-1.570796327"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("arachne_sensors"), "launch", "gemini335.launch.py"]
                    )
                ),
                launch_arguments={
                    "publish_color": "false",
                    "publish_pointcloud": "false",
                    "publish_depth_color": "false",
                    "depth_batch_frames": "1",
                    "with_color_view": "false",
                    "with_depth_view": "false",
                    "with_tf": "true",
                    "camera_parent_frame": LaunchConfiguration("camera_parent_frame"),
                    "camera_optical_x": LaunchConfiguration("camera_optical_x"),
                    "camera_optical_y": LaunchConfiguration("camera_optical_y"),
                    "camera_optical_z": LaunchConfiguration("camera_optical_z"),
                    "camera_optical_roll": LaunchConfiguration("camera_optical_roll"),
                    "camera_optical_pitch": LaunchConfiguration("camera_optical_pitch"),
                    "camera_optical_yaw": LaunchConfiguration("camera_optical_yaw"),
                    "color_width": "640",
                    "color_height": "480",
                    "color_fps": "30.0",
                    "depth_width": "640",
                    "depth_height": "480",
                    "depth_fps": "15.0",
                    "color_v4l2_controls": "brightness=20,exposure_auto=0,exposure_absolute=45,gain=0",
                    "projection_flip_x": "true",
                    "projection_flip_y": "true",
                    "color_yuv_layout": "YUYV",
                }.items(),
                condition=IfCondition(LaunchConfiguration("with_camera")),
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("arachne_operator"),
                                    "launch",
                                    "teach_visualization.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "with_rviz": "false",
                            "input_joint_states_topic": LaunchConfiguration("joint_states_topic"),
                            "with_lidar": "true",
                            "with_lslidar_driver": "false",
                            "with_ee_camera": "true",
                            "aubo_sdk_fallback_enabled": "true",
                            "aubo_sdk_ip": LaunchConfiguration("aubo_robot_ip"),
                            "aubo_sdk_port": "30004",
                            "aubo_sdk_timeout_sec": "0.5",
                        }.items(),
                    )
                ],
            ),
            Node(
                package="arachne_sensors",
                executable="depth_to_pointcloud",
                name="arachne_depth_to_pointcloud",
                output="screen",
                parameters=[
                    {
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                        "stride": ParameterValue(LaunchConfiguration("stride"), value_type=int),
                        "max_depth_m": ParameterValue(
                            LaunchConfiguration("max_depth_m"), value_type=float
                        ),
                        "frames": ParameterValue(LaunchConfiguration("frames"), value_type=int),
                        "continuous": ParameterValue(
                            LaunchConfiguration("continuous"), value_type=bool
                        ),
                        "projection_flip_x": ParameterValue(
                            LaunchConfiguration("projection_flip_x"), value_type=bool
                        ),
                        "projection_flip_y": ParameterValue(
                            LaunchConfiguration("projection_flip_y"), value_type=bool
                        ),
                        "target_frame": LaunchConfiguration("target_frame"),
                        "min_target_z_m": ParameterValue(
                            LaunchConfiguration("min_target_z_m"), value_type=float
                        ),
                        "max_target_z_m": ParameterValue(
                            LaunchConfiguration("max_target_z_m"), value_type=float
                        ),
                        "min_publish_points": ParameterValue(
                            LaunchConfiguration("min_publish_points"), value_type=int
                        ),
                        "exit_after_publish": False,
                    }
                ],
            ),
            Node(
                package="arachne_sim",
                executable="end_effector_direction_markers",
                name="end_effector_direction_markers",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="arachne_depth_snapshot_rviz",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("arachne_sensors"), "rviz", "depth_snapshot.rviz"]
                    ),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("with_rviz")),
            ),
        ]
    )
