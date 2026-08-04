from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_node = Node(
        package="arachne_sensors",
        executable="gemini335_v4l2_node",
        name="gemini335_v4l2_node",
        output="screen",
        parameters=[
            {
                "color_device": LaunchConfiguration("color_device"),
                "color_width": ParameterValue(
                    LaunchConfiguration("color_width"), value_type=int
                ),
                "color_height": ParameterValue(
                    LaunchConfiguration("color_height"), value_type=int
                ),
                "color_fps": ParameterValue(LaunchConfiguration("color_fps"), value_type=float),
                "color_fourcc": LaunchConfiguration("color_fourcc"),
                "color_yuv_layout": LaunchConfiguration("color_yuv_layout"),
                "color_v4l2_controls": LaunchConfiguration("color_v4l2_controls"),
                "color_capture_mode": LaunchConfiguration("color_capture_mode"),
                "color_frame_id": LaunchConfiguration("color_frame_id"),
                "publish_color": ParameterValue(
                    LaunchConfiguration("publish_color"), value_type=bool
                ),
                "depth_device": LaunchConfiguration("depth_device"),
                "depth_width": ParameterValue(
                    LaunchConfiguration("depth_width"), value_type=int
                ),
                "depth_height": ParameterValue(
                    LaunchConfiguration("depth_height"), value_type=int
                ),
                "depth_fps": ParameterValue(LaunchConfiguration("depth_fps"), value_type=float),
                "depth_fourcc": LaunchConfiguration("depth_fourcc"),
                "depth_batch_frames": ParameterValue(
                    LaunchConfiguration("depth_batch_frames"), value_type=int
                ),
                "depth_capture_timeout_sec": ParameterValue(
                    LaunchConfiguration("depth_capture_timeout_sec"), value_type=float
                ),
                "depth_frame_id": LaunchConfiguration("depth_frame_id"),
                "depth_scale": ParameterValue(
                    LaunchConfiguration("depth_scale"), value_type=float
                ),
                "pointcloud_min_depth_m": ParameterValue(
                    LaunchConfiguration("pointcloud_min_depth_m"), value_type=float
                ),
                "pointcloud_max_depth_m": ParameterValue(
                    LaunchConfiguration("pointcloud_max_depth_m"), value_type=float
                ),
                "publish_depth": ParameterValue(
                    LaunchConfiguration("publish_depth"), value_type=bool
                ),
                "publish_depth_color": ParameterValue(
                    LaunchConfiguration("publish_depth_color"), value_type=bool
                ),
                "publish_pointcloud": ParameterValue(
                    LaunchConfiguration("publish_pointcloud"), value_type=bool
                ),
                "pointcloud_decimation": ParameterValue(
                    LaunchConfiguration("pointcloud_decimation"), value_type=int
                ),
                "pointcloud_rate": ParameterValue(
                    LaunchConfiguration("pointcloud_rate"), value_type=float
                ),
                "camera_fx": ParameterValue(LaunchConfiguration("camera_fx"), value_type=float),
                "camera_fy": ParameterValue(LaunchConfiguration("camera_fy"), value_type=float),
                "factory_intrinsics_file": LaunchConfiguration("factory_intrinsics_file"),
                "projection_flip_x": ParameterValue(
                    LaunchConfiguration("projection_flip_x"), value_type=bool
                ),
                "projection_flip_y": ParameterValue(
                    LaunchConfiguration("projection_flip_y"), value_type=bool
                ),
            }
        ],
    )

    color_view = Node(
        package="image_view",
        executable="image_view",
        name="gemini335_color_view",
        remappings=[("image", "/camera/color/image_raw")],
        condition=IfCondition(LaunchConfiguration("with_color_view")),
        output="screen",
    )

    depth_view = Node(
        package="image_view",
        executable="image_view",
        name="gemini335_depth_view",
        remappings=[("image", "/camera/depth/image_color")],
        condition=IfCondition(LaunchConfiguration("with_depth_view")),
        output="screen",
    )

    color_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gemini335_color_tf",
        arguments=[
            "--x",
            LaunchConfiguration("camera_optical_x"),
            "--y",
            LaunchConfiguration("camera_optical_y"),
            "--z",
            LaunchConfiguration("camera_optical_z"),
            "--roll",
            LaunchConfiguration("camera_optical_roll"),
            "--pitch",
            LaunchConfiguration("camera_optical_pitch"),
            "--yaw",
            LaunchConfiguration("camera_optical_yaw"),
            "--frame-id",
            LaunchConfiguration("camera_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("color_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("with_tf")),
        output="screen",
    )

    depth_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gemini335_depth_tf",
        arguments=[
            "--x",
            LaunchConfiguration("camera_optical_x"),
            "--y",
            LaunchConfiguration("camera_optical_y"),
            "--z",
            LaunchConfiguration("camera_optical_z"),
            "--roll",
            LaunchConfiguration("camera_optical_roll"),
            "--pitch",
            LaunchConfiguration("camera_optical_pitch"),
            "--yaw",
            LaunchConfiguration("camera_optical_yaw"),
            "--frame-id",
            LaunchConfiguration("camera_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("depth_frame_id"),
        ],
        condition=IfCondition(LaunchConfiguration("with_tf")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("color_device", default_value="/dev/video6"),
            DeclareLaunchArgument("color_width", default_value="640"),
            DeclareLaunchArgument("color_height", default_value="480"),
            DeclareLaunchArgument("color_fps", default_value="30.0"),
            DeclareLaunchArgument("color_fourcc", default_value="YUYV"),
            DeclareLaunchArgument("color_yuv_layout", default_value="YUYV"),
            DeclareLaunchArgument("color_v4l2_controls", default_value=""),
            DeclareLaunchArgument("color_capture_mode", default_value="opencv"),
            DeclareLaunchArgument("color_frame_id", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("publish_color", default_value="true"),
            DeclareLaunchArgument("depth_device", default_value="/dev/video0"),
            DeclareLaunchArgument("depth_width", default_value="640"),
            DeclareLaunchArgument("depth_height", default_value="480"),
            DeclareLaunchArgument("depth_fps", default_value="5.0"),
            DeclareLaunchArgument("depth_fourcc", default_value="Z16 "),
            DeclareLaunchArgument("depth_batch_frames", default_value="3"),
            DeclareLaunchArgument("depth_capture_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument("depth_frame_id", default_value="camera_depth_optical_frame"),
            DeclareLaunchArgument("depth_scale", default_value="0.001"),
            DeclareLaunchArgument("pointcloud_min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("pointcloud_max_depth_m", default_value="2.0"),
            DeclareLaunchArgument("publish_depth", default_value="true"),
            DeclareLaunchArgument("publish_depth_color", default_value="true"),
            DeclareLaunchArgument("publish_pointcloud", default_value="true"),
            DeclareLaunchArgument("pointcloud_decimation", default_value="6"),
            DeclareLaunchArgument("pointcloud_rate", default_value="5.0"),
            DeclareLaunchArgument("camera_fx", default_value="0.0"),
            DeclareLaunchArgument("camera_fy", default_value="0.0"),
            # Path to factory calibration JSON (see scripts/vision/orbbec_factory_intrinsics.sh).
            # Default is workspace-root-relative (matching teach_panel recording_dir style);
            # empty means legacy behavior (camera_fx/camera_fy params or width*0.9 guess).
            DeclareLaunchArgument(
                "factory_intrinsics_file",
                default_value="config/gemini335_intrinsics/factory_calibration.json",
            ),
            DeclareLaunchArgument("projection_flip_x", default_value="true"),
            DeclareLaunchArgument("projection_flip_y", default_value="true"),
            DeclareLaunchArgument("camera_parent_frame", default_value="ee_camera_link"),
            DeclareLaunchArgument("camera_optical_x", default_value="0.0201"),
            DeclareLaunchArgument("camera_optical_y", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_z", default_value="0.05"),
            DeclareLaunchArgument("camera_optical_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_yaw", default_value="-1.570796327"),
            DeclareLaunchArgument("with_tf", default_value="true"),
            DeclareLaunchArgument("with_color_view", default_value="false"),
            DeclareLaunchArgument("with_depth_view", default_value="false"),
            camera_node,
            color_tf,
            depth_tf,
            color_view,
            depth_view,
        ]
    )
