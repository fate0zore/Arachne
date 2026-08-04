from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DISPLAY_ARM_ZEROS = {
    "aubo_shoulder_joint": -1.5707963267949,
    "aubo_upperArm_joint": 0.201570428261868,
    "aubo_foreArm_joint": 1.65970467002488,
    "aubo_wrist1_joint": 0.485178041391533,
    "aubo_wrist2_joint": 1.67675136677345,
    "aubo_wrist3_joint": 0.76432946885334,
}


def _float_launch_arg(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _prefix_robot_links(robot_description: str, prefix: str) -> str:
    if not prefix:
        return robot_description
    root = ET.fromstring(robot_description)
    for link in root.findall("link"):
        name = link.get("name")
        if name and not name.startswith(prefix):
            link.set("name", f"{prefix}{name}")
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None:
            link = parent.get("link")
            if link and not link.startswith(prefix):
                parent.set("link", f"{prefix}{link}")
        if child is not None:
            link = child.get("link")
            if link and not link.startswith(prefix):
                child.set("link", f"{prefix}{link}")
    for gazebo in root.findall("gazebo"):
        reference = gazebo.get("reference")
        if reference and not reference.startswith(prefix):
            gazebo.set("reference", f"{prefix}{reference}")
    return ET.tostring(root, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory("arachne_description"))
    model_path = pkg_share / "urdf" / "arachne.urdf.xacro"
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context))
    generated_urdf = Path("/tmp/arachne_display.urdf")

    mappings = {
        "arm_mount_xyz": LaunchConfiguration("arm_mount_xyz").perform(context),
        "arm_mount_rpy": LaunchConfiguration("arm_mount_rpy").perform(context),
        "ee_support_xyz": LaunchConfiguration("ee_support_xyz").perform(context),
        "ee_support_rpy": LaunchConfiguration("ee_support_rpy").perform(context),
        "tool_adapter_xyz": LaunchConfiguration("tool_adapter_xyz").perform(context),
        "tool_adapter_rpy": LaunchConfiguration("tool_adapter_rpy").perform(context),
        "gripper_type": LaunchConfiguration("gripper_type").perform(context),
        "with_lidar": LaunchConfiguration("with_lidar").perform(context),
        "with_ee_camera": LaunchConfiguration("with_ee_camera").perform(context),
        "with_rear_rack": LaunchConfiguration("with_rear_rack").perform(context),
        "with_front_basket": LaunchConfiguration("with_front_basket").perform(context),
        "front_basket_xyz": LaunchConfiguration("front_basket_xyz").perform(context),
        "front_basket_rpy": LaunchConfiguration("front_basket_rpy").perform(context),
        "rear_rack_xyz": LaunchConfiguration("rear_rack_xyz").perform(context),
        "rear_rack_rpy": LaunchConfiguration("rear_rack_rpy").perform(context),
        "lidar_xyz": LaunchConfiguration("lidar_xyz").perform(context),
        "lidar_rpy": LaunchConfiguration("lidar_rpy").perform(context),
        "ee_camera_xyz": LaunchConfiguration("ee_camera_xyz").perform(context),
        "ee_camera_rpy": LaunchConfiguration("ee_camera_rpy").perform(context),
    }

    display_frame_prefix = LaunchConfiguration("display_frame_prefix").perform(context)
    camera_depth_frame = f"{display_frame_prefix}{LaunchConfiguration('camera_depth_frame').perform(context)}"
    robot_description = _prefix_robot_links(
        xacro.process_file(str(model_path), mappings=mappings).toxml(),
        display_frame_prefix,
    )
    generated_urdf.write_text(robot_description)
    display_arm_zeros = {
        f"zeros.{joint_name}": _float_launch_arg(context, joint_name)
        for joint_name in DISPLAY_ARM_ZEROS
    }
    display_base_link = f"{display_frame_prefix}base_link"

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="arachne_display_robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                }
            ],
            remappings=[
                ("joint_states", LaunchConfiguration("display_joint_states_topic")),
                ("robot_description", LaunchConfiguration("display_robot_description_topic")),
            ],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="default_joint_state_publisher",
            arguments=[str(generated_urdf)],
            parameters=[display_arm_zeros],
            remappings=[("joint_states", "/arachne/default_joint_states")],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            arguments=[str(generated_urdf)],
            parameters=[display_arm_zeros],
            remappings=[("joint_states", "/arachne/gui_joint_states")],
            condition=IfCondition(LaunchConfiguration("use_gui")),
            output="screen",
        ),
        Node(
            package="arachne_gripper",
            executable="joint_state_mux",
            name="joint_state_mux",
            parameters=[
                {
                    "publish_rate": ParameterValue(
                        LaunchConfiguration("joint_state_publish_rate"), value_type=float
                    ),
                    "preview_active_timeout_sec": 0.35,
                    "output_topic": LaunchConfiguration("display_joint_states_topic"),
                }
            ],
            output="screen",
        ),
        Node(
            package="arachne_sim",
            executable="base_sim_controller",
            name="base_sim_controller",
            parameters=[
                {
                    "max_linear_velocity": ParameterValue(
                        LaunchConfiguration("base_max_linear_velocity"), value_type=float
                    ),
                    "max_angular_velocity": ParameterValue(
                        LaunchConfiguration("base_max_angular_velocity"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_base_sim")),
            output="screen",
        ),
        Node(
            package="arachne_sim",
            executable="base_teleop_gui",
            name="base_teleop_gui",
            parameters=[
                {
                    "linear_speed": ParameterValue(
                        LaunchConfiguration("base_linear_speed"), value_type=float
                    ),
                    "angular_speed": ParameterValue(
                        LaunchConfiguration("base_angular_speed"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_base_gui")),
            output="screen",
        ),
        Node(
            package="arachne_gripper",
            executable="gripper_sim_controller",
            name="gripper_sim_controller",
            parameters=[
                {
                    "profile": LaunchConfiguration("gripper_sim_profile"),
                    "open_position": ParameterValue(
                        LaunchConfiguration("gripper_open_position"), value_type=float
                    ),
                    "closed_position": ParameterValue(
                        LaunchConfiguration("gripper_closed_position"), value_type=float
                    ),
                    "max_velocity": ParameterValue(
                        LaunchConfiguration("gripper_max_velocity"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_gripper_sim")),
            output="screen",
        ),
        Node(
            package="arachne_gripper",
            executable="gripper_state_gui",
            name="gripper_state_gui",
            condition=IfCondition(LaunchConfiguration("with_gripper_gui")),
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="display_camera_depth_optical_tf",
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
                f"{display_frame_prefix}ee_camera_link",
                "--child-frame-id",
                camera_depth_frame,
            ],
            condition=IfCondition(LaunchConfiguration("with_camera_optical_tf")),
            output="screen",
        ),
        Node(
            package="arachne_sim",
            executable="end_effector_direction_markers",
            name="end_effector_direction_markers",
            parameters=[
                {
                    "marker_topic": LaunchConfiguration("direction_marker_topic"),
                    "tool_frame": f"{display_frame_prefix}tool0",
                    "camera_frame": camera_depth_frame,
                    "tool_axis": ParameterValue(
                        LaunchConfiguration("tool_direction_axis"), value_type=str
                    ),
                    "camera_axis": ParameterValue(
                        LaunchConfiguration("camera_direction_axis"), value_type=str
                    ),
                    "tool_length_m": ParameterValue(
                        LaunchConfiguration("tool_direction_length_m"), value_type=float
                    ),
                    "camera_length_m": ParameterValue(
                        LaunchConfiguration("camera_direction_length_m"), value_type=float
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("with_direction_markers")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("with_rviz")),
            output="screen",
        ),
    ]
    if display_frame_prefix:
        nodes.insert(
            1,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="arachne_display_base_link_bridge",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    display_base_link,
                ],
                output="screen",
            ),
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_mount_xyz", default_value="0.22 0.0 0.105"),
            DeclareLaunchArgument("arm_mount_rpy", default_value="0.0 0.0 1.57079632679"),
            DeclareLaunchArgument("ee_support_xyz", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("ee_support_rpy", default_value="0.0 0.0 2.35619449019"),
            DeclareLaunchArgument("tool_adapter_xyz", default_value="-0.049334103 0.049874070 0.021816675"),
            DeclareLaunchArgument("tool_adapter_rpy", default_value="1.570796327 0.0 0.0"),
            DeclareLaunchArgument("gripper_type", default_value="ms42dc"),
            DeclareLaunchArgument("with_base_sim", default_value="true"),
            DeclareLaunchArgument("with_base_gui", default_value="false"),
            DeclareLaunchArgument("base_linear_speed", default_value="0.25"),
            DeclareLaunchArgument("base_angular_speed", default_value="0.65"),
            DeclareLaunchArgument("base_max_linear_velocity", default_value="0.8"),
            DeclareLaunchArgument("base_max_angular_velocity", default_value="1.4"),
            DeclareLaunchArgument("with_gripper_sim", default_value="false"),
            DeclareLaunchArgument("with_gripper_gui", default_value="false"),
            DeclareLaunchArgument("gripper_sim_profile", default_value="ms42dc"),
            DeclareLaunchArgument("gripper_open_position", default_value="-1.0"),
            DeclareLaunchArgument("gripper_closed_position", default_value="-1.0"),
            DeclareLaunchArgument("gripper_max_velocity", default_value="-1.0"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    Path(get_package_share_directory("arachne_description"))
                    / "rviz"
                    / "arachne_model.rviz"
                ),
            ),
            DeclareLaunchArgument("with_lidar", default_value="true"),
            DeclareLaunchArgument("with_ee_camera", default_value="true"),
            DeclareLaunchArgument("with_rear_rack", default_value="true"),
            DeclareLaunchArgument("with_front_basket", default_value="true"),
            DeclareLaunchArgument("with_direction_markers", default_value="true"),
            DeclareLaunchArgument("with_camera_optical_tf", default_value="true"),
            DeclareLaunchArgument(
                "direction_marker_topic", default_value="/arachne/model/direction_markers"
            ),
            DeclareLaunchArgument("tool_direction_axis", default_value="plus_z"),
            DeclareLaunchArgument("camera_direction_axis", default_value="plus_z"),
            DeclareLaunchArgument("tool_direction_length_m", default_value="0.22"),
            DeclareLaunchArgument("camera_direction_length_m", default_value="0.38"),
            DeclareLaunchArgument("camera_depth_frame", default_value="camera_depth_optical_frame"),
            DeclareLaunchArgument("camera_optical_x", default_value="0.0201"),
            DeclareLaunchArgument("camera_optical_y", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_z", default_value="0.05"),
            DeclareLaunchArgument("camera_optical_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_optical_yaw", default_value="-1.570796327"),
            DeclareLaunchArgument("joint_state_publish_rate", default_value="80.0"),
            DeclareLaunchArgument(
                "display_joint_states_topic", default_value="/arachne/display/joint_states"
            ),
            DeclareLaunchArgument(
                "display_robot_description_topic",
                default_value="/arachne/display/robot_description",
            ),
            DeclareLaunchArgument("display_frame_prefix", default_value=""),
            DeclareLaunchArgument("front_basket_xyz", default_value="0.4655 0.0 -0.0715"),
            DeclareLaunchArgument("front_basket_rpy", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("rear_rack_xyz", default_value="-0.16 0.0 0.105"),
            DeclareLaunchArgument("rear_rack_rpy", default_value="0.0 0.0 1.57079632679"),
            DeclareLaunchArgument("lidar_xyz", default_value="0.0 0.035 0.6223"),
            DeclareLaunchArgument("lidar_rpy", default_value="0.0 0.0 0.0"),
            DeclareLaunchArgument("ee_camera_xyz", default_value="0.0 -0.0741 0.005"),
            DeclareLaunchArgument("ee_camera_rpy", default_value="0.0 0.0 1.57079632679"),
            DeclareLaunchArgument(
                "aubo_shoulder_joint",
                default_value=str(DISPLAY_ARM_ZEROS["aubo_shoulder_joint"]),
            ),
            DeclareLaunchArgument(
                "aubo_upperArm_joint",
                default_value=str(DISPLAY_ARM_ZEROS["aubo_upperArm_joint"]),
            ),
            DeclareLaunchArgument(
                "aubo_foreArm_joint",
                default_value=str(DISPLAY_ARM_ZEROS["aubo_foreArm_joint"]),
            ),
            DeclareLaunchArgument(
                "aubo_wrist1_joint",
                default_value=str(DISPLAY_ARM_ZEROS["aubo_wrist1_joint"]),
            ),
            DeclareLaunchArgument(
                "aubo_wrist2_joint",
                default_value=str(DISPLAY_ARM_ZEROS["aubo_wrist2_joint"]),
            ),
            DeclareLaunchArgument(
                "aubo_wrist3_joint",
                default_value=str(DISPLAY_ARM_ZEROS["aubo_wrist3_joint"]),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
