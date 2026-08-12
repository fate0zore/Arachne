from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arachne_operator.kinematics import AuboI5Kinematics

REAL_ARM_JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class RealHardwareAcceptanceTest(Node):
    def __init__(self) -> None:
        super().__init__("arachne_real_hardware_acceptance_test")
        self.declare_parameter("confirm_motion", False)
        self.declare_parameter("run_base_test", True)
        self.declare_parameter("run_arm_test", True)
        self.declare_parameter("run_gripper_test", True)
        self.declare_parameter("sequence_mode", "parallel")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("arm_command_mode", "action")
        self.declare_parameter(
            "arm_follow_joint_trajectory_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("arm_trajectory_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter(
            "legacy_arm_trajectory_topic", "/aubo_arm_controller/joint_trajectory"
        )
        self.declare_parameter("arm_state_joint_names", ",".join(REAL_ARM_JOINTS))
        self.declare_parameter("arm_command_joint_names", ",".join(REAL_ARM_JOINTS))
        self.declare_parameter("gripper_command_topic", "/arachne/gripper/command")
        self.declare_parameter("base_distance_m", 0.2)
        self.declare_parameter("base_linear_speed", 0.06)
        self.declare_parameter("base_yaw_deg", 30.0)
        self.declare_parameter("base_angular_speed", 0.22)
        self.declare_parameter("base_distance_tolerance", 0.015)
        self.declare_parameter("base_yaw_tolerance_deg", 1.5)
        self.declare_parameter("base_settle_sec", 0.8)
        self.declare_parameter("arm_z_delta_m", 0.2)
        self.declare_parameter("arm_z_frame", "aubo_base")
        self.declare_parameter("arm_duration_sec", 5.0)
        self.declare_parameter("arm_settle_sec", 1.0)
        self.declare_parameter("arm_position_tolerance", 0.008)
        self.declare_parameter("arm_ik_damping", 0.08)
        self.declare_parameter("arm_ik_max_iterations", 180)
        self.declare_parameter("arm_ik_max_step", 0.06)
        self.declare_parameter("arm_max_joint_delta", 1.0)
        self.declare_parameter("arm_goal_tolerance", 0.03)
        self.declare_parameter("arm_goal_time_margin_sec", 4.0)
        self.declare_parameter("arm_circle_axis", 1)
        self.declare_parameter("arm_circle_radius_m", 0.1)
        self.declare_parameter("arm_circle_points", 32)
        self.declare_parameter("arm_circle_revolutions", 1.0)
        self.declare_parameter("arm_circle_max_joint_delta", 0.75)
        self.declare_parameter("gripper_cycles", 5)
        self.declare_parameter("gripper_pause_sec", 4.5)
        self.declare_parameter("gripper_final_state", "open")
        self.declare_parameter("feedback_timeout_sec", 8.0)

        self.confirm_motion = bool(self.get_parameter("confirm_motion").value)
        self.latest_odom: Odometry | None = None
        self.latest_joint_state: JointState | None = None
        self.current_arm: dict[str, float] = {}
        self.arm_state_joint_names = self._parse_joint_names(
            str(self.get_parameter("arm_state_joint_names").value)
        )
        self.arm_command_joint_names = self._parse_joint_names(
            str(self.get_parameter("arm_command_joint_names").value)
        )
        if len(self.arm_state_joint_names) != 6 or len(self.arm_command_joint_names) != 6:
            raise ValueError("arm_state_joint_names and arm_command_joint_names must list 6 joints")
        self.kinematics = AuboI5Kinematics()
        self.arm_command_mode = str(self.get_parameter("arm_command_mode").value).strip().lower()
        if self.arm_command_mode not in ("topic", "action"):
            raise ValueError("arm_command_mode must be topic or action")

        arm_topics = []
        for topic in (
            str(self.get_parameter("arm_trajectory_topic").value),
            str(self.get_parameter("legacy_arm_trajectory_topic").value),
        ):
            if topic and topic not in arm_topics:
                arm_topics.append(topic)

        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.arm_publishers = [
            self.create_publisher(JointTrajectory, topic, 10) for topic in arm_topics
        ]
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_follow_joint_trajectory_action").value),
        )
        self.gripper_pub = self.create_publisher(
            String, str(self.get_parameter("gripper_command_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, "/arachne/acceptance/status", 10)

        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )

    def run(self) -> bool:
        self._status("acceptance test loaded")
        self._status(f"plan: {self._plan_summary()}")
        if not self.confirm_motion:
            self._status("dry run only: set confirm_motion:=true to command real hardware", warn=True)
            return True

        try:
            self._status("REAL MOTION CONFIRMED")
            sequence_mode = str(self.get_parameter("sequence_mode").value).strip().lower()
            if sequence_mode == "parallel":
                self._run_parallel_sequence()
            elif sequence_mode == "sequential":
                if bool(self.get_parameter("run_base_test").value):
                    self._wait_for_odom()
                    self._run_base_sequence()
                if bool(self.get_parameter("run_arm_test").value):
                    self._wait_for_arm_state()
                    self._run_arm_sequence()
                if bool(self.get_parameter("run_gripper_test").value):
                    self._run_gripper_sequence()
            else:
                raise RuntimeError("sequence_mode must be parallel or sequential")
            self._publish_stop()
            self._status("acceptance test complete")
            return True
        except Exception as exc:
            self._publish_stop()
            self._publish_gripper("stop")
            self._status(f"acceptance test failed: {exc}", warn=True)
            return False

    def _plan_summary(self) -> str:
        sequence_mode = str(self.get_parameter("sequence_mode").value).strip().lower()
        steps: list[str] = []
        if bool(self.get_parameter("run_base_test").value):
            distance = float(self.get_parameter("base_distance_m").value)
            yaw_deg = float(self.get_parameter("base_yaw_deg").value)
            steps.append(
                f"base +{distance:.3f}m/-{distance:.3f}m, "
                f"left {yaw_deg:.1f}deg/return, right {yaw_deg:.1f}deg/return"
            )
        if bool(self.get_parameter("run_arm_test").value):
            if sequence_mode == "parallel":
                axis = self._arm_circle_axis()
                radius = float(self.get_parameter("arm_circle_radius_m").value)
                steps.append(
                    f"tool0 circle radius {radius:.3f}m around base {axis.upper()} "
                    "with current tool position as center"
                )
            else:
                z_delta = float(self.get_parameter("arm_z_delta_m").value)
                frame = str(self.get_parameter("arm_z_frame").value)
                steps.append(f"tool0 z +{z_delta:.3f}m/return in {frame}")
        if bool(self.get_parameter("run_gripper_test").value):
            if sequence_mode == "parallel":
                steps.append("gripper open-close until base sequence ends")
            else:
                steps.append("gripper open-close x5")
        return "; ".join(steps) if steps else "no subsystem selected"

    def _run_parallel_sequence(self) -> None:
        if not bool(self.get_parameter("run_base_test").value):
            raise RuntimeError("parallel sequence requires run_base_test:=true")
        self._wait_for_odom()
        if bool(self.get_parameter("run_arm_test").value):
            self._wait_for_arm_state()

        arm_result_future = None
        arm_target = None
        arm_reference = None
        if bool(self.get_parameter("run_arm_test").value):
            arm_reference = self._current_arm_vector()
            arm_duration = self._estimated_base_sequence_duration()
            trajectory, arm_target = self._make_arm_circle_trajectory(arm_duration)
            axis = self._arm_circle_axis()
            arm_result_future = self._command_arm_async(trajectory, f"tool-{axis}-circle")

        gripper_enabled = bool(self.get_parameter("run_gripper_test").value)
        gripper_pause = float(self.get_parameter("gripper_pause_sec").value)
        next_gripper_time = time.monotonic()
        gripper_state = "open"
        gripper_toggles = 0

        def tick() -> None:
            nonlocal next_gripper_time, gripper_state, gripper_toggles
            if not gripper_enabled:
                return
            now = time.monotonic()
            if now < next_gripper_time:
                return
            self._publish_gripper(gripper_state)
            gripper_toggles += 1
            self._status(f"parallel gripper: {gripper_state} #{gripper_toggles}")
            gripper_state = "close" if gripper_state == "open" else "open"
            next_gripper_time = now + max(gripper_pause, 0.1)

        self._status("parallel sequence: base starts, gripper toggles until base ends")
        self._run_base_sequence(tick=tick)

        if gripper_enabled:
            final_state = str(self.get_parameter("gripper_final_state").value).strip().lower()
            if final_state in ("open", "close", "stop"):
                self._publish_gripper(final_state)
                self._status(
                    f"parallel gripper: final_state={final_state}, toggles={gripper_toggles}"
                )

        if arm_result_future is not None and arm_target is not None:
            arm_result_timeout = (
                self._estimated_base_sequence_duration()
                + float(self.get_parameter("arm_goal_time_margin_sec").value)
                + float(self.get_parameter("feedback_timeout_sec").value)
            )
            self._finish_arm_action(
                arm_result_future,
                f"tool-{self._arm_circle_axis()}-circle",
                timeout_sec=arm_result_timeout,
            )
            self._wait_for_arm_target(
                arm_target,
                f"tool-{self._arm_circle_axis()}-circle",
                arm_reference,
            )

    def _estimated_base_sequence_duration(self) -> float:
        distance = abs(float(self.get_parameter("base_distance_m").value))
        linear_speed = max(abs(float(self.get_parameter("base_linear_speed").value)), 1e-3)
        yaw = abs(math.radians(float(self.get_parameter("base_yaw_deg").value)))
        angular_speed = max(abs(float(self.get_parameter("base_angular_speed").value)), 1e-3)
        settle = max(float(self.get_parameter("base_settle_sec").value), 0.0)
        return 2.0 * distance / linear_speed + 4.0 * yaw / angular_speed + 6.0 * settle

    def _run_base_sequence(self, tick: Callable[[], None] | None = None) -> None:
        distance = float(self.get_parameter("base_distance_m").value)
        yaw = math.radians(float(self.get_parameter("base_yaw_deg").value))
        self._status("base test: forward")
        self._drive_relative(distance, tick=tick)
        self._settle(tick=tick)
        self._status("base test: backward")
        self._drive_relative(-distance, tick=tick)
        self._settle(tick=tick)
        self._status("base test: left yaw")
        self._turn_relative(yaw, tick=tick)
        self._settle(tick=tick)
        self._status("base test: return from left yaw")
        self._turn_relative(-yaw, tick=tick)
        self._settle(tick=tick)
        self._status("base test: right yaw")
        self._turn_relative(-yaw, tick=tick)
        self._settle(tick=tick)
        self._status("base test: return from right yaw")
        self._turn_relative(yaw, tick=tick)
        self._settle(tick=tick)

    def _run_arm_sequence(self) -> None:
        q_start = self._current_arm_vector()
        start_transform = self.kinematics.fk(q_start)
        start_position = start_transform[:3, 3]
        frame = str(self.get_parameter("arm_z_frame").value).strip().lower()
        z_delta = float(self.get_parameter("arm_z_delta_m").value)
        if frame in ("tool", "tool0", "ee"):
            direction = start_transform[:3, 2]
        else:
            direction = np.array([0.0, 0.0, 1.0], dtype=float)
        target_position = start_position + direction * z_delta

        ok, q_up, error, iterations = self.kinematics.solve_position(
            q_start,
            target_position,
            tolerance=float(self.get_parameter("arm_position_tolerance").value),
            damping=float(self.get_parameter("arm_ik_damping").value),
            max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
            max_step=float(self.get_parameter("arm_ik_max_step").value),
        )
        max_delta = float(np.max(np.abs(q_up - q_start)))
        if not ok:
            raise RuntimeError(f"arm IK failed: best_error={error:.4f} after {iterations} iterations")
        if max_delta > float(self.get_parameter("arm_max_joint_delta").value):
            raise RuntimeError(f"arm IK joint delta too large: {max_delta:.3f} rad")

        self._status(
            "arm test: tool0 z up "
            f"{z_delta:.3f}m, ik_error={error:.4f}, max_joint_delta={max_delta:.3f}"
        )
        self._command_arm(q_up, "up")
        self._wait_for_arm_target(q_up, "up", q_start)
        self._sleep(float(self.get_parameter("arm_settle_sec").value))
        self._status("arm test: return to start")
        self._command_arm(q_start, "return")
        self._wait_for_arm_target(q_start, "return", q_up)
        self._sleep(float(self.get_parameter("arm_settle_sec").value))

    def _make_arm_circle_trajectory(self, duration: float) -> tuple[JointTrajectory, np.ndarray]:
        q_start = self._current_arm_vector()
        start_transform = self.kinematics.fk(q_start)
        start_position = start_transform[:3, 3]
        axis = self._arm_circle_axis()
        radius = float(self.get_parameter("arm_circle_radius_m").value)
        points = max(int(self.get_parameter("arm_circle_points").value), 8)
        revolutions = float(self.get_parameter("arm_circle_revolutions").value)
        max_total_delta = float(self.get_parameter("arm_circle_max_joint_delta").value)
        base_x = np.array([1.0, 0.0, 0.0], dtype=float)
        base_y = np.array([0.0, 1.0, 0.0], dtype=float)
        base_z = np.array([0.0, 0.0, 1.0], dtype=float)
        if axis == "x":
            plane_a, plane_b = base_y, base_z
        elif axis == "y":
            plane_a, plane_b = base_x, base_z
        elif axis == "z":
            plane_a, plane_b = base_x, base_y
        else:
            raise RuntimeError(f"arm_circle_axis must be x, y, or z; got {axis!r}")

        center = np.array(start_position, dtype=float)
        q_seed = np.array(q_start, dtype=float)
        path: list[np.ndarray] = [np.array(q_start, dtype=float)]
        max_delta = 0.0
        max_error = 0.0
        for index in range(0, points + 1):
            theta = 2.0 * math.pi * revolutions * index / points
            target_position = center + radius * (
                math.cos(theta) * plane_a + math.sin(theta) * plane_b
            )
            ok, q_seed, error, iterations = self.kinematics.solve_position(
                q_seed,
                target_position,
                tolerance=float(self.get_parameter("arm_position_tolerance").value),
                damping=float(self.get_parameter("arm_ik_damping").value),
                max_iterations=int(self.get_parameter("arm_ik_max_iterations").value),
                max_step=float(self.get_parameter("arm_ik_max_step").value),
            )
            max_error = max(max_error, error)
            max_delta = max(max_delta, float(np.max(np.abs(q_seed - q_start))))
            if not ok:
                raise RuntimeError(
                    "arm circle IK failed: "
                    f"point={index}/{points}, best_error={error:.4f}, iterations={iterations}"
                )
            if max_delta > max_total_delta:
                raise RuntimeError(
                    "arm circle joint delta too large: "
                    f"{max_delta:.3f} rad > {max_total_delta:.3f} rad"
                )
            path.append(np.array(q_seed, dtype=float))

        # Return to the measured starting pose at the circle center.
        path.append(np.array(q_start, dtype=float))
        trajectory = self._make_arm_multi_point_trajectory(path, duration)
        self._status(
            "arm circle: "
            f"frame=base, axis={axis}, center=current_tool, radius={radius:.3f}m, "
            f"points={points}, duration={duration:.1f}s, "
            f"max_ik_error={max_error:.4f}, max_joint_delta={max_delta:.3f}"
        )
        return trajectory, np.array(q_start, dtype=float)

    def _arm_circle_axis(self) -> str:
        value = self.get_parameter("arm_circle_axis").value
        if isinstance(value, bool):
            index = 1 if value else 0
        elif isinstance(value, int):
            index = value
        else:
            text = str(value).strip().lower()
            aliases = {"x": "x", "tool_x": "x", "y": "y", "tool_y": "y", "z": "z", "tool_z": "z"}
            if text in aliases:
                return aliases[text]
            index = int(text)
        axes = ("x", "y", "z")
        if index < 0 or index >= len(axes):
            raise RuntimeError(f"arm_circle_axis must be 0/x, 1/y, or 2/z; got {value!r}")
        return axes[index]

    def _run_gripper_sequence(self) -> None:
        cycles = int(self.get_parameter("gripper_cycles").value)
        pause = float(self.get_parameter("gripper_pause_sec").value)
        for index in range(cycles):
            self._status(f"gripper test: cycle {index + 1}/{cycles} open")
            self._publish_gripper("open")
            self._sleep(pause)
            self._status(f"gripper test: cycle {index + 1}/{cycles} close")
            self._publish_gripper("close")
            self._sleep(pause)
        final_state = str(self.get_parameter("gripper_final_state").value).strip().lower()
        if final_state in ("open", "close", "stop"):
            self._publish_gripper(final_state)
            self._status(f"gripper test: final_state={final_state}")

    def _drive_relative(self, distance: float, tick: Callable[[], None] | None = None) -> None:
        start = self._current_pose2d()
        heading = np.array([math.cos(start.yaw), math.sin(start.yaw)])
        speed = float(self.get_parameter("base_linear_speed").value)
        tolerance = float(self.get_parameter("base_distance_tolerance").value)
        timeout = abs(distance) / max(abs(speed), 1e-3) + 6.0
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if tick is not None:
                tick()
            pose = self._current_pose2d()
            offset = np.array([pose.x - start.x, pose.y - start.y])
            progress = float(offset @ heading)
            if sign * progress >= abs(distance) - tolerance:
                self._publish_stop()
                self._status(f"base distance reached: {progress:.3f}m target={distance:.3f}m")
                return
            twist = Twist()
            twist.linear.x = sign * abs(speed)
            self.cmd_vel_pub.publish(twist)
            self._spin_sleep(0.05)
        raise TimeoutError(f"base distance timeout target={distance:.3f}m")

    def _turn_relative(self, angle: float, tick: Callable[[], None] | None = None) -> None:
        start = self._current_pose2d()
        speed = float(self.get_parameter("base_angular_speed").value)
        tolerance = math.radians(float(self.get_parameter("base_yaw_tolerance_deg").value))
        timeout = abs(angle) / max(abs(speed), 1e-3) + 6.0
        sign = 1.0 if angle >= 0.0 else -1.0
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if tick is not None:
                tick()
            pose = self._current_pose2d()
            delta = self._angle_diff(pose.yaw, start.yaw)
            if sign * delta >= abs(angle) - tolerance:
                self._publish_stop()
                self._status(
                    f"base yaw reached: {math.degrees(delta):.2f}deg "
                    f"target={math.degrees(angle):.2f}deg"
                )
                return
            twist = Twist()
            twist.angular.z = sign * abs(speed)
            self.cmd_vel_pub.publish(twist)
            self._spin_sleep(0.05)
        raise TimeoutError(f"base yaw timeout target={math.degrees(angle):.2f}deg")

    def _make_arm_trajectory(self, positions: np.ndarray) -> JointTrajectory:
        return self._make_arm_multi_point_trajectory(
            [np.array(positions, dtype=float)],
            float(self.get_parameter("arm_duration_sec").value),
        )

    def _make_arm_multi_point_trajectory(
        self, positions: list[np.ndarray], duration: float
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.arm_command_joint_names)
        count = max(len(positions), 1)
        point_times = [max(duration * index / count, 0.2) for index in range(1, count + 1)]
        trajectory.points = []
        for index, joint_positions in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in joint_positions]
            point.velocities = self._trajectory_point_velocities(positions, point_times, index)
            point.accelerations = [0.0 for _ in point.positions]
            point_time = point_times[index]
            point.time_from_start.sec = int(point_time)
            point.time_from_start.nanosec = int((point_time % 1.0) * 1e9)
            trajectory.points.append(point)
        return trajectory

    def _trajectory_point_velocities(
        self, positions: list[np.ndarray], point_times: list[float], index: int
    ) -> list[float]:
        if len(positions) <= 1 or index <= 0 or index >= len(positions) - 1:
            return [0.0 for _ in positions[index]]
        dt = max(point_times[index + 1] - point_times[index - 1], 1e-6)
        velocity = (positions[index + 1] - positions[index - 1]) / dt
        return [float(value) for value in velocity]

    def _command_arm(self, positions: np.ndarray, label: str) -> None:
        trajectory = self._make_arm_trajectory(positions)
        if self.arm_command_mode == "action":
            self._send_arm_action(trajectory, label)
        else:
            self._publish_arm_topic(trajectory, label)

    def _command_arm_async(self, trajectory: JointTrajectory, label: str):
        if self.arm_command_mode == "action":
            return self._start_arm_action(trajectory, label)
        self._publish_arm_topic(trajectory, label)
        return None

    def _publish_arm_topic(self, trajectory: JointTrajectory, label: str) -> None:
        self._wait_for_arm_topic_subscribers()
        for publisher in self.arm_publishers:
            self.get_logger().info(
                f"publishing arm {label} trajectory on {publisher.topic_name}"
            )
            publisher.publish(trajectory)

    def _send_arm_action(self, trajectory: JointTrajectory, label: str) -> None:
        result_future = self._start_arm_action(trajectory, label)
        self._finish_arm_action(result_future, label)

    def _start_arm_action(self, trajectory: JointTrajectory, label: str):
        action_name = str(self.get_parameter("arm_follow_joint_trajectory_action").value)
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        if not self.arm_action_client.wait_for_server(timeout_sec=timeout):
            raise TimeoutError(f"arm action server not available: {action_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        margin = float(self.get_parameter("arm_goal_time_margin_sec").value)
        goal.goal_time_tolerance.sec = int(margin)
        goal.goal_time_tolerance.nanosec = int((margin % 1.0) * 1e9)

        self.get_logger().info(f"sending arm {label} action goal to {action_name}")
        goal_future = self.arm_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=timeout)
        goal_handle = goal_future.result()
        if goal_handle is None:
            raise TimeoutError(f"arm {label} action goal response timed out")
        if not goal_handle.accepted:
            raise RuntimeError(f"arm {label} action goal rejected")

        return goal_handle.get_result_async()

    def _finish_arm_action(self, result_future, label: str, timeout_sec: float | None = None) -> None:
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        margin = float(self.get_parameter("arm_goal_time_margin_sec").value)
        result_timeout = (
            timeout_sec
            if timeout_sec is not None
            else float(self.get_parameter("arm_duration_sec").value) + margin + timeout
        )
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        result_response = result_future.result()
        if result_response is None:
            raise TimeoutError(f"arm {label} action result timed out")
        result = result_response.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"arm {label} action failed: code={result.error_code} {result.error_string}"
            )
        self.get_logger().info(f"arm {label} action result: SUCCESSFUL")

    def _wait_for_arm_topic_subscribers(self) -> None:
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        deadline = time.monotonic() + timeout
        missing: list[str] = []
        while rclpy.ok() and time.monotonic() < deadline:
            missing = [
                publisher.topic_name
                for publisher in self.arm_publishers
                if publisher.get_subscription_count() <= 0
            ]
            if not missing:
                return
            self._spin_sleep(0.05)
        raise TimeoutError(f"missing arm trajectory subscribers: {missing}")

    def _wait_for_arm_target(
        self, target: np.ndarray, label: str, start_reference: np.ndarray
    ) -> None:
        tolerance = float(self.get_parameter("arm_goal_tolerance").value)
        timeout = (
            float(self.get_parameter("arm_duration_sec").value)
            + float(self.get_parameter("arm_goal_time_margin_sec").value)
            + float(self.get_parameter("feedback_timeout_sec").value)
        )
        deadline = time.monotonic() + timeout
        target_delta = float(np.max(np.abs(target - start_reference)))
        best_error = float(np.max(np.abs(self._current_arm_vector() - target)))
        while rclpy.ok() and time.monotonic() < deadline:
            current = self._current_arm_vector()
            error = float(np.max(np.abs(current - target)))
            best_error = min(best_error, error)
            if error <= tolerance:
                self.get_logger().info(
                    f"arm {label} feedback reached: max_error={error:.4f}, "
                    f"target_delta={target_delta:.4f}"
                )
                return
            self._spin_sleep(0.05)

        current = self._current_arm_vector()
        current_list = [round(float(value), 6) for value in current]
        target_list = [round(float(value), 6) for value in target]
        raise TimeoutError(
            f"arm {label} feedback did not reach target: best_error={best_error:.4f}, "
            f"target_delta={target_delta:.4f}, current={current_list}, target={target_list}"
        )

    def _publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _settle(self, tick: Callable[[], None] | None = None) -> None:
        self._publish_stop()
        self._sleep(float(self.get_parameter("base_settle_sec").value), tick=tick)

    def _sleep(self, seconds: float, tick: Callable[[], None] | None = None) -> None:
        deadline = time.monotonic() + max(seconds, 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            if tick is not None:
                tick()
            self._spin_sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))

    def _spin_sleep(self, seconds: float) -> None:
        rclpy.spin_once(self, timeout_sec=max(seconds, 0.0))

    def _wait_for_odom(self) -> None:
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.latest_odom is None and time.monotonic() < deadline:
            self._spin_sleep(0.05)
        if self.latest_odom is None:
            raise TimeoutError("no /odom feedback")

    def _wait_for_arm_state(self) -> None:
        timeout = float(self.get_parameter("feedback_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.current_arm for name in self.arm_state_joint_names):
                return
            self._spin_sleep(0.05)
        missing = [name for name in self.arm_state_joint_names if name not in self.current_arm]
        raise TimeoutError(f"missing arm joint states: {missing}")

    def _current_pose2d(self) -> Pose2D:
        if self.latest_odom is None:
            raise RuntimeError("odom is not available")
        pose = self.latest_odom.pose.pose
        yaw = self._yaw_from_quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        return Pose2D(pose.position.x, pose.position.y, yaw)

    def _current_arm_vector(self) -> np.ndarray:
        return np.array([self.current_arm[name] for name in self.arm_state_joint_names], dtype=float)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        for name, position in zip(msg.name, msg.position):
            if name in self.arm_state_joint_names:
                self.current_arm[name] = float(position)

    def _parse_joint_names(self, value: str) -> tuple[str, ...]:
        names = tuple(item.strip() for item in value.split(",") if item.strip())
        return names

    def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _angle_diff(self, angle: float, reference: float) -> float:
        return math.atan2(math.sin(angle - reference), math.cos(angle - reference))

    def _status(self, text: str, warn: bool = False) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RealHardwareAcceptanceTest()
    exit_code = 0
    try:
        if not node.run():
            exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        node._publish_stop()
        node._publish_gripper("stop")
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
