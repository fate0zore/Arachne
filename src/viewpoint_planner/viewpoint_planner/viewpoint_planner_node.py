"""Viewpoint planner ROS node: plans viewpoints around a target via service.

Only plans — never moves. The caller passes the target position (e.g. from
taco_instances.base_grasp_xyz); this node is decoupled from perception.
"""

from __future__ import annotations

import argparse
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from visualization_msgs.msg import MarkerArray

from arachne_operator.kinematics import AUBO_I5_JOINT_LIMITS, REAL_ARM_JOINT_NAMES
from viewpoint_planner.ik_filter import IkReachabilityFilter
from viewpoint_planner.markers import build_viewpoint_marker_array
from viewpoint_planner.strategies.base import get_strategy
from viewpoint_planner.tf_utils import (
    compose_tool0_from_camera,
    mat_to_quat_xyzw,
    quat_to_matrix_xyzw,
    to_aubo_frame,
)
from viewpoint_planner_msgs.msg import Viewpoint
from viewpoint_planner_msgs.srv import PlanViewpoints

# Fallback IK seed when no /joint_states message has arrived yet
# (mirrors grasp_preview_pipeline DEFAULT_ARM_JOINTS).
DEFAULT_Q_SEED = (
    -1.5707963267949,
    0.201570428261868,
    1.65970467002488,
    0.485178041391533,
    1.67675136677345,
    0.76432946885334,
)


def _flatten_limits(limits: tuple[tuple[float, float], ...]) -> list[float]:
    return [value for pair in limits for value in pair]


class ViewpointPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("viewpoint_planner")

        self.declare_parameter("viewpoint_plan_service", "/arachne/viewpoint/plan")
        self.declare_parameter("strategy", "fibonacci_sphere")
        self.declare_parameter("radius_m", 0.5)
        self.declare_parameter("num_viewpoints", 32)
        self.declare_parameter("filter_ik_reachable", True)
        self.declare_parameter("up_vector_base", [0.0, 0.0, 1.0])
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_limits", _flatten_limits(AUBO_I5_JOINT_LIMITS))
        self.declare_parameter("default_q_seed", list(DEFAULT_Q_SEED))
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("aubo_base_frame", "aubo_base_link")
        self.declare_parameter("tool0_frame", "tool0")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        # TF startup race guard: wait up to this long for the TF tree to fill
        # before failing a plan (service may be called right after auto-start).
        self.declare_parameter("tf_wait_timeout_sec", 3.0)
        # IK budget: unreachable targets are slow to reject (each pose runs the
        # full numerical-iteration budget), so cap total IK time and mark the
        # remaining poses unreachable to keep the service within its timeout.
        self.declare_parameter("ik_time_budget_sec", 12.0)
        # RViz visualization: publish a MarkerArray per plan (DELETEALL first).
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("markers_topic", "/arachne/viewpoint/markers")
        self.declare_parameter("markers_frame", "base_link")

        service_name = self.get_parameter("viewpoint_plan_service").value
        self._ik_filter = IkReachabilityFilter(
            joint_limits=self._resolve_joint_limits()
        )
        self._default_seed = np.asarray(
            self.get_parameter("default_q_seed").value, dtype=float
        )
        self._current_q: np.ndarray | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        joint_topic = self.get_parameter("joint_states_topic").value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._joint_sub = self.create_subscription(
            JointState, joint_topic, self._joint_state_cb, qos
        )

        self.create_service(PlanViewpoints, service_name, self._handle_plan)
        markers_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            # RViz2's MarkerArray display defaults to Transient Local; a
            # Volatile publisher is incompatible and invisible to it.
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("markers_topic").value), markers_qos
        )
        self.get_logger().info(f"viewpoint_planner ready on {service_name}")

    def _resolve_joint_limits(self) -> tuple[tuple[float, float], ...]:
        values = self.get_parameter("joint_limits").value
        if values is not None and len(values) == 12:
            return tuple((float(values[i]), float(values[i + 1])) for i in range(0, 12, 2))
        self.get_logger().warn(
            f"joint_limits parameter has {len(values) if values is not None else 0} "
            "values (expected 12); falling back to Aubo i5 nominal limits"
        )
        return AUBO_I5_JOINT_LIMITS

    def _joint_state_cb(self, msg: JointState) -> None:
        try:
            q = [
                msg.position[msg.name.index(name)] for name in REAL_ARM_JOINT_NAMES
            ]
        except ValueError:
            return  # joint_states lacks the arm joints (e.g. simulator-only tree)
        self._current_q = np.asarray(q, dtype=float)

    def _handle_plan(
        self, request: PlanViewpoints.Request, response: PlanViewpoints.Response
    ) -> PlanViewpoints.Response:
        try:
            response = self._plan_inner(request, response)
        except Exception as exc:  # service must always answer
            self.get_logger().error(f"viewpoint planning failed: {exc}")
            response.success = False
            response.message = f"viewpoint planning failed: {exc}"
            response.viewpoints = []
        self._publish_markers(request, response)
        return response

    def _publish_markers(
        self, request: PlanViewpoints.Request, response: PlanViewpoints.Response
    ) -> None:
        if not bool(self.get_parameter("publish_markers").value):
            return
        markers = build_viewpoint_marker_array(
            request, response, str(self.get_parameter("markers_frame").value)
        )
        self._marker_pub.publish(markers)

    def _plan_inner(
        self, request: PlanViewpoints.Request, response: PlanViewpoints.Response
    ) -> PlanViewpoints.Response:
        strategy_name = request.strategy or str(
            self.get_parameter("strategy").value
        )
        radius = (
            request.radius_m if request.radius_m > 0.0
            else float(self.get_parameter("radius_m").value)
        )
        count = (
            int(request.num_viewpoints) if request.num_viewpoints > 0
            else int(self.get_parameter("num_viewpoints").value)
        )
        filter_ik = bool(request.filter_ik_reachable) or bool(
            self.get_parameter("filter_ik_reachable").value
        )
        up = (
            request.up_vector_base
            if any(v != 0.0 for v in request.up_vector_base)
            else [float(v) for v in self.get_parameter("up_vector_base").value]
        )
        target = tuple(float(v) for v in request.target_xyz_base)

        try:
            strategy = get_strategy(strategy_name)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response

        samples = strategy.plan(
            target, radius, count, up_vector_base=tuple(up)
        )

        tool0_in_camera, base_in_aubo = self._lookup_transforms()
        if tool0_in_camera is None or base_in_aubo is None:
            response.success = False
            response.message = (
                "TF lookup failed (tool0<-camera or aubo_base_link<-base_link); "
                "check that URDF/static TF and camera extrinsics are published"
            )
            return response

        seed = self._current_q if self._current_q is not None else self._default_seed
        stamp = self.get_clock().now().to_msg()

        tool0_poses_base: list[np.ndarray] = []
        tool0_poses_aubo: list[np.ndarray] = []
        for sample in samples:
            camera_pose = np.eye(4)
            camera_pose[:3, 3] = np.asarray(sample.camera_xyz_base, dtype=float)
            camera_pose[:3, :3] = quat_to_matrix_xyzw(sample.camera_quat_xyzw)

            tool0_pose_base = compose_tool0_from_camera(camera_pose, tool0_in_camera)
            tool0_pose_aubo = to_aubo_frame(tool0_pose_base, base_in_aubo)
            tool0_poses_base.append(tool0_pose_base)
            tool0_poses_aubo.append(tool0_pose_aubo)

        # IK budget caps total solve time: unreachable targets are slow to
        # reject, so remaining poses are marked unreachable when exhausted.
        ik_results = self._ik_filter.filter_poses(
            tool0_poses_aubo,
            seed,
            time_budget_sec=float(self.get_parameter("ik_time_budget_sec").value),
        )

        viewpoint_msgs: list[Viewpoint] = []
        for sample, tool0_pose_base, result in zip(
            samples, tool0_poses_base, ik_results
        ):
            viewpoint_msgs.append(
                Viewpoint(
                    stamp=stamp,
                    camera_xyz_base=[float(v) for v in sample.camera_xyz_base],
                    camera_quat_xyzw=list(sample.camera_quat_xyzw),
                    tool0_xyz_base=[float(v) for v in tool0_pose_base[:3, 3]],
                    tool0_quat_xyzw=list(mat_to_quat_xyzw(tool0_pose_base[:3, :3])),
                    ik_reachable=result.ok,
                    ik_joints=[float(v) for v in result.q] if result.ok else [0.0] * 6,
                )
            )

        # 设计语义（§5「逐点标记可达性」）：保留全部采样点，ik_reachable 逐点标记；
        # filter_ik 仅控制是否执行 IK 校验，不删除采样点。
        response.success = True
        response.message = (
            f"planned {len(viewpoint_msgs)} viewpoints "
            f"(strategy={strategy_name}, radius={radius}, count={len(samples)}"
            f", filter_ik={filter_ik}); ik_reachable="
            f"{sum(1 for v in viewpoint_msgs if v.ik_reachable)}/{len(viewpoint_msgs)}"
        )
        response.viewpoints = viewpoint_msgs
        return response

    def _lookup_transform_wait(
        self, target_frame: str, source_frame: str, timeout: Duration
    ) -> Any:
        """Wait up to timeout for target<-source, polling lookup_transform.

        The Python tf2_ros bindings do not expose a synchronous
        wait_for_transform, so poll instead. The multi-threaded executor keeps
        processing TF messages while this service thread sleeps.
        """
        deadline = time.monotonic() + timeout.nanoseconds / 1e9
        while True:
            try:
                return self._tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )
            except TransformException:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.1)

    def _lookup_transforms(self) -> tuple[Any | None, Any | None]:
        """Resolve tool0<-camera and aubo_base_link<-base_link 4x4 transforms."""
        timeout = Duration(
            seconds=max(float(self.get_parameter("tf_wait_timeout_sec").value), 0.0)
        )
        try:
            tool0_msg = self._lookup_transform_wait(
                self.get_parameter("tool0_frame").value,
                self.get_parameter("camera_frame").value,
                timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(f"tool0<-camera lookup failed: {exc}")
            return None, None
        try:
            base_msg = self._lookup_transform_wait(
                self.get_parameter("aubo_base_frame").value,
                self.get_parameter("base_frame").value,
                timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(f"aubo_base_link<-base_link lookup failed: {exc}")
            return None, None
        tool0_in_camera = self._msg_to_transform(tool0_msg)
        base_in_aubo = self._msg_to_transform(base_msg)
        return tool0_in_camera, base_in_aubo

    @staticmethod
    def _msg_to_transform(transform_stamped: Any) -> np.ndarray:
        from viewpoint_planner.tf_utils import transform_from_parts

        translation = transform_stamped.transform.translation
        quat = transform_stamped.transform.rotation
        return transform_from_parts(
            (translation.x, translation.y, translation.z),
            (quat.x, quat.y, quat.z, quat.w),
        )


def _dry_run_check() -> None:
    """Offline validation: no ROS, no hardware, no TF."""
    strategy = get_strategy("fibonacci_sphere")
    samples = strategy.plan((0.3, 0.0, 0.2), 0.5, 32)
    assert len(samples) == 32, f"expected 32 samples, got {len(samples)}"
    for sample in samples:
        assert abs(float(np.linalg.norm(np.asarray(sample.camera_xyz_base) - np.asarray([0.3, 0.0, 0.2]))) - 0.5) < 1e-9

    ik_filter = IkReachabilityFilter()
    q_known = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    reachable = ik_filter.check(ik_filter.kinematics.fk(q_known), q_seed=q_known)
    assert reachable.ok, "IK self-pose should be reachable"

    print("viewpoint_planner dry-run check ok")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Arachne viewpoint planner")
    parser.add_argument(
        "--dry-run-check",
        action="store_true",
        help="validate imports and entrypoint wiring without starting ROS nodes or hardware I/O",
    )
    args, ros_args = parser.parse_known_args(argv)
    if args.dry_run_check:
        _dry_run_check()
        return

    rclpy.init(args=ros_args)
    node = ViewpointPlannerNode()
    # Multi-threaded so service callbacks can block on wait_for_transform
    # while TF/joint_states subscriptions keep updating the buffers.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
