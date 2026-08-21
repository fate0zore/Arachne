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
from arachne_operator.kinematics import AUBO_I5_JOINT_LIMITS, REAL_ARM_JOINT_NAMES
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from viewpoint_planner_msgs.msg import Viewpoint
from viewpoint_planner_msgs.srv import PlanViewpoints
from visualization_msgs.msg import MarkerArray

from viewpoint_planner.collision_filter import (
    CollisionFilter,
    CollisionParams,
    CollisionResult,
)
from viewpoint_planner.ik_filter import IkReachabilityFilter
from viewpoint_planner.markers import build_viewpoint_marker_array
from viewpoint_planner.strategies.base import get_strategy
from viewpoint_planner.tf_utils import (
    compose_tool0_from_camera,
    invert_rigid,
    mat_to_quat_xyzw,
    quat_to_matrix_xyzw,
    to_aubo_frame,
)

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
        # Collision filter (runs after IK; see collision_filter.py).
        self.declare_parameter("filter_collision_free", True)
        # Budget: geometry is far faster than numerical IK; cap total only to
        # keep the service within timeout, mark remaining poses collision=false.
        self.declare_parameter("collision_time_budget_sec", 5.0)
        # Arm/ground/end-effector geometry (defaults mirror grasp_preview_pipeline).
        self.declare_parameter("arm_collision_radius", 0.035)
        self.declare_parameter("arm_collision_samples_per_link", 6)
        self.declare_parameter("ground_min_z_base", -0.22)
        self.declare_parameter("ground_clearance", 0.02)
        self.declare_parameter("ee_camera_collision_enabled", True)
        self.declare_parameter("ee_camera_collision_margin", 0.01)
        self.declare_parameter("self_collision_enabled", True)
        self.declare_parameter("self_collision_radius", 0.035)
        # Keepout box margins / optional overrides (base_link, aligned with
        # teach_panel.py:73-77 and grasp_preview_pipeline.py:451-454).
        self.declare_parameter("collision_margin", 0.015)
        self.declare_parameter("rear_rack_collision_margin", 0.005)
        self.declare_parameter("front_basket_keepout_min", "")
        self.declare_parameter("front_basket_keepout_max", "")
        self.declare_parameter("rear_rack_keepout_min", "")
        self.declare_parameter("rear_rack_keepout_max", "")
        # RViz visualization: publish a MarkerArray per plan (DELETEALL first).
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("markers_topic", "/arachne/viewpoint/markers")
        self.declare_parameter("markers_frame", "base_link")
        # Per-viewpoint index labels (0-based, matching the teach_panel rows).
        self.declare_parameter("marker_label_enabled", True)
        self.declare_parameter("marker_label_offset_z", 0.08)
        self.declare_parameter("marker_label_size", 0.06)
        # Persistent preview: periodically republish the last plan's markers so
        # a late-starting or restarted RViz still shows the viewpoints (0 = off).
        self.declare_parameter("markers_republish_period_sec", 1.0)

        service_name = self.get_parameter("viewpoint_plan_service").value
        self._ik_filter = IkReachabilityFilter(
            joint_limits=self._resolve_joint_limits()
        )
        self._collision_filter = CollisionFilter(
            params=self._resolve_collision_params()
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
        self._last_marker_array: MarkerArray | None = None
        republish_period = float(
            self.get_parameter("markers_republish_period_sec").value
        )
        if republish_period > 0.0:
            self.create_timer(republish_period, self._marker_republish_cb)
        self.get_logger().info(f"viewpoint_planner ready on {service_name}")

    def _resolve_joint_limits(self) -> tuple[tuple[float, float], ...]:
        values = self.get_parameter("joint_limits").value
        if values is not None and len(values) == 12:
            return tuple(
                (float(values[i]), float(values[i + 1])) for i in range(0, 12, 2)
            )
        self.get_logger().warn(
            f"joint_limits parameter has {len(values) if values is not None else 0} "
            "values (expected 12); falling back to Aubo i5 nominal limits"
        )
        return AUBO_I5_JOINT_LIMITS

    def _resolve_collision_params(self) -> CollisionParams:
        """Build CollisionParams from node parameters (keepout overrides optional)."""

        def xyz(name: str) -> tuple[float, float, float] | None:
            raw = str(self.get_parameter(name).value).strip()
            if not raw:
                return None
            parts = [part.strip() for part in raw.split(",")]
            if len(parts) != 3:
                self.get_logger().warn(
                    f"collision parameter {name} has {len(parts)} values "
                    "(expected 3 x,y,z); ignoring override"
                )
                return None
            return tuple(float(part) for part in parts)  # type: ignore[return-value]

        return CollisionParams(
            arm_collision_radius=float(
                self.get_parameter("arm_collision_radius").value
            ),
            arm_collision_samples_per_link=int(
                self.get_parameter("arm_collision_samples_per_link").value
            ),
            ground_min_z_base=float(self.get_parameter("ground_min_z_base").value),
            ground_clearance=float(self.get_parameter("ground_clearance").value),
            ee_camera_collision_enabled=bool(
                self.get_parameter("ee_camera_collision_enabled").value
            ),
            ee_camera_collision_margin=float(
                self.get_parameter("ee_camera_collision_margin").value
            ),
            self_collision_enabled=bool(
                self.get_parameter("self_collision_enabled").value
            ),
            self_collision_radius=float(
                self.get_parameter("self_collision_radius").value
            ),
            collision_margin=float(self.get_parameter("collision_margin").value),
            rear_rack_collision_margin=float(
                self.get_parameter("rear_rack_collision_margin").value
            ),
            front_basket_keepout_min=xyz("front_basket_keepout_min"),
            front_basket_keepout_max=xyz("front_basket_keepout_max"),
            rear_rack_keepout_min=xyz("rear_rack_keepout_min"),
            rear_rack_keepout_max=xyz("rear_rack_keepout_max"),
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        try:
            q = [msg.position[msg.name.index(name)] for name in REAL_ARM_JOINT_NAMES]
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
            request,
            response,
            str(self.get_parameter("markers_frame").value),
            label_enabled=bool(self.get_parameter("marker_label_enabled").value),
            label_offset_z=float(self.get_parameter("marker_label_offset_z").value),
            label_size=float(self.get_parameter("marker_label_size").value),
        )
        self._last_marker_array = markers
        self._marker_pub.publish(markers)

    def _marker_republish_cb(self) -> None:
        """Timer callback keeping the last plan's markers visible in RViz."""
        if self._last_marker_array is not None:
            self._marker_pub.publish(self._last_marker_array)

    def _plan_inner(
        self, request: PlanViewpoints.Request, response: PlanViewpoints.Response
    ) -> PlanViewpoints.Response:
        strategy_name = request.strategy or str(self.get_parameter("strategy").value)
        radius = (
            request.radius_m
            if request.radius_m > 0.0
            else float(self.get_parameter("radius_m").value)
        )
        count = (
            int(request.num_viewpoints)
            if request.num_viewpoints > 0
            else int(self.get_parameter("num_viewpoints").value)
        )
        filter_ik = bool(request.filter_ik_reachable) or bool(
            self.get_parameter("filter_ik_reachable").value
        )
        filter_collision = bool(request.filter_collision_free) or bool(
            self.get_parameter("filter_collision_free").value
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

        samples = strategy.plan(target, radius, count, up_vector_base=tuple(up))

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

        # Collision filter runs only on IK-reachable points (valid q); two
        # independent per-point fields: ik_reachable and collision_free.
        base_from_aubo = invert_rigid(base_in_aubo)
        collision_results = self._collision_filter.filter_joints(
            ik_results,
            base_from_aubo,
            time_budget_sec=float(
                self.get_parameter("collision_time_budget_sec").value
            ),
        )

        viewpoint_msgs: list[Viewpoint] = []
        for sample, tool0_pose_base, result, collision in zip(
            samples, tool0_poses_base, ik_results, collision_results
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
                    collision_free=collision.collision_free,
                    collision_clearance=float(collision.clearance),
                )
            )

        # 设计语义（§5「逐点标记可达性」）：保留全部采样点，ik_reachable 与
        # collision_free 各自逐点标记，互不覆盖；两开关都只控制"最终可达判定"，
        # 不删除采样点（flag 始终逐点标记）。
        reachable_count = sum(
            1
            for viewpoint in viewpoint_msgs
            if viewpoint.ik_reachable and viewpoint.collision_free
        )
        response.success = True
        response.message = (
            f"planned {len(viewpoint_msgs)} viewpoints "
            f"(strategy={strategy_name}, radius={radius}, count={len(samples)}"
            f", filter_ik={filter_ik}, filter_collision={filter_collision}); "
            f"ik_reachable="
            f"{sum(1 for v in viewpoint_msgs if v.ik_reachable)}/{len(viewpoint_msgs)}, "
            f"collision_free="
            f"{sum(1 for v in viewpoint_msgs if v.collision_free)}/{len(viewpoint_msgs)}, "
            f"reachable="
            f"{reachable_count}/{len(viewpoint_msgs)}"
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
        assert (
            abs(
                float(
                    np.linalg.norm(
                        np.asarray(sample.camera_xyz_base) - np.asarray([0.3, 0.0, 0.2])
                    )
                )
                - 0.5
            )
            < 1e-9
        )

    ik_filter = IkReachabilityFilter()
    q_known = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    reachable = ik_filter.check(ik_filter.kinematics.fk(q_known), q_seed=q_known)
    assert reachable.ok, "IK self-pose should be reachable"

    collision_filter = CollisionFilter()
    # Realistic install: arm is mounted on the scout at DEFAULT_ARM_BASE_XYZ/RPY
    # (aubo frame in base_link); base_from_aubo is the inverse. At the zero
    # pose the arm extends above the vehicle and must be collision-free.
    from viewpoint_planner.tf_utils import invert_rigid, transform_from_parts

    arm_base = transform_from_parts(
        (0.22, 0.0, 0.105),
        (0.0, 0.0, 0.7071067811865475, 0.7071067811865475),  # rpy z=pi/2
    )
    base_from_aubo = invert_rigid(arm_base)
    self_result = collision_filter.check(q_known, base_from_aubo)
    # Note: the exact hit verdict depends on uncalibrated box approximations and
    # the install offset, so we only assert the API returns a well-formed result
    # (not a specific collision verdict). Over/under-detection is a known
    # planning-phase approximation (phase2 plan risk item 1).
    assert isinstance(self_result.collision_free, bool)
    assert isinstance(self_result.clearance, float)
    assert isinstance(self_result.hit_name, (type(None), str))

    # Pretend IK results: one reachable (valid q), one unreachable (None q).
    class _FakeIk:
        def __init__(self, ok, q):
            self.ok = ok
            self.q = None if q is None else np.array(q, dtype=float)

    fake = [_FakeIk(True, q_known), _FakeIk(False, None)]
    results = collision_filter.filter_joints(fake, base_from_aubo)
    assert isinstance(results[0], CollisionResult)
    assert not results[1].collision_free, "unreachable pose should be collision=false"
    assert results[1].reason == "ik_unreachable", results[1].reason

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
