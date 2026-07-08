from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from arachne_operator.control_owner import (
    DEFAULT_AUBO_CONTROL_OWNER_PATH,
    control_owner_available,
)
from arachne_operator.joint_utils import AUBO_JOINT_ALIASES
from arachne_operator.repo_paths import root_dir

try:
    from arachne_hardware.aubo_tcp_driver import AuboDirectJsonRpc
except ModuleNotFoundError:
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "arachne_hardware"))
    from arachne_hardware.aubo_tcp_driver import AuboDirectJsonRpc


TERMINAL_STATES = {"succeeded", "failed", "canceled"}
DEFAULT_AUBO_TEACH_FLAG_PATH = "/tmp/arachne_aubo_teach_mode"
DEFAULT_REAL_FIXED_LIFT_JOINTS = "-1.392228627,-0.587456810,1.402798238,0.420158124,1.570706911,0.178573568"
DEFAULT_REAL_FIXED_BASKET_JOINTS = "-1.0532545919568441,0.13642934952944055,2.6318805595335437,1.0042082880690795,1.5823432026398125,1.308814683852632"
DEFAULT_RELEASE_POINT_B_JOINTS = "-1.407594293659515,0.07478614140534845,2.611345219296526,1.0028015483718502,1.6581842928233566,0.8873273897703009"
RELEASE_POINTS = {"basket_a", "basket_b"}


def _angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _yaw_from_odom(msg: Odometry) -> float:
    orientation = msg.pose.pose.orientation
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class PreflightResult:
    ok: bool
    checks: list[dict[str, Any]] = field(default_factory=list)

    def add(self, name: str, ok: bool, message: str, *, required: bool = True) -> None:
        self.checks.append(
            {
                "name": name,
                "ok": bool(ok),
                "required": bool(required),
                "message": message,
            }
        )
        if required and not ok:
            self.ok = False


@dataclass
class TaskSnapshot:
    task_id: str
    state: str
    message: str
    run_dir: str
    started_at: str
    finished_at: str
    returncode: int | None
    grasp_preview_log_dir: str
    preflight: list[dict[str, Any]]
    base: dict[str, Any]


class GraspTaskServer(Node):
    """Guarded wrapper around the existing grasp_preview real execution demo."""

    def __init__(self) -> None:
        super().__init__("arachne_grasp_task_server")

        self.declare_parameter("workspace_root", "")
        self.declare_parameter("runner_script", "")
        self.declare_parameter("log_root", "log/grasp_tasks")
        self.declare_parameter("execute_real", False)
        self.declare_parameter("confirm_execute_real", False)
        self.declare_parameter("with_rviz", False)
        self.declare_parameter("classes", "")
        self.declare_parameter("confidence", 0.08)
        self.declare_parameter("device_id", 0)
        self.declare_parameter("real_execute_backend", "sdk_move_joint")
        self.declare_parameter("real_return_home", True)
        self.declare_parameter("real_fixed_post_grasp", True)
        self.declare_parameter(
            "real_fixed_search_joints", "-1.629044,0.031622,1.684745,0.079056,1.575197,0.754000"
        )
        self.declare_parameter("real_fixed_lift_joints", DEFAULT_REAL_FIXED_LIFT_JOINTS)
        self.declare_parameter("real_fixed_basket_joints", DEFAULT_REAL_FIXED_BASKET_JOINTS)
        self.declare_parameter("release_point", "basket_a")
        self.declare_parameter("release_point_a_joints", DEFAULT_REAL_FIXED_BASKET_JOINTS)
        self.declare_parameter("release_point_b_joints", DEFAULT_RELEASE_POINT_B_JOINTS)
        self.declare_parameter("real_sdk_move_speed", 0.18)
        self.declare_parameter("real_sdk_move_accel", 0.25)
        self.declare_parameter("real_gripper_settle_sec", 2.0)
        self.declare_parameter("real_preopen_gripper", False)
        self.declare_parameter("real_sdk_ip", os.environ.get("AUBO_ROBOT_IP", "192.168.127.128"))
        self.declare_parameter("real_sdk_rpc_port", 30004)
        self.declare_parameter("real_sdk_rpc_timeout", 2.0)
        self.declare_parameter("aubo_teach_flag_path", DEFAULT_AUBO_TEACH_FLAG_PATH)
        self.declare_parameter("aubo_control_owner_path", DEFAULT_AUBO_CONTROL_OWNER_PATH)
        self.declare_parameter("aubo_control_owner_name", "grasp_task_server")
        self.declare_parameter("aubo_move_joint_action_name", "/arachne/aubo/move_joint")
        self.declare_parameter("prefer_aubo_move_joint_action", False)
        self.declare_parameter("aubo_move_joint_fallback_internal", False)
        self.declare_parameter("aubo_move_joint_wait_server_sec", 0.5)
        self.declare_parameter("grasp_base_offset", "-0.23,0.01,0.16")
        self.declare_parameter(
            "extra_args",
            "--planner-backend local --planning-key-waypoints grasp --vertical-approach --no-lock-grasp-orientation --tool-orientation-limit-deg 45 --grasp-orientation-yaw-offsets-deg 0 --grasp-orientation-tilt-offsets-deg 0,8,-8 --local-position-tolerance 0.010 --local-orientation-tolerance 0.50 --local-planning-timeout-sec 2.0 --local-ik-max-iterations 90 --real-gripper-require-capture --real-sdk-semantic-targets-only --real-sdk-max-targets 6",
        )
        self.declare_parameter("preview_on_start", False)
        self.declare_parameter("preview_runner_script", "")
        self.declare_parameter("preview_extra_args", "--planner-backend none --imgsz 640")
        self.declare_parameter("warm_execute_preview", False)
        self.declare_parameter("real_execute_trigger_topic", "/arachne/grasp_preview/execute_real")
        self.declare_parameter("real_execute_trigger_service", "/arachne/grasp_preview/execute_real_now")
        self.declare_parameter("restart_search_topic", "/arachne/grasp_preview/restart_search")
        self.declare_parameter("warm_execute_timeout_sec", 25.0)
        self.declare_parameter("warm_execute_trigger_wait_sec", 20.0)
        self.declare_parameter("planning_recovery_base_enabled", True)
        self.declare_parameter(
            "planning_recovery_base_sequence",
            "forward:0.10,forward:0.10,forward:0.10",
        )
        self.declare_parameter("planning_recovery_restore_on_failure", True)
        self.declare_parameter("preflight_timeout_sec", 2.0)
        self.declare_parameter("skip_preflight", False)
        self.declare_parameter("status_publish_period_sec", 0.5)
        self.declare_parameter("require_safety_state_machine", False)
        self.declare_parameter("set_safety_autonomous_on_start", True)
        self.declare_parameter("set_safety_manual_on_finish", True)
        self.declare_parameter("require_aubo_status", False)
        self.declare_parameter("require_joint_states", True)
        self.declare_parameter("require_gripper_status", True)
        self.declare_parameter("require_odom", False)
        self.declare_parameter("require_camera_topics", False)
        self.declare_parameter("max_grasp_attempts", 3)
        self.declare_parameter("retry_on_gripper_miss", True)
        self.declare_parameter("gripper_miss_retry_delay_sec", 0.8)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "teach_visualization_joint_states_topic",
            "/arachne/teach_visualization/joint_states",
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("aubo_status_topic", "/arachne/hardware/aubo_status")
        self.declare_parameter("gripper_status_topic", "/arachne/hardware/gripper_status")
        self.declare_parameter("base_status_topic", "/arachne/hardware/base_status")
        self.declare_parameter("safety_state_topic", "/arachne/safety/state")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("base_command_topic", "/arachne/grasp_task/base_command")
        self.declare_parameter("base_state_topic", "/arachne/grasp_task/base_state")
        self.declare_parameter("base_linear_speed", 0.16)
        self.declare_parameter("base_angular_speed", 0.60)
        self.declare_parameter("base_replay_linear_speed", 0.40)
        self.declare_parameter("base_replay_angular_speed", 0.36)
        self.declare_parameter("base_position_tolerance", 0.02)
        self.declare_parameter("base_yaw_tolerance_deg", 0.5)
        self.declare_parameter("base_manual_publish_rate", 12.0)
        self.declare_parameter("base_motion_max_segment_sec", 20.0)
        self.declare_parameter("base_pose_timeout_sec", 3.0)
        self.declare_parameter("allow_base_commands_during_grasp", False)

        self.root = self._resolve_workspace_root()
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.base_cancel_event = threading.Event()
        self.process: subprocess.Popen[str] | None = None
        self.idle_preview_process: subprocess.Popen[str] | None = None
        self.idle_preview_log_file = None
        self.idle_preview_reader: threading.Thread | None = None
        self.idle_preview_last_start = 0.0
        self.idle_preview_config_key = ""
        self.worker: threading.Thread | None = None
        self.base_worker: threading.Thread | None = None
        self.runner_success_requested = False
        self.runner_planning_failed = False
        self.runner_real_execution_started = False
        self.runner_real_execution_blocked = False
        self.runner_gripper_capture_failed = False
        self.runner_waiting_for_execute_trigger = False
        self.latest: dict[str, tuple[float, Any]] = {}
        self.state = "idle"
        self.message = "ready"
        self.task_id = ""
        self.run_dir: Path | None = None
        self.started_at = ""
        self.finished_at = ""
        self.returncode: int | None = None
        self.preflight_checks: list[dict[str, Any]] = []
        self.grasp_preview_log_dir = ""
        self.manual_base_velocity: tuple[float, float] | None = None
        self.active_base_motion: dict[str, Any] | None = None
        self.base_motion_segments: list[dict[str, Any]] = []
        self.base_state = "idle"
        self.base_message = "ready"
        self.base_started_at = ""
        self.base_finished_at = ""
        self.base_active_command: dict[str, Any] = {}
        self.base_latest_result: dict[str, Any] = {}
        self.last_planning_recovery_steps: list[dict[str, Any]] = []
        self.restore_worker: threading.Thread | None = None

        self.state_pub = self.create_publisher(String, "/arachne/grasp_task/state", 10)
        self.event_pub = self.create_publisher(String, "/arachne/grasp_task/event", 10)
        self.restart_search_pub = self.create_publisher(
            Empty, str(self.get_parameter("restart_search_topic").value), 10
        )
        self.real_execute_trigger_pub = self.create_publisher(
            Empty, str(self.get_parameter("real_execute_trigger_topic").value), 10
        )
        self.base_state_pub = self.create_publisher(
            String, str(self.get_parameter("base_state_topic").value), 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_service(Trigger, "/arachne/grasp_task/start", self._start_cb)
        self.create_service(Trigger, "/arachne/grasp_task/cancel", self._cancel_cb)
        self.create_service(Trigger, "/arachne/grasp_task/stop", self._cancel_cb)
        self.create_service(Trigger, "/arachne/grasp_task/restore", self._restore_cb)
        self.create_service(Trigger, "/arachne/grasp_task/status", self._status_cb)
        self.create_service(Trigger, "/arachne/grasp_task/preflight", self._preflight_cb)
        self.create_service(Trigger, "/arachne/grasp_task/base_stop", self._base_stop_cb)
        self.create_service(Trigger, "/arachne/grasp_task/base_status", self._base_status_cb)

        self.set_autonomous_client = self.create_client(
            Trigger, "/arachne/safety/set_autonomous"
        )
        self.set_manual_client = self.create_client(Trigger, "/arachne/safety/set_manual")
        self.real_execute_trigger_client = self.create_client(
            Trigger, str(self.get_parameter("real_execute_trigger_service").value)
        )

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._cache_joint_states,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("teach_visualization_joint_states_topic").value),
            self._cache_joint_states,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            lambda msg: self._cache("odom", msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("aubo_status_topic").value),
            lambda msg: self._cache("aubo_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_status_topic").value),
            lambda msg: self._cache("gripper_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("base_status_topic").value),
            lambda msg: self._cache("base_status", msg.data),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("safety_state_topic").value),
            lambda msg: self._cache("safety_state", msg.data),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            lambda msg: self._cache("color_image", msg.header.stamp),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            lambda msg: self._cache("depth_image", msg.header.stamp),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("base_command_topic").value),
            self._base_command_cb,
            10,
        )

        period = max(float(self.get_parameter("status_publish_period_sec").value), 0.1)
        self.create_timer(period, self._publish_state)
        self.create_timer(period, self._publish_base_state)
        self.create_timer(1.0, self._idle_preview_tick)
        base_rate = max(float(self.get_parameter("base_manual_publish_rate").value), 1.0)
        self.create_timer(1.0 / base_rate, self._publish_manual_base_velocity)
        self._event("server_ready", {"workspace_root": str(self.root)})

    def _resolve_workspace_root(self) -> Path:
        configured = str(self.get_parameter("workspace_root").value).strip()
        if configured:
            return Path(configured).expanduser().resolve()
        env_root = os.environ.get("ARACHNE_ROOT_DIR", "").strip()
        if env_root:
            return Path(env_root).expanduser().resolve()
        return root_dir()

    def _cache(self, key: str, value: Any) -> None:
        with self.lock:
            self.latest[key] = (time.monotonic(), value)

    def _cache_joint_states(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        found: dict[str, float] = {}
        for canonical, aliases in AUBO_JOINT_ALIASES:
            for alias in aliases:
                if alias in positions:
                    found[canonical] = float(positions[alias])
                    break
        with self.lock:
            self.latest["joint_states"] = (time.monotonic(), msg)
            if len(found) == len(AUBO_JOINT_ALIASES):
                self.latest["aubo_joints"] = (time.monotonic(), found)

    def _start_cb(self, _request, response):
        with self.lock:
            busy = self.worker is not None and self.worker.is_alive()
            if busy or self.state not in ("idle", *TERMINAL_STATES):
                response.success = False
                response.message = self._snapshot_json()
                return response
            self.cancel_event.clear()
            self.worker = threading.Thread(target=self._run_task, daemon=True)
            self.worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _cancel_cb(self, _request, response):
        self.cancel_event.set()
        self._terminate_process("cancel requested")
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _restore_cb(self, _request, response):
        with self.lock:
            busy = self.restore_worker is not None and self.restore_worker.is_alive()
            if busy:
                response.success = False
                response.message = self._snapshot_json()
                return response
            self.restore_worker = threading.Thread(target=self._restore_worker, daemon=True)
            self.restore_worker.start()
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _status_cb(self, _request, response):
        response.success = True
        response.message = self._snapshot_json()
        return response

    def _preflight_cb(self, _request, response):
        result = self._run_preflight(set_autonomous=False)
        with self.lock:
            self.preflight_checks = result.checks
        response.success = result.ok
        response.message = json.dumps(
            {"ok": result.ok, "checks": result.checks},
            ensure_ascii=False,
            sort_keys=True,
        )
        return response

    def _idle_preview_tick(self) -> None:
        if not bool(self.get_parameter("preview_on_start").value):
            self._stop_idle_preview("preview disabled")
            return
        if self._grasp_task_active():
            if not bool(self.get_parameter("warm_execute_preview").value):
                self._stop_idle_preview("grasp task active")
            return
        with self.lock:
            process = self.idle_preview_process
            last_start = self.idle_preview_last_start
            config_key = self.idle_preview_config_key
        if process is not None and process.poll() is None:
            if config_key != self._idle_preview_config_key():
                self._stop_idle_preview("preview config changed")
                with self.lock:
                    self.idle_preview_last_start = 0.0
            else:
                return
        with self.lock:
            last_start = self.idle_preview_last_start
        if time.monotonic() - last_start < 10.0:
            return
        self._start_idle_preview()

    def _idle_preview_config_key(self) -> str:
        return "|".join(
            (
                str(self.get_parameter("real_gripper_settle_sec").value),
                str(bool(self.get_parameter("real_preopen_gripper").value)),
            )
        )

    def _start_idle_preview(self) -> None:
        with self.lock:
            existing = self.idle_preview_process
        if existing is not None and existing.poll() is None:
            return
        self._clear_orphan_aubo_teach_gate()
        try:
            command, env = self._idle_preview_command()
        except ValueError as exc:
            self._event("idle_preview_error", {"error": str(exc)})
            return
        log_dir = self._log_root() / "idle_preview"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / (datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
        try:
            process = subprocess.Popen(
                command,
                cwd=str(self.root),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except Exception as exc:
            self._event("idle_preview_error", {"error": str(exc), "command": command})
            return
        with self.lock:
            self.idle_preview_process = process
            self.idle_preview_last_start = time.monotonic()
            self.idle_preview_config_key = self._idle_preview_config_key()
            self.idle_preview_reader = threading.Thread(
                target=self._idle_preview_reader_loop,
                args=(process, log_path),
                daemon=True,
            )
            self.idle_preview_reader.start()
        self._event("idle_preview_started", {"pid": process.pid, "log": str(log_path)})

    def _idle_preview_reader_loop(self, process: subprocess.Popen[str], log_path: Path) -> None:
        try:
            with log_path.open("w", encoding="utf-8") as log_file:
                if process.stdout is None:
                    return
                for line in process.stdout:
                    log_file.write(line)
                    log_file.flush()
                    self._parse_process_line(line)
        except Exception as exc:
            self._event("idle_preview_reader_error", {"error": str(exc), "log": str(log_path)})

    def _idle_preview_command(self) -> tuple[list[str], dict[str, str]]:
        configured_runner = str(self.get_parameter("preview_runner_script").value).strip()
        use_warm_execute = bool(self.get_parameter("warm_execute_preview").value) and bool(
            self.get_parameter("execute_real").value
        )
        if use_warm_execute:
            with self.lock:
                has_cached_joints = self.latest.get("aubo_joints") is not None
            if not has_cached_joints:
                self._refresh_aubo_joints_from_rpc()
        extra_source = "extra_args" if use_warm_execute else "preview_extra_args"
        extra = shlex.split(str(self.get_parameter(extra_source).value))
        if use_warm_execute:
            extra.extend(
                [
                    "--real-execute-trigger-topic",
                    str(self.get_parameter("real_execute_trigger_topic").value),
                    "--real-execute-trigger-service",
                    str(self.get_parameter("real_execute_trigger_service").value),
                ]
            )
        env = os.environ.copy()
        env["ARACHNE_GRASP_START_MOVEIT"] = "false"
        env["ARACHNE_GRASP_START_MODEL"] = "false"
        env["ARACHNE_GRASP_START_CAMERA"] = "false"
        env["ARACHNE_GRASP_WITH_RVIZ"] = "false"
        env["ARACHNE_GRASP_DISPLAY_FRAME_PREFIX"] = ""
        env["ARACHNE_GRASP_CAMERA_PARENT_FRAME"] = "ee_camera_link"
        env["ARACHNE_GRASP_EXECUTE_REAL"] = "true" if use_warm_execute else "false"
        if use_warm_execute and bool(self.get_parameter("confirm_execute_real").value):
            env["ARACHNE_CONFIRM_GRASP_EXECUTE_REAL"] = "YES"
        env["ARACHNE_GRASP_REAL_EXECUTE_BACKEND"] = str(
            self.get_parameter("real_execute_backend").value
        )
        env["ARACHNE_GRASP_REAL_SEARCH_SCAN"] = "false"
        env["ARACHNE_GRASP_REAL_SDK_IP"] = str(self.get_parameter("real_sdk_ip").value)
        env["ARACHNE_GRASP_REAL_RETURN_HOME"] = (
            "true" if bool(self.get_parameter("real_return_home").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_FIXED_POST_GRASP"] = (
            "true" if bool(self.get_parameter("real_fixed_post_grasp").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_FIXED_SEARCH_JOINTS"] = str(
            self.get_parameter("real_fixed_search_joints").value
        )
        env["ARACHNE_GRASP_REAL_FIXED_LIFT_JOINTS"] = str(
            self.get_parameter("real_fixed_lift_joints").value
        )
        env["ARACHNE_GRASP_REAL_FIXED_BASKET_JOINTS"] = self._resolved_release_basket_joints()
        with self.lock:
            latest_joints = self.latest.get("aubo_joints")
        if latest_joints is not None:
            joint_map = latest_joints[1]
            names = [canonical for canonical, _aliases in AUBO_JOINT_ALIASES]
            if isinstance(joint_map, dict) and all(name in joint_map for name in names):
                env["ARACHNE_GRASP_ARM_JOINTS"] = ",".join(
                    f"{float(joint_map[name]):.12g}" for name in names
                )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_SPEED"] = str(
            self.get_parameter("real_sdk_move_speed").value
        )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_ACCEL"] = str(
            self.get_parameter("real_sdk_move_accel").value
        )
        env["ARACHNE_GRASP_REAL_GRIPPER_SETTLE_SEC"] = str(
            self.get_parameter("real_gripper_settle_sec").value
        )
        env["ARACHNE_GRASP_REAL_PREOPEN_GRIPPER"] = (
            "true" if bool(self.get_parameter("real_preopen_gripper").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_SDK_TEACH_FLAG_PATH"] = str(
            self.get_parameter("aubo_teach_flag_path").value
        )
        env["ARACHNE_GRASP_AUBO_CONTROL_OWNER_PATH"] = str(
            self.get_parameter("aubo_control_owner_path").value
        )
        env["ARACHNE_GRASP_AUBO_CONTROL_OWNER_NAME"] = str(
            self.get_parameter("aubo_control_owner_name").value
        )
        env["ARACHNE_GRASP_AUBO_MOVE_JOINT_ACTION_NAME"] = str(
            self.get_parameter("aubo_move_joint_action_name").value
        )
        env["ARACHNE_GRASP_PREFER_AUBO_MOVE_JOINT_ACTION"] = (
            "true" if bool(self.get_parameter("prefer_aubo_move_joint_action").value) else "false"
        )
        env["ARACHNE_GRASP_AUBO_MOVE_JOINT_FALLBACK_INTERNAL"] = (
            "true"
            if bool(self.get_parameter("aubo_move_joint_fallback_internal").value)
            else "false"
        )
        env["ARACHNE_GRASP_AUBO_MOVE_JOINT_WAIT_SERVER_SEC"] = str(
            self.get_parameter("aubo_move_joint_wait_server_sec").value
        )
        env["ARACHNE_GRASP_CLASSES"] = str(self.get_parameter("classes").value)
        env["ARACHNE_GRASP_CONF"] = str(self.get_parameter("confidence").value)
        env["ARACHNE_GRASP_DEVICE_ID"] = str(self.get_parameter("device_id").value)
        env["ARACHNE_GRASP_BASE_OFFSET"] = str(self.get_parameter("grasp_base_offset").value)
        command = self._configured_or_pipeline_command(
            configured_runner,
            extra,
            env,
            execute_real=use_warm_execute,
        )
        return command, env

    def _stop_idle_preview(self, reason: str) -> None:
        with self.lock:
            process = self.idle_preview_process
            log_file = self.idle_preview_log_file
            self.idle_preview_process = None
            self.idle_preview_log_file = None
            self.idle_preview_config_key = ""
        if process is not None and process.poll() is None:
            self._event("idle_preview_stop", {"reason": reason, "pid": process.pid})
            self._terminate_process_group(process, reason, timeout=2.0)
        if log_file is not None:
            try:
                log_file.close()
            except Exception:
                pass

    def _wait_for_aubo_control_release(self, timeout: float) -> tuple[bool, str]:
        deadline = time.monotonic() + max(float(timeout), 0.0)
        last = ""
        while time.monotonic() <= deadline:
            owner_ok, owner_message = self._aubo_control_owner_available()
            gate_ok, gate_message = self._aubo_teach_gate_available()
            last = f"{owner_message}; {gate_message}"
            if owner_ok and gate_ok:
                return True, last
            if owner_ok and self._clear_orphan_aubo_teach_gate():
                return True, f"{owner_message}; cleared orphan teach gate"
            time.sleep(0.05)
        return False, last

    def _clear_orphan_aubo_teach_gate(self) -> bool:
        gate = Path(str(self.get_parameter("aubo_teach_flag_path").value))
        owner = Path(str(self.get_parameter("aubo_control_owner_path").value))
        if not gate.exists() or owner.exists():
            return False
        try:
            text = gate.read_text(encoding="utf-8", errors="replace").strip()
        except OSError:
            return False
        if not text.startswith("1"):
            return False
        try:
            gate.unlink(missing_ok=True)
        except OSError:
            return False
        self._event("aubo_teach_gate_orphan_cleared", {"path": str(gate), "value": text})
        return True

    def _base_stop_cb(self, _request, response):
        ok, message = self._stop_base("service stop")
        response.success = ok
        response.message = message
        return response

    def _base_status_cb(self, _request, response):
        response.success = True
        response.message = self._base_snapshot_json()
        return response

    def _base_command_cb(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        try:
            payload: Any = json.loads(text)
        except json.JSONDecodeError:
            payload = text
        try:
            ok, message = self._handle_base_command(payload)
        except Exception as exc:
            ok = False
            message = f"base command rejected: {exc}"
            self._set_base_state("failed", message)
        self._event("base_command", {"ok": ok, "message": message, "payload": payload})

    def request_base_motion(self, payload: dict[str, Any]) -> tuple[bool, str]:
        """Internal entry for future autonomous task policies.

        The ROS String topic uses the same parser for smoke tests and external
        bridges, but the intended production path is for task logic in this node
        to call this method after detection/planning decides a base motion is
        needed.
        """
        return self._handle_base_command(payload)

    def _handle_base_command(self, payload: Any) -> tuple[bool, str]:
        if isinstance(payload, str):
            payload = {"command": payload}
        if not isinstance(payload, dict):
            raise ValueError("base command must be a string or JSON object")

        raw_command = str(payload.get("command", payload.get("type", ""))).strip().lower()
        command = raw_command
        if not command:
            raise ValueError("missing base command")
        if raw_command == "turn_left" and "direction" not in payload:
            payload = dict(payload, direction="left")
        elif raw_command == "turn_right" and "direction" not in payload:
            payload = dict(payload, direction="right")
        command = {
            "backward": "back",
            "reverse": "back",
            "turn_left": "turn_relative",
            "turn_right": "turn_relative",
            "drive": "drive_relative",
            "move": "drive_relative",
            "velocity": "timed",
            "cmd_vel": "timed",
            "segments": "replay_segments",
        }.get(command, command)

        if command == "stop":
            return self._stop_base("command stop")

        if self._grasp_task_active() and not bool(
            self.get_parameter("allow_base_commands_during_grasp").value
        ):
            return False, "base command rejected: grasp task is active"

        if command in ("forward", "back", "left", "right", "manual", "jog"):
            direction = str(payload.get("direction", command)).strip().lower()
            if direction in ("backward", "reverse"):
                direction = "back"
            self._drive_base_manual(direction)
            return True, self._base_message_snapshot()

        if command == "drive_relative":
            distance = self._payload_float(payload, "distance_m", "distance", default=0.0)
            speed = self._payload_float(
                payload,
                "speed_m_s",
                "speed_mps",
                "linear_speed_mps",
                "linear_x",
                default=0.0,
            )
            return self._start_base_worker(
                "drive_relative",
                payload,
                lambda: self._drive_distance(distance, speed_override=abs(speed) if speed > 0.0 else None),
            )

        if command == "turn_relative":
            if "angle_rad" in payload:
                angle = float(payload["angle_rad"])
            elif "angle_deg" in payload:
                angle = math.radians(float(payload["angle_deg"]))
            else:
                sign = -1.0 if str(payload.get("direction", "")).lower() == "right" else 1.0
                angle = sign * math.radians(float(payload.get("angle", 0.0)))
            return self._start_base_worker(
                "turn_relative",
                payload,
                lambda: self._turn_relative(angle),
            )

        if command == "timed":
            linear_x = self._payload_float(payload, "linear_x", "vx", "linear", default=0.0)
            angular_z = self._payload_float(payload, "angular_z", "wz", "yaw_rate", default=0.0)
            duration = self._payload_float(payload, "duration_sec", "duration", default=0.0)
            return self._start_base_worker(
                "timed",
                payload,
                lambda: self._run_timed_base_motion(linear_x, angular_z, duration),
            )

        if command in ("replay_segments", "replay"):
            segments = payload.get("segments", payload.get("base_motion", []))
            if not isinstance(segments, list):
                raise ValueError("segments must be a list")
            return self._start_base_worker(
                "replay_segments",
                payload,
                lambda: self._replay_base_motion(segments, label=str(payload.get("label", "base"))),
            )

        if command == "segment":
            segment = payload.get("segment", payload)
            if not isinstance(segment, dict):
                raise ValueError("segment must be an object")
            return self._start_base_worker(
                "segment",
                payload,
                lambda: self._replay_base_motion([segment], label=str(payload.get("label", "base"))),
            )

        raise ValueError(f"unsupported base command: {command}")

    def _grasp_task_active(self) -> bool:
        with self.lock:
            worker_active = self.worker is not None and self.worker.is_alive()
            return worker_active or self.state in ("preflight", "running")

    def _payload_float(self, payload: dict[str, Any], *names: str, default: float) -> float:
        for name in names:
            if name in payload:
                return float(payload[name])
        return float(default)

    def _start_base_worker(self, label: str, payload: dict[str, Any], target) -> tuple[bool, str]:
        with self.lock:
            busy = self.base_worker is not None and self.base_worker.is_alive()
            if busy:
                return False, self._base_message_snapshot()
            self.base_cancel_event.clear()
            self.manual_base_velocity = None
            self._close_base_motion_locked(time.monotonic())
            self.base_state = "running"
            self.base_message = f"base {label} started"
            self.base_started_at = datetime.now().isoformat(timespec="seconds")
            self.base_finished_at = ""
            self.base_active_command = dict(payload)
            self.base_latest_result = {}
            self.base_worker = threading.Thread(
                target=self._base_worker_runner,
                args=(label, target),
                daemon=True,
            )
            self.base_worker.start()
        self._event("base_state", {"state": "running", "message": f"base {label} started"})
        self._publish_base_state()
        return True, f"base {label} started"

    def _base_worker_runner(self, label: str, target) -> None:
        try:
            target()
            if self.base_cancel_event.is_set():
                self._set_base_state("canceled", f"base {label} canceled")
            else:
                self._set_base_state("succeeded", f"base {label} complete")
        except Exception as exc:
            if self.base_cancel_event.is_set():
                self._set_base_state("canceled", f"base {label} canceled")
            else:
                self._set_base_state("failed", f"base {label} failed: {exc}")
        finally:
            self._publish_base_stop()

    def _drive_base_manual(self, direction: str) -> None:
        linear = float(self.get_parameter("base_linear_speed").value)
        angular = float(self.get_parameter("base_angular_speed").value)
        mapping = {
            "forward": (linear, 0.0),
            "back": (-linear, 0.0),
            "left": (0.0, angular),
            "right": (0.0, -angular),
            "stop": (0.0, 0.0),
        }
        if direction not in mapping:
            raise ValueError(f"unsupported manual base direction: {direction}")
        vx, wz = mapping[direction]
        self._track_base_motion(direction, vx, wz)
        with self.lock:
            self.manual_base_velocity = None if direction == "stop" else (vx, wz)
            if direction == "stop":
                self.base_state = "idle"
                self.base_message = "base manual stop"
                self.base_finished_at = datetime.now().isoformat(timespec="seconds")
            else:
                self.base_cancel_event.clear()
                self.base_state = "manual"
                self.base_message = f"base manual {direction}"
                self.base_started_at = self.base_started_at or datetime.now().isoformat(timespec="seconds")
                self.base_finished_at = ""
                self.base_active_command = {"command": "manual", "direction": direction}
        self.set_base_velocity(vx, wz)
        self._event("base_state", {"state": self.base_state, "message": self.base_message})
        self._publish_base_state()

    def _track_base_motion(self, direction: str, linear_x: float, angular_z: float) -> None:
        now = time.monotonic()
        with self.lock:
            if direction == "stop" or (abs(linear_x) < 1e-9 and abs(angular_z) < 1e-9):
                self._close_base_motion_locked(now)
                return
            self._close_base_motion_locked(now)
            self.active_base_motion = {
                "command": direction,
                "linear_x": float(linear_x),
                "angular_z": float(angular_z),
                "_start_pose": self._base_pose_values_locked(),
                "start_stamp": datetime.now().isoformat(timespec="seconds"),
                "_start_monotonic": now,
            }

    def _close_base_motion_locked(self, now: float) -> None:
        if self.active_base_motion is None:
            return
        active = dict(self.active_base_motion)
        start = float(active.get("_start_monotonic", now))
        duration = max(0.0, now - start)
        if duration >= 0.05:
            end_pose = self._base_pose_values_locked()
            segment = self._make_relative_base_segment(active, end_pose, duration)
            segment["end_stamp"] = datetime.now().isoformat(timespec="seconds")
            self.base_motion_segments.append(segment)
            self.base_latest_result = {"recorded_segment": segment}
            self.base_message = self._describe_base_segment(segment)
        self.active_base_motion = None

    def _base_pose_values_locked(self) -> list[float]:
        item = self.latest.get("odom")
        if item is None or not isinstance(item[1], Odometry):
            return []
        pose = item[1].pose.pose
        return [pose.position.x, pose.position.y, _yaw_from_odom(item[1])]

    def _make_relative_base_segment(
        self, active: dict[str, Any], end_pose: list[float], duration: float
    ) -> dict[str, Any]:
        command = str(active.get("command", "stop"))
        linear_x = float(active.get("linear_x", 0.0))
        angular_z = float(active.get("angular_z", 0.0))
        start_pose = active.get("_start_pose", [])
        source = "timed"
        signed_distance = linear_x * duration
        signed_angle = angular_z * duration

        if len(start_pose) == 3 and len(end_pose) == 3:
            dx = float(end_pose[0]) - float(start_pose[0])
            dy = float(end_pose[1]) - float(start_pose[1])
            start_yaw = float(start_pose[2])
            signed_distance = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
            signed_angle = _angle_diff(float(end_pose[2]), start_yaw)
            source = "odom"

        if command in ("forward", "back"):
            if abs(signed_distance) < 1e-5:
                signed_distance = linear_x * duration
                source = "timed"
            action = "forward" if signed_distance >= 0.0 else "back"
            return {
                "type": "linear",
                "action": action,
                "distance_m": abs(float(signed_distance)),
                "signed_distance_m": float(signed_distance),
                "duration_sec": float(duration),
                "linear_x": linear_x,
                "source": source,
                "start_stamp": active.get("start_stamp", ""),
            }

        if command in ("left", "right"):
            if abs(signed_angle) < 1e-5:
                signed_angle = angular_z * duration
                source = "timed"
            action = "left" if signed_angle >= 0.0 else "right"
            return {
                "type": "angular",
                "action": action,
                "angle_rad": abs(float(signed_angle)),
                "signed_angle_rad": float(signed_angle),
                "duration_sec": float(duration),
                "angular_z": angular_z,
                "source": source,
                "start_stamp": active.get("start_stamp", ""),
            }

        return {
            "type": "timed",
            "action": command,
            "duration_sec": float(duration),
            "linear_x": linear_x,
            "angular_z": angular_z,
            "source": source,
            "start_stamp": active.get("start_stamp", ""),
        }

    def _describe_base_segment(self, segment: dict[str, Any]) -> str:
        action = str(segment.get("action", "base"))
        if segment.get("type") == "linear":
            return f"base recorded: {action} {float(segment.get('distance_m', 0.0)):.3f} m"
        if segment.get("type") == "angular":
            angle = math.degrees(float(segment.get("angle_rad", 0.0)))
            return f"base recorded: {action} {angle:.1f} deg"
        return f"base recorded: {action} {float(segment.get('duration_sec', 0.0)):.1f} s"

    def _publish_manual_base_velocity(self) -> None:
        with self.lock:
            velocity = self.manual_base_velocity
        if velocity is None:
            return
        self.set_base_velocity(velocity[0], velocity[1])

    def set_base_velocity(self, linear_x: float, angular_z: float) -> None:
        if not rclpy.ok():
            return
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        try:
            self.cmd_vel_pub.publish(twist)
        except Exception as exc:
            if "publisher's context is invalid" in str(exc):
                return
            raise

    def _publish_base_stop(self) -> None:
        self.set_base_velocity(0.0, 0.0)

    def _stop_base(self, reason: str) -> tuple[bool, str]:
        self.base_cancel_event.set()
        with self.lock:
            self.manual_base_velocity = None
            self._close_base_motion_locked(time.monotonic())
        self._publish_base_stop()
        with self.lock:
            running = self.base_worker is not None and self.base_worker.is_alive()
        if not running:
            self._set_base_state("idle", f"base stopped: {reason}")
        else:
            self._set_base_state("canceled", f"base stopping: {reason}")
        return True, self._base_snapshot_json()

    def _run_timed_base_motion(self, linear_x: float, angular_z: float, duration: float) -> None:
        max_duration = float(self.get_parameter("base_motion_max_segment_sec").value)
        duration = max(0.0, min(float(duration), max_duration))
        deadline = time.monotonic() + duration
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() < deadline:
            self.set_base_velocity(linear_x, angular_z)
            time.sleep(0.05)

    def _drive_distance(self, distance: float, speed_override: float | None = None) -> None:
        start = self._current_base_pose()
        heading_x = math.cos(start.yaw)
        heading_y = math.sin(start.yaw)
        tolerance = float(self.get_parameter("base_position_tolerance").value)
        speed = (
            abs(float(speed_override))
            if speed_override is not None
            else abs(float(self.get_parameter("base_replay_linear_speed").value))
        )
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.monotonic() + abs(distance) / max(speed, 1e-3) + 8.0
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() < deadline:
            current = self._current_base_pose(timeout=0.1)
            progress = (current.x - start.x) * heading_x + (current.y - start.y) * heading_y
            if sign * progress >= abs(distance) - tolerance:
                self.base_latest_result = {
                    "type": "linear",
                    "target_m": float(distance),
                    "progress_m": float(progress),
                }
                return
            self.set_base_velocity(sign * speed, 0.0)
            time.sleep(0.05)
        if self.base_cancel_event.is_set():
            return
        raise TimeoutError(f"base distance timeout target={distance:.3f}m")

    def _turn_relative(self, angle: float) -> None:
        current = self._current_base_pose()
        target = math.atan2(math.sin(current.yaw + angle), math.cos(current.yaw + angle))
        self._turn_to_yaw(target)

    def _turn_to_yaw(self, target_yaw: float) -> None:
        tolerance = math.radians(float(self.get_parameter("base_yaw_tolerance_deg").value))
        speed = abs(float(self.get_parameter("base_replay_angular_speed").value))
        initial_error = abs(_angle_diff(target_yaw, self._current_base_pose().yaw))
        deadline = time.monotonic() + max(12.0, initial_error / max(speed, 1e-3) + 8.0)
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() < deadline:
            current = self._current_base_pose(timeout=0.1)
            error = _angle_diff(target_yaw, current.yaw)
            if abs(error) <= tolerance:
                self.base_latest_result = {
                    "type": "angular",
                    "target_yaw_rad": float(target_yaw),
                    "error_rad": float(error),
                }
                return
            self.set_base_velocity(0.0, math.copysign(speed, error))
            time.sleep(0.05)
        if self.base_cancel_event.is_set():
            return
        raise TimeoutError(f"base yaw timeout target={math.degrees(target_yaw):.2f}deg")

    def _replay_base_motion(self, segments: list[dict[str, Any]], label: str) -> None:
        max_duration = float(self.get_parameter("base_motion_max_segment_sec").value)
        for index, segment in enumerate(segments, start=1):
            if self.base_cancel_event.is_set():
                break
            normalized = self._normalize_base_motion_segment(segment)
            motion_type = normalized.get("type")
            self._set_base_state(
                "running",
                f"base segment {index}/{len(segments)} for {label}: {motion_type}",
            )
            if motion_type == "linear":
                distance = float(normalized.get("signed_distance_m", 0.0))
                speed = abs(float(normalized.get("linear_x", 0.0)))
                self._drive_distance(distance, speed_override=speed if speed > 0.0 else None)
            elif motion_type == "angular":
                self._turn_relative(float(normalized.get("signed_angle_rad", 0.0)))
            else:
                duration = max(0.0, min(float(normalized.get("duration_sec", 0.0)), max_duration))
                linear_x = float(normalized.get("linear_x", 0.0))
                angular_z = float(normalized.get("angular_z", 0.0))
                self._run_timed_base_motion(linear_x, angular_z, duration)
            self._publish_base_stop()
            time.sleep(0.1)

    def _normalize_base_motion_segment(self, segment: dict[str, Any]) -> dict[str, Any]:
        if segment.get("type") == "linear":
            normalized = dict(segment)
            if "signed_distance_m" not in normalized:
                action = str(normalized.get("action", "forward"))
                distance = abs(float(normalized.get("distance_m", 0.0)))
                normalized["signed_distance_m"] = -distance if action == "back" else distance
            normalized["distance_m"] = abs(float(normalized.get("signed_distance_m", 0.0)))
            normalized["action"] = (
                "forward" if float(normalized.get("signed_distance_m", 0.0)) >= 0.0 else "back"
            )
            return normalized
        if segment.get("type") == "angular":
            normalized = dict(segment)
            if "signed_angle_rad" not in normalized:
                action = str(normalized.get("action", "left"))
                angle = abs(float(normalized.get("angle_rad", 0.0)))
                normalized["signed_angle_rad"] = -angle if action == "right" else angle
            normalized["angle_rad"] = abs(float(normalized.get("signed_angle_rad", 0.0)))
            normalized["action"] = (
                "left" if float(normalized.get("signed_angle_rad", 0.0)) >= 0.0 else "right"
            )
            return normalized
        if segment.get("type") == "timed":
            return dict(segment)

        command = str(segment.get("command", ""))
        active = {
            "command": command,
            "linear_x": float(segment.get("linear_x", 0.0)),
            "angular_z": float(segment.get("angular_z", 0.0)),
            "_start_pose": segment.get("start_pose", []),
            "start_stamp": segment.get("start_stamp", ""),
        }
        return self._make_relative_base_segment(
            active,
            segment.get("end_pose", []),
            float(segment.get("duration_sec", 0.0)),
        )

    def _current_base_pose(self, timeout: float | None = None) -> Pose2D:
        if timeout is None:
            timeout = float(self.get_parameter("base_pose_timeout_sec").value)
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while rclpy.ok() and not self.base_cancel_event.is_set() and time.monotonic() <= deadline:
            with self.lock:
                item = self.latest.get("odom")
            if item is not None and isinstance(item[1], Odometry):
                msg = item[1]
                pose = msg.pose.pose
                return Pose2D(pose.position.x, pose.position.y, _yaw_from_odom(msg))
            time.sleep(0.02)
        raise TimeoutError("missing /odom")

    def _set_base_state(self, state: str, message: str) -> None:
        with self.lock:
            self.base_state = state
            self.base_message = message
            if state in ("idle", "succeeded", "failed", "canceled"):
                self.base_finished_at = datetime.now().isoformat(timespec="seconds")
        self._event("base_state", {"state": state, "message": message})
        self._publish_base_state()

    def _base_message_snapshot(self) -> str:
        with self.lock:
            return self.base_message

    def _base_snapshot_dict(self) -> dict[str, Any]:
        with self.lock:
            pose_values = self._base_pose_values_locked()
            active_command = dict(self.base_active_command)
            result = dict(self.base_latest_result)
            segments = [dict(item) for item in self.base_motion_segments]
            worker_busy = self.base_worker is not None and self.base_worker.is_alive()
            manual = self.manual_base_velocity
            return {
                "state": self.base_state,
                "message": self.base_message,
                "started_at": self.base_started_at,
                "finished_at": self.base_finished_at,
                "worker_busy": worker_busy,
                "manual_velocity": list(manual) if manual is not None else [],
                "active_command": active_command,
                "latest_result": result,
                "recorded_segments": segments,
                "pose": pose_values,
            }

    def _base_snapshot_json(self) -> str:
        return json.dumps(self._base_snapshot_dict(), ensure_ascii=False, sort_keys=True)

    def _publish_base_state(self) -> None:
        msg = String()
        msg.data = self._base_snapshot_json()
        self.base_state_pub.publish(msg)

    def _run_task(self) -> None:
        use_warm_preview = (
            bool(self.get_parameter("preview_on_start").value)
            and bool(self.get_parameter("warm_execute_preview").value)
            and bool(self.get_parameter("execute_real").value)
        )
        if not use_warm_preview:
            self._stop_idle_preview("grasp task starting")
        ok, message = self._wait_for_aubo_control_release(4.0)
        if not ok:
            self._event("idle_preview_control_release_timeout", {"message": message})
        task_id = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        run_dir = self._log_root() / task_id
        run_dir.mkdir(parents=True, exist_ok=True)
        with self.lock:
            self.task_id = task_id
            self.run_dir = run_dir
            self.started_at = datetime.now().isoformat(timespec="seconds")
            self.finished_at = ""
            self.returncode = None
            self.grasp_preview_log_dir = ""
            self.preflight_checks = []
            self.runner_success_requested = False
        skip_preflight = bool(self.get_parameter("skip_preflight").value)
        self._set_state(
            "preflight",
            "startup preflight trusted" if skip_preflight else "checking real-hardware readiness",
        )
        self._write_json(run_dir / "task_request.json", self._task_request())
        try:
            self._resolved_release_basket_joints()
        except ValueError as exc:
            self._event("release_point_invalid", {"error": str(exc)})
            self._finish_task("failed", str(exc), returncode=None)
            return

        result = (
            PreflightResult(ok=True)
            if skip_preflight
            else self._run_preflight(
                set_autonomous=bool(self.get_parameter("set_safety_autonomous_on_start").value)
            )
        )
        with self.lock:
            self.preflight_checks = result.checks
        self._write_json(run_dir / "preflight.json", {"ok": result.ok, "checks": result.checks})
        if not result.ok:
            message = self._preflight_failure_message(result)
            self._write_preflight_process_log(run_dir, result)
            self._event(
                "preflight_failed",
                {"failures": self._failed_required_checks(result), "message": message},
            )
            self._finish_task("failed", f"preflight failed: {message}", returncode=None)
            return

        if use_warm_preview:
            ok, message = self._run_warm_execute_preview(run_dir)
            self._finish_task("succeeded" if ok else "failed", message, returncode=0 if ok else None)
            return

        command, env = self._runner_command()
        recovery_steps = self._planning_recovery_steps()
        self._write_json(
            run_dir / "runner.json",
            {
                "command": command,
                "environment_overrides": self._logged_environment(env),
                "planning_recovery_steps": recovery_steps,
            },
        )

        returncode: int | None = None
        executed_recovery: list[dict[str, Any]] = []
        task_succeeded = False
        final_failure_message = ""
        with self.lock:
            self.last_planning_recovery_steps.clear()
        recovery_enabled = (
            bool(self.get_parameter("planning_recovery_base_enabled").value)
            and bool(self.get_parameter("execute_real").value)
            and bool(self.get_parameter("confirm_execute_real").value)
        )
        max_planning_attempts = 1 + (len(recovery_steps) if recovery_enabled else 0)
        max_grasp_attempts = max(int(self.get_parameter("max_grasp_attempts").value), 1)
        retry_on_miss = bool(self.get_parameter("retry_on_gripper_miss").value)
        retry_delay = max(float(self.get_parameter("gripper_miss_retry_delay_sec").value), 0.0)
        for grasp_attempt in range(max_grasp_attempts):
            if self.cancel_event.is_set():
                break
            if grasp_attempt > 0 and retry_delay > 0.0:
                time.sleep(retry_delay)
            for attempt in range(max_planning_attempts):
                if self.cancel_event.is_set():
                    break
                process_log = self._attempt_process_log_path(run_dir, grasp_attempt, attempt)
                attempt_label = (
                    f"grasp attempt {grasp_attempt + 1}/{max_grasp_attempts}, "
                    f"plan try {attempt + 1}/{max_planning_attempts}"
                )
                self._set_state("running", f"grasp_preview pipeline started: {attempt_label}")
                try:
                    returncode = self._run_runner_process(command, env, process_log)
                except FileNotFoundError as exc:
                    self._event("runner_error", {"error": str(exc)})
                    self._finish_task("failed", f"runner missing: {exc}", returncode=None)
                    return
                except Exception as exc:  # pragma: no cover - defensive around live process I/O.
                    self._event("runner_error", {"error": str(exc)})
                    self._finish_task("failed", f"runner error: {exc}", returncode=None)
                    return

                with self.lock:
                    runner_success = self.runner_success_requested
                    planning_failed = self.runner_planning_failed
                    real_blocked = self.runner_real_execution_blocked
                    real_started = self.runner_real_execution_started
                    gripper_miss = self.runner_gripper_capture_failed

                if self.cancel_event.is_set():
                    break
                if real_blocked:
                    final_failure_message = "real execution blocked"
                    break
                if gripper_miss:
                    final_failure_message = "gripper did not capture object"
                    break
                if (
                    planning_failed
                    and not real_started
                    and recovery_enabled
                    and attempt < len(recovery_steps)
                ):
                    step = recovery_steps[attempt]
                    try:
                        self._run_planning_recovery_step(step, attempt + 1)
                        executed_recovery.append(step)
                        with self.lock:
                            self.last_planning_recovery_steps.append(dict(step))
                        continue
                    except Exception as exc:
                        self._event("planning_recovery_failed", {"step": step, "error": str(exc)})
                        final_failure_message = f"planning recovery failed: {exc}"
                        break
                if planning_failed and not real_started:
                    final_failure_message = "planning failed before real motion"
                    break
                if runner_success or returncode == 0:
                    task_succeeded = True
                    final_failure_message = ""
                    break
                final_failure_message = f"grasp task failed with return code {returncode}"
                break

            if task_succeeded or self.cancel_event.is_set():
                break
            with self.lock:
                gripper_miss = self.runner_gripper_capture_failed
            if gripper_miss and retry_on_miss and grasp_attempt < max_grasp_attempts - 1:
                self._event(
                    "grasp_retry",
                    {
                        "reason": "gripper_miss",
                        "next_attempt": grasp_attempt + 2,
                        "max_attempts": max_grasp_attempts,
                    },
                )
                continue
            break

        if (
            not self.cancel_event.is_set()
            and not task_succeeded
            and executed_recovery
            and bool(self.get_parameter("planning_recovery_restore_on_failure").value)
        ):
            self._restore_planning_recovery_steps(executed_recovery)
            with self.lock:
                self.last_planning_recovery_steps.clear()

        if self.cancel_event.is_set():
            self._finish_task("canceled", "task canceled", returncode=returncode)
        elif task_succeeded:
            self._finish_task("succeeded", "grasp task complete", returncode=returncode)
        else:
            self._finish_task(
                "failed",
                final_failure_message or f"grasp task failed with return code {returncode}",
                returncode=returncode,
            )

    def _attempt_process_log_path(self, run_dir: Path, grasp_attempt: int, plan_attempt: int) -> Path:
        if grasp_attempt == 0 and plan_attempt == 0:
            return run_dir / "process.log"
        return run_dir / f"process_attempt_{grasp_attempt + 1}_plan_{plan_attempt + 1}.log"

    def _run_warm_execute_preview(self, run_dir: Path) -> tuple[bool, str]:
        process_log = run_dir / "process.log"
        with self.lock:
            self.runner_success_requested = False
            self.runner_planning_failed = False
            self.runner_real_execution_started = False
            self.runner_real_execution_blocked = False
            self.runner_gripper_capture_failed = False
            self.runner_waiting_for_execute_trigger = False
        self._start_idle_preview()
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and rclpy.ok() and not self.cancel_event.is_set():
            with self.lock:
                process = self.idle_preview_process
            if process is not None and process.poll() is None:
                break
            time.sleep(0.05)
        else:
            process_log.write_text("warm preview pipeline did not start\n", encoding="utf-8")
            return False, "warm preview pipeline did not start"

        with process_log.open("w", encoding="utf-8") as log_file:
            log_file.write("Using warm grasp_preview pipeline.\n")
            log_file.write("See idle_preview log for full pipeline output.\n")
            log_file.flush()
            timeout = max(float(self.get_parameter("warm_execute_timeout_sec").value), 1.0)
            self.restart_search_pub.publish(Empty())
            log_file.write("Restart search requested before execute trigger.\n")
            log_file.flush()
            time.sleep(0.1)
            self.real_execute_trigger_pub.publish(Empty())
            log_file.write("Execute trigger topic armed; repeating until motion starts.\n")
            log_file.flush()
            self._event(
                "warm_execute_triggered",
                {
                    "pid": process.pid,
                    "message": "topic trigger armed",
                    "trigger_wait_sec": 0.0,
                },
            )
            deadline = time.monotonic() + timeout
            next_trigger = time.monotonic()
            motion_deadline_extended = False
            while time.monotonic() < deadline and rclpy.ok() and not self.cancel_event.is_set():
                with self.lock:
                    success = self.runner_success_requested
                    planning_failed = self.runner_planning_failed
                    real_blocked = self.runner_real_execution_blocked
                    gripper_miss = self.runner_gripper_capture_failed
                    real_started = self.runner_real_execution_started
                if not real_started and time.monotonic() >= next_trigger:
                    self.real_execute_trigger_pub.publish(Empty())
                    next_trigger = time.monotonic() + 0.3
                if real_started and not motion_deadline_extended:
                    deadline = max(deadline, time.monotonic() + 45.0)
                    motion_deadline_extended = True
                if success:
                    return True, "grasp task complete"
                if gripper_miss:
                    return False, "gripper did not capture object"
                if real_blocked:
                    return False, "real execution blocked"
                if planning_failed:
                    return False, "planning failed before real motion"
                if process.poll() is not None:
                    return False, f"warm preview pipeline exited: {process.returncode}"
                time.sleep(0.05)
            if self.cancel_event.is_set():
                self._stop_idle_preview("warm execution canceled")
                return False, "task canceled"
            return False, "warm execute timeout"

    def _run_runner_process(
        self, command: list[str], env: dict[str, str], process_log: Path
    ) -> int | None:
        with self.lock:
            self.runner_success_requested = False
            self.runner_planning_failed = False
            self.runner_real_execution_started = False
            self.runner_real_execution_blocked = False
            self.runner_gripper_capture_failed = False
        with process_log.open("w", encoding="utf-8") as log_file:
            self.process = subprocess.Popen(
                command,
                cwd=str(self.root),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            try:
                if self.process.stdout is not None:
                    for line in self.process.stdout:
                        log_file.write(line)
                        log_file.flush()
                        self._parse_process_line(line)
                        if self.cancel_event.is_set():
                            self._terminate_process("cancel requested")
                return self.process.wait()
            finally:
                self.process = None

    def _failed_required_checks(self, result: PreflightResult) -> list[dict[str, Any]]:
        return [
            dict(item)
            for item in result.checks
            if bool(item.get("required", True)) and not bool(item.get("ok", False))
        ]

    def _preflight_failure_message(self, result: PreflightResult) -> str:
        failures = self._failed_required_checks(result)
        if not failures:
            return "unknown preflight failure"
        parts = []
        for item in failures[:4]:
            name = str(item.get("name", "check")).strip()
            message = str(item.get("message", "")).strip()
            parts.append(f"{name}: {message}" if message else name)
        if len(failures) > 4:
            parts.append(f"+{len(failures) - 4} more")
        return "; ".join(parts)

    def _write_preflight_process_log(
        self, run_dir: Path, result: PreflightResult
    ) -> None:
        process_log = run_dir / "process.log"
        with process_log.open("w", encoding="utf-8") as log_file:
            log_file.write("Grasp task did not start the grasp_preview runner.\n")
            log_file.write("Reason: preflight failed before process launch.\n\n")
            failures = self._failed_required_checks(result)
            if failures:
                log_file.write("Required failures:\n")
                for item in failures:
                    log_file.write(
                        f"  - {item.get('name', 'check')}: {item.get('message', '')}\n"
                    )
                log_file.write("\n")
            log_file.write("All preflight checks:\n")
            for item in result.checks:
                status = "OK" if bool(item.get("ok", False)) else "FAIL"
                required = "required" if bool(item.get("required", True)) else "optional"
                log_file.write(
                    f"  [{status}] {item.get('name', 'check')} ({required}): "
                    f"{item.get('message', '')}\n"
                )
            log_file.write("\n")
            log_file.write(f"preflight.json: {run_dir / 'preflight.json'}\n")
            log_file.write(f"summary.json: {run_dir / 'summary.json'}\n")

    def _restore_worker(self) -> None:
        self.cancel_event.set()
        self._terminate_process("restore requested")
        self._stop_base("restore requested")
        with self.lock:
            steps = [dict(step) for step in self.last_planning_recovery_steps]
        if not steps:
            self._set_state("idle", "restore complete: no recorded recovery motion")
            self._event("restore_complete", {"restored_steps": 0})
            return
        self._set_state("running", f"restoring base recovery motion: {len(steps)} step(s)")
        try:
            self._restore_planning_recovery_steps(steps)
            with self.lock:
                self.last_planning_recovery_steps.clear()
            self._set_state("idle", "restore complete")
            self._event("restore_complete", {"restored_steps": len(steps)})
        except Exception as exc:
            self._set_state("failed", f"restore failed: {exc}")
            self._event("restore_failed", {"error": str(exc), "steps": steps})

    def _planning_recovery_steps(self) -> list[dict[str, Any]]:
        raw = str(self.get_parameter("planning_recovery_base_sequence").value).strip()
        if not raw:
            return []
        steps: list[dict[str, Any]] = []
        for token in raw.split(","):
            token = token.strip()
            if not token:
                continue
            if ":" in token:
                action, value = token.split(":", 1)
            else:
                action, value = token, ""
            action = action.strip().lower()
            value = value.strip().lower()
            if action in ("forward", "back", "backward", "reverse"):
                distance = float(value.rstrip("m") or "0.04")
                if action in ("back", "backward", "reverse"):
                    distance = -abs(distance)
                else:
                    distance = abs(distance)
                steps.append({"type": "linear", "signed_distance_m": distance})
            elif action in ("turn_left", "left", "turn_right", "right"):
                if value.endswith("deg"):
                    angle = math.radians(float(value[:-3]))
                elif value.endswith("rad"):
                    angle = float(value[:-3])
                else:
                    angle = math.radians(float(value or "5"))
                if action in ("turn_right", "right"):
                    angle = -abs(angle)
                else:
                    angle = abs(angle)
                steps.append({"type": "angular", "signed_angle_rad": angle})
        return steps

    def _run_planning_recovery_step(self, step: dict[str, Any], attempt: int) -> None:
        self._set_state("running", f"planning recovery base move {attempt}: {step}")
        self._event("planning_recovery_start", {"attempt": attempt, "step": step})
        self.base_cancel_event.clear()
        try:
            self._execute_recovery_step(step)
            self._event("planning_recovery_done", {"attempt": attempt, "step": step})
        finally:
            self._publish_base_stop()
            time.sleep(0.25)

    def _restore_planning_recovery_steps(self, steps: list[dict[str, Any]]) -> None:
        for index, step in enumerate(reversed(steps), start=1):
            inverse = self._inverse_recovery_step(step)
            try:
                self._set_state("running", f"restoring base after failed planning {index}: {inverse}")
                self.base_cancel_event.clear()
                self._execute_recovery_step(inverse)
            except Exception as exc:
                self._event("planning_recovery_restore_failed", {"step": inverse, "error": str(exc)})
                break
            finally:
                self._publish_base_stop()
                time.sleep(0.25)

    def _inverse_recovery_step(self, step: dict[str, Any]) -> dict[str, Any]:
        if step.get("type") == "linear":
            return {
                "type": "linear",
                "signed_distance_m": -float(step.get("signed_distance_m", 0.0)),
            }
        if step.get("type") == "angular":
            return {
                "type": "angular",
                "signed_angle_rad": -float(step.get("signed_angle_rad", 0.0)),
            }
        return dict(step)

    def _execute_recovery_step(self, step: dict[str, Any]) -> None:
        if self.cancel_event.is_set():
            return
        if step.get("type") == "linear":
            distance = float(step.get("signed_distance_m", 0.0))
            speed = abs(float(self.get_parameter("base_replay_linear_speed").value))
            duration = abs(distance) / max(speed, 1e-3)
            self._run_timed_base_motion(math.copysign(speed, distance), 0.0, duration)
            return
        if step.get("type") == "angular":
            angle = float(step.get("signed_angle_rad", 0.0))
            speed = abs(float(self.get_parameter("base_replay_angular_speed").value))
            duration = abs(angle) / max(speed, 1e-3)
            self._run_timed_base_motion(0.0, math.copysign(speed, angle), duration)
            return
        raise ValueError(f"unsupported recovery step: {step}")

    def _run_preflight(self, *, set_autonomous: bool) -> PreflightResult:
        timeout = max(float(self.get_parameter("preflight_timeout_sec").value), 0.1)
        result = PreflightResult(ok=True)

        result.add("workspace_root", self.root.exists(), str(self.root), required=True)
        configured_runner = str(self.get_parameter("runner_script").value).strip()
        if configured_runner:
            runner = self._runner_path(configured_runner)
            result.add("runner_script", runner.exists() and os.access(runner, os.X_OK), str(runner), required=True)
        else:
            pipeline = Path(__file__).with_name("grasp_preview_pipeline.py")
            result.add("grasp_preview_pipeline", pipeline.is_file(), str(pipeline), required=True)
        result.add(
            "install_setup",
            (self.root / "install" / "setup.bash").exists(),
            str(self.root / "install" / "setup.bash"),
            required=True,
        )
        execute_real = bool(self.get_parameter("execute_real").value)
        confirmed = bool(self.get_parameter("confirm_execute_real").value)
        result.add(
            "real_execution_confirmation",
            (not execute_real) or confirmed,
            "execute_real requires confirm_execute_real:=true",
            required=True,
        )
        if execute_real:
            ok, message = self._aubo_control_owner_available()
            result.add("aubo_control_owner", ok, message, required=True)
            ok, message = self._aubo_teach_gate_available()
            result.add("aubo_teach_gate", ok, message, required=True)

        if set_autonomous:
            required = bool(self.get_parameter("require_safety_state_machine").value)
            ok, message = self._call_trigger(
                self.set_autonomous_client,
                timeout if required else 0.05,
            )
            result.add("safety_set_autonomous", ok, message, required=required)

        if execute_real and bool(self.get_parameter("require_joint_states").value):
            self._refresh_aubo_joints_from_rpc()

        required_inputs: list[str] = []
        if bool(self.get_parameter("require_aubo_status").value):
            required_inputs.append("aubo_status")
        if bool(self.get_parameter("require_joint_states").value):
            required_inputs.append("aubo_joints")
        if bool(self.get_parameter("require_gripper_status").value):
            required_inputs.append("gripper_status")
        if bool(self.get_parameter("require_odom").value):
            required_inputs.append("odom")
        if bool(self.get_parameter("require_safety_state_machine").value):
            required_inputs.append("safety_state")
        if bool(self.get_parameter("require_camera_topics").value):
            required_inputs.extend(["color_image", "depth_image"])
        self._wait_for_fresh_inputs(timeout, required_inputs)
        self._add_cached_check(
            result,
            "aubo_status",
            required=bool(self.get_parameter("require_aubo_status").value),
            max_age=max(timeout + 1.0, 2.0),
            validator=lambda value: "not reachable" not in str(value).lower(),
        )
        self._add_cached_check(
            result,
            "aubo_joints",
            required=bool(self.get_parameter("require_joint_states").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "gripper_status",
            required=bool(self.get_parameter("require_gripper_status").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "odom",
            required=bool(self.get_parameter("require_odom").value),
            max_age=max(timeout + 1.0, 2.0),
        )
        self._add_cached_check(
            result,
            "safety_state",
            required=bool(self.get_parameter("require_safety_state_machine").value),
            max_age=max(timeout + 1.0, 2.0),
            validator=lambda value: not str(value).lower().startswith(("disabled", "estop", "fault")),
        )
        if bool(self.get_parameter("require_camera_topics").value):
            self._add_cached_check(result, "color_image", required=True, max_age=max(timeout + 1.0, 2.0))
            self._add_cached_check(result, "depth_image", required=True, max_age=max(timeout + 1.0, 2.0))
        return result

    def _aubo_control_owner_available(self) -> tuple[bool, str]:
        path = Path(str(self.get_parameter("aubo_control_owner_path").value))
        ok, message = control_owner_available(path)
        if ok:
            return True, message
        return False, f"{message}; turn teach off or stop manual jog before starting grasp"

    def _aubo_teach_gate_available(self) -> tuple[bool, str]:
        path = Path(str(self.get_parameter("aubo_teach_flag_path").value))
        if not path.exists():
            return True, f"available: {path}"
        try:
            text = path.read_text(encoding="utf-8", errors="replace").strip()
        except OSError as exc:
            return False, f"teach gate unreadable {path}: {exc}"
        if text.startswith("1"):
            return (
                False,
                f"active: {path} value={text!r}; turn teach off or stop manual jog before starting grasp",
            )
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            return False, f"inactive stale teach gate {path} could not be cleared: {exc}"
        return True, f"cleared inactive stale teach gate value={text!r}"

    def _refresh_aubo_joints_from_rpc(self) -> None:
        try:
            ip = str(self.get_parameter("real_sdk_ip").value)
            port = int(self.get_parameter("real_sdk_rpc_port").value)
            timeout = max(float(self.get_parameter("real_sdk_rpc_timeout").value), 0.1)
            with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                raw = rpc.robot_call("RobotState.getJointPositions")
            joints = self._normalise_aubo_rpc_joints(raw)
        except Exception as exc:
            self._event("aubo_joints_rpc_refresh_failed", {"error": str(exc)})
            return
        names = [canonical for canonical, _aliases in AUBO_JOINT_ALIASES]
        with self.lock:
            self.latest["aubo_joints"] = (
                time.monotonic(),
                {name: value for name, value in zip(names, joints)},
            )
        self._event("aubo_joints_rpc_refresh", {"source": f"{ip}:{port}", "joints": joints})

    def _normalise_aubo_rpc_joints(self, value: Any) -> list[float]:
        if isinstance(value, dict):
            for key in ("joint_positions", "jointPositions", "positions", "q", "data", "value"):
                if key in value:
                    return self._normalise_aubo_rpc_joints(value[key])
        if isinstance(value, (list, tuple)) and len(value) >= 6:
            joints = [float(item) for item in value[:6]]
            if max(abs(item) for item in joints) > 10.0:
                raise RuntimeError(f"Aubo SDK joint values do not look like radians: {joints}")
            return joints
        raise RuntimeError(f"cannot parse Aubo SDK joint positions from {value!r}")

    def _wait_for_fresh_inputs(self, timeout: float, keys: list[str] | None = None) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                if not keys and self.latest:
                    return
                if keys and all(key in self.latest for key in keys):
                    return
            time.sleep(0.05)

    def _add_cached_check(
        self,
        result: PreflightResult,
        name: str,
        *,
        required: bool,
        max_age: float,
        validator=None,
    ) -> None:
        with self.lock:
            item = self.latest.get(name)
        if item is None:
            result.add(name, False, "no recent message", required=required)
            return
        stamp, value = item
        age = time.monotonic() - stamp
        ok = age <= max_age
        message = f"age={age:.2f}s value={self._short_value(value)}"
        if ok and validator is not None:
            try:
                ok = bool(validator(value))
            except Exception as exc:
                ok = False
                message += f" validator_error={exc}"
        result.add(name, ok, message, required=required)

    def _call_trigger(self, client, timeout: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"service unavailable: {getattr(client, 'srv_name', 'trigger')}"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return False, f"service timeout: {getattr(client, 'srv_name', 'trigger')}"
        response = future.result()
        if response is None:
            return False, f"empty response: {getattr(client, 'srv_name', 'trigger')}"
        return bool(response.success), str(response.message)

    def _runner_command(self) -> tuple[list[str], dict[str, str]]:
        execute_real = bool(self.get_parameter("execute_real").value)
        configured_runner = str(self.get_parameter("runner_script").value).strip()
        extra = shlex.split(str(self.get_parameter("extra_args").value))
        if not configured_runner and not any(
            token == "--save-dir" or token.startswith("--save-dir=") for token in extra
        ):
            with self.lock:
                run_dir = self.run_dir
            if run_dir is not None:
                extra.extend(["--save-dir", str(run_dir / "preview")])

        env = os.environ.copy()
        env["ARACHNE_GRASP_EXECUTE_REAL"] = "true" if execute_real else "false"
        env["ARACHNE_GRASP_START_MOVEIT"] = "false"
        env["ARACHNE_GRASP_START_MODEL"] = "false"
        env["ARACHNE_GRASP_START_CAMERA"] = "false"
        env["ARACHNE_GRASP_DISPLAY_FRAME_PREFIX"] = ""
        env["ARACHNE_GRASP_CAMERA_PARENT_FRAME"] = "ee_camera_link"
        env["ARACHNE_GRASP_WITH_RVIZ"] = "true" if bool(self.get_parameter("with_rviz").value) else "false"
        env["ARACHNE_GRASP_CLASSES"] = str(self.get_parameter("classes").value)
        env["ARACHNE_GRASP_CONF"] = str(self.get_parameter("confidence").value)
        env["ARACHNE_GRASP_DEVICE_ID"] = str(self.get_parameter("device_id").value)
        env["ARACHNE_GRASP_REAL_EXECUTE_BACKEND"] = str(
            self.get_parameter("real_execute_backend").value
        )
        env["ARACHNE_GRASP_REAL_SDK_IP"] = str(self.get_parameter("real_sdk_ip").value)
        env["ARACHNE_GRASP_REAL_RETURN_HOME"] = (
            "true" if bool(self.get_parameter("real_return_home").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_FIXED_POST_GRASP"] = (
            "true" if bool(self.get_parameter("real_fixed_post_grasp").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_FIXED_SEARCH_JOINTS"] = str(
            self.get_parameter("real_fixed_search_joints").value
        )
        env["ARACHNE_GRASP_REAL_FIXED_LIFT_JOINTS"] = str(
            self.get_parameter("real_fixed_lift_joints").value
        )
        env["ARACHNE_GRASP_REAL_FIXED_BASKET_JOINTS"] = self._resolved_release_basket_joints()
        with self.lock:
            latest_joints = self.latest.get("aubo_joints")
        if latest_joints is not None:
            joint_map = latest_joints[1]
            names = [canonical for canonical, _aliases in AUBO_JOINT_ALIASES]
            if isinstance(joint_map, dict) and all(name in joint_map for name in names):
                env["ARACHNE_GRASP_ARM_JOINTS"] = ",".join(
                    f"{float(joint_map[name]):.12g}" for name in names
                )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_SPEED"] = str(
            self.get_parameter("real_sdk_move_speed").value
        )
        env["ARACHNE_GRASP_REAL_SDK_MOVE_ACCEL"] = str(
            self.get_parameter("real_sdk_move_accel").value
        )
        env["ARACHNE_GRASP_REAL_GRIPPER_SETTLE_SEC"] = str(
            self.get_parameter("real_gripper_settle_sec").value
        )
        env["ARACHNE_GRASP_REAL_PREOPEN_GRIPPER"] = (
            "true" if bool(self.get_parameter("real_preopen_gripper").value) else "false"
        )
        env["ARACHNE_GRASP_REAL_SDK_TEACH_FLAG_PATH"] = str(
            self.get_parameter("aubo_teach_flag_path").value
        )
        env["ARACHNE_GRASP_AUBO_CONTROL_OWNER_PATH"] = str(
            self.get_parameter("aubo_control_owner_path").value
        )
        env["ARACHNE_GRASP_AUBO_CONTROL_OWNER_NAME"] = str(
            self.get_parameter("aubo_control_owner_name").value
        )
        env["ARACHNE_GRASP_AUBO_MOVE_JOINT_ACTION_NAME"] = str(
            self.get_parameter("aubo_move_joint_action_name").value
        )
        env["ARACHNE_GRASP_PREFER_AUBO_MOVE_JOINT_ACTION"] = (
            "true" if bool(self.get_parameter("prefer_aubo_move_joint_action").value) else "false"
        )
        env["ARACHNE_GRASP_AUBO_MOVE_JOINT_FALLBACK_INTERNAL"] = (
            "true"
            if bool(self.get_parameter("aubo_move_joint_fallback_internal").value)
            else "false"
        )
        env["ARACHNE_GRASP_AUBO_MOVE_JOINT_WAIT_SERVER_SEC"] = str(
            self.get_parameter("aubo_move_joint_wait_server_sec").value
        )
        env["ARACHNE_GRASP_BASE_OFFSET"] = str(self.get_parameter("grasp_base_offset").value)
        if bool(self.get_parameter("confirm_execute_real").value):
            env["ARACHNE_CONFIRM_GRASP_EXECUTE_REAL"] = "YES"
        command = self._configured_or_pipeline_command(
            configured_runner,
            extra,
            env,
            execute_real=execute_real,
        )
        return command, env

    def _resolved_release_basket_joints(self) -> str:
        release_point = str(self.get_parameter("release_point").value).strip().lower()
        if release_point not in RELEASE_POINTS:
            raise ValueError(f"unsupported release_point: {release_point or '<empty>'}")
        legacy = str(self.get_parameter("real_fixed_basket_joints").value).strip()
        point_a = str(self.get_parameter("release_point_a_joints").value).strip()
        point_b = str(self.get_parameter("release_point_b_joints").value).strip()
        if release_point == "basket_b":
            if not point_b:
                raise ValueError("release_point:=basket_b requires release_point_b_joints")
            return point_b
        if legacy and legacy != DEFAULT_REAL_FIXED_BASKET_JOINTS:
            return legacy
        return point_a or legacy

    def _configured_or_pipeline_command(
        self,
        configured_runner: str,
        extra: list[str],
        env: dict[str, str],
        *,
        execute_real: bool,
    ) -> list[str]:
        if configured_runner:
            runner_path = self._runner_path(configured_runner)
            command = [str(runner_path)]
            if extra:
                command.extend(extra)
            return command
        return self._pipeline_command(extra, env, execute_real=execute_real)

    def _runner_path(self, configured_runner: str) -> Path:
        configured = Path(configured_runner)
        if configured.is_absolute():
            return configured
        return self.root / configured

    def _pipeline_command(
        self,
        extra: list[str],
        env: dict[str, str],
        *,
        execute_real: bool,
    ) -> list[str]:
        command = [
            os.environ.get("ARACHNE_SYSTEM_PYTHON", sys.executable),
            "-m",
            "arachne_operator.grasp_preview_pipeline",
            "--model",
            self._default_yolo_model(),
            "--venv",
            os.environ.get("ARACHNE_GRASP_YOLO_VENV", str(self.root / "yolo_workspace/.venv")),
            "--yolo-task",
            os.environ.get("ARACHNE_GRASP_YOLO_TASK", "detect"),
            "--classes",
            env["ARACHNE_GRASP_CLASSES"],
            "--conf",
            env["ARACHNE_GRASP_CONF"],
            "--device-id",
            env["ARACHNE_GRASP_DEVICE_ID"],
            f"--grasp-base-offset={env['ARACHNE_GRASP_BASE_OFFSET']}",
            "--aubo-base-frame",
            "aubo_base_link",
            "--real-execute-backend",
            env["ARACHNE_GRASP_REAL_EXECUTE_BACKEND"],
            "--real-sdk-ip",
            env["ARACHNE_GRASP_REAL_SDK_IP"],
            "--real-sdk-move-speed",
            env["ARACHNE_GRASP_REAL_SDK_MOVE_SPEED"],
            "--real-sdk-move-accel",
            env["ARACHNE_GRASP_REAL_SDK_MOVE_ACCEL"],
            "--real-gripper-settle-sec",
            env["ARACHNE_GRASP_REAL_GRIPPER_SETTLE_SEC"],
            "--real-sdk-teach-flag-path",
            env["ARACHNE_GRASP_REAL_SDK_TEACH_FLAG_PATH"],
            "--real-sdk-control-owner-path",
            env["ARACHNE_GRASP_AUBO_CONTROL_OWNER_PATH"],
            "--real-sdk-control-owner-name",
            env["ARACHNE_GRASP_AUBO_CONTROL_OWNER_NAME"],
            "--aubo-move-joint-action-name",
            env["ARACHNE_GRASP_AUBO_MOVE_JOINT_ACTION_NAME"],
            "--aubo-move-joint-wait-server-sec",
            env["ARACHNE_GRASP_AUBO_MOVE_JOINT_WAIT_SERVER_SEC"],
        ]
        command.append(
            "--real-preopen-gripper"
            if env["ARACHNE_GRASP_REAL_PREOPEN_GRIPPER"] == "true"
            else "--no-real-preopen-gripper"
        )
        if execute_real:
            command.append("--execute-real")
        if env.get("ARACHNE_CONFIRM_GRASP_EXECUTE_REAL") == "YES":
            command.extend(["--execute-real-confirm", "YES"])
        command.append(
            "--real-return-home"
            if env["ARACHNE_GRASP_REAL_RETURN_HOME"] == "true"
            else "--no-real-return-home"
        )
        if env["ARACHNE_GRASP_REAL_FIXED_POST_GRASP"] == "true":
            command.append("--real-fixed-post-grasp")
        fixed_lift = env.get("ARACHNE_GRASP_REAL_FIXED_LIFT_JOINTS", "").strip()
        if fixed_lift:
            command.append(f"--real-fixed-lift-joints={fixed_lift}")
        fixed_basket = env.get("ARACHNE_GRASP_REAL_FIXED_BASKET_JOINTS", "").strip()
        if fixed_basket:
            command.append(f"--real-fixed-basket-joints={fixed_basket}")
        fixed_search = env.get("ARACHNE_GRASP_REAL_FIXED_SEARCH_JOINTS", "").strip()
        if fixed_search:
            command.append(f"--real-fixed-search-joints={fixed_search}")
        start_joints = env.get("ARACHNE_GRASP_ARM_JOINTS", "").strip()
        if start_joints:
            command.append(f"--real-start-joints={start_joints}")
        command.append(
            "--prefer-aubo-move-joint-action"
            if env["ARACHNE_GRASP_PREFER_AUBO_MOVE_JOINT_ACTION"] == "true"
            else "--no-prefer-aubo-move-joint-action"
        )
        command.append(
            "--aubo-move-joint-fallback-internal"
            if env["ARACHNE_GRASP_AUBO_MOVE_JOINT_FALLBACK_INTERNAL"] == "true"
            else "--no-aubo-move-joint-fallback-internal"
        )
        command.extend(extra)
        return command

    def _default_yolo_model(self) -> str:
        configured = os.environ.get("ARACHNE_GRASP_YOLO_MODEL", "").strip()
        if configured:
            return configured
        for rel in (
            "yolo_workspace/weights/yolo26n.onnx",
            "yolo_workspace/weights/yolo26n.pt",
            "yolo_workspace/weights/yolo26m.onnx",
            "yolo_workspace/weights/yolo26m.pt",
        ):
            path = self.root / rel
            if path.exists():
                return str(path)
        return str(self.root / "yolo_workspace/weights/yolo26n.onnx")

    def _log_root(self) -> Path:
        configured = Path(str(self.get_parameter("log_root").value)).expanduser()
        if configured.is_absolute():
            return configured
        return self.root / configured

    def _task_request(self) -> dict[str, Any]:
        names = (
            "execute_real",
            "confirm_execute_real",
            "with_rviz",
            "classes",
            "confidence",
            "device_id",
            "real_execute_backend",
            "real_return_home",
            "real_fixed_post_grasp",
            "real_fixed_search_joints",
            "real_fixed_lift_joints",
            "real_fixed_basket_joints",
            "release_point",
            "release_point_a_joints",
            "release_point_b_joints",
            "real_sdk_move_speed",
            "real_sdk_move_accel",
            "real_gripper_settle_sec",
            "real_preopen_gripper",
            "aubo_teach_flag_path",
            "aubo_control_owner_path",
            "aubo_control_owner_name",
            "aubo_move_joint_action_name",
            "prefer_aubo_move_joint_action",
            "aubo_move_joint_fallback_internal",
            "aubo_move_joint_wait_server_sec",
            "grasp_base_offset",
            "extra_args",
            "preview_on_start",
            "warm_execute_preview",
            "preview_extra_args",
            "planning_recovery_base_enabled",
            "planning_recovery_base_sequence",
            "planning_recovery_restore_on_failure",
            "skip_preflight",
            "max_grasp_attempts",
            "retry_on_gripper_miss",
            "gripper_miss_retry_delay_sec",
            "require_aubo_status",
            "require_gripper_status",
            "require_camera_topics",
        )
        return {name: self.get_parameter(name).value for name in names}

    def _logged_environment(self, env: dict[str, str]) -> dict[str, str]:
        keys = sorted(key for key in env if key.startswith("ARACHNE_GRASP_"))
        keys.append("ARACHNE_CONFIRM_GRASP_EXECUTE_REAL")
        return {key: env.get(key, "") for key in keys}

    def _parse_process_line(self, line: str) -> None:
        stripped = line.strip()
        if not stripped:
            return
        lower = stripped.lower()
        match = re.search(r"Grasp preview logs:\s*(.+)$", stripped)
        if match:
            with self.lock:
                self.grasp_preview_log_dir = match.group(1).strip()
            self._event("grasp_preview_log_dir", {"path": match.group(1).strip()})
        if "sending REAL arm motion" in stripped or "REAL moveJoint" in stripped:
            with self.lock:
                self.runner_real_execution_started = True
                self.runner_waiting_for_execute_trigger = False
        if "REAL gripper command" in stripped:
            with self.lock:
                self.runner_real_execution_started = True
                self.runner_waiting_for_execute_trigger = False
        if "REAL trajectory ready; waiting for execute trigger" in stripped:
            with self.lock:
                self.runner_waiting_for_execute_trigger = True
            self._event("real_execution_waiting_for_trigger", {"line": stripped})
        if "REAL execution trigger received" in stripped:
            with self.lock:
                self.runner_waiting_for_execute_trigger = False
            self._event("real_execution_trigger_received", {"line": stripped})
        if (
            "gripper_capture_failed" in lower
            or "gripper capture failed" in lower
            or "gripper did not capture" in lower
        ):
            with self.lock:
                self.runner_gripper_capture_failed = True
            self._event("gripper_capture_failed", {"line": stripped})
        if (
            "REAL arm SDK moveJoint sequence complete" in stripped
            or "REAL arm trajectory complete" in stripped
            or "REAL arm AuboMoveJoint action sequence complete" in stripped
        ):
            with self.lock:
                self.runner_success_requested = True
            self._event("arm_sequence_complete", {"line": stripped})
            self._terminate_process("real arm sequence complete")
        elif "real execution blocked" in lower:
            with self.lock:
                self.runner_real_execution_blocked = True
            self._event("real_execution_blocked", {"line": stripped})
            self._terminate_process("real execution blocked")
        elif (
            "arm trajectory unavailable" in lower
            or "planning failed" in lower
        ):
            with self.lock:
                real_started = self.runner_real_execution_started
                if not real_started:
                    self.runner_planning_failed = True
            self._event("planning_failed", {"line": stripped, "real_started": real_started})
            if not real_started:
                self._terminate_process("planning failed before real motion")
        elif "REAL execution is armed" in stripped:
            self._event("real_execution_armed", {"line": stripped})
        elif "grasp task failed" in lower or "failed" in lower:
            self._event("runner_warning", {"line": stripped})

    def _finish_task(self, state: str, message: str, *, returncode: int | None) -> None:
        with self.lock:
            self.returncode = returncode
            self.finished_at = datetime.now().isoformat(timespec="seconds")
        if bool(self.get_parameter("set_safety_manual_on_finish").value):
            self._call_trigger(self.set_manual_client, timeout=0.8)
        self._set_state(state, message)
        run_dir = self.run_dir
        if run_dir is not None:
            self._write_json(run_dir / "summary.json", asdict(self._snapshot()))

    def _terminate_process(self, reason: str) -> None:
        with self.lock:
            process = self.process
        if process is None or process.poll() is not None:
            return
        self._event("terminate", {"reason": reason})
        self._terminate_process_group(process, reason, timeout=3.0)

    def _terminate_process_group(
        self, process: subprocess.Popen[str], reason: str, *, timeout: float
    ) -> None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        except Exception as exc:
            self.get_logger().warning(f"SIGINT failed: {exc}")
        deadline = time.monotonic() + max(float(timeout), 0.1)
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            except Exception as exc:
                self.get_logger().warning(f"SIGTERM failed: {exc}")

    def _set_state(self, state: str, message: str) -> None:
        with self.lock:
            self.state = state
            self.message = message
        self._event("state", {"state": state, "message": message})
        self._publish_state()

    def _event(self, kind: str, data: dict[str, Any]) -> None:
        event = {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            "task_id": self.task_id,
            **data,
        }
        text = json.dumps(event, ensure_ascii=False, sort_keys=True)
        msg = String()
        msg.data = text
        try:
            self.event_pub.publish(msg)
        except Exception as exc:
            if "context is invalid" not in str(exc) and "publisher's context is invalid" not in str(exc):
                raise
        run_dir = self.run_dir
        if run_dir is not None:
            try:
                with (run_dir / "events.jsonl").open("a", encoding="utf-8") as handle:
                    handle.write(text + "\n")
            except OSError as exc:
                self.get_logger().warning(f"event log write failed: {exc}")
        if kind == "state":
            self.get_logger().info(f"{data.get('state')}: {data.get('message')}")

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._snapshot_json()
        self.state_pub.publish(msg)

    def _snapshot(self) -> TaskSnapshot:
        with self.lock:
            return TaskSnapshot(
                task_id=self.task_id,
                state=self.state,
                message=self.message,
                run_dir=str(self.run_dir or ""),
                started_at=self.started_at,
                finished_at=self.finished_at,
                returncode=self.returncode,
                grasp_preview_log_dir=self.grasp_preview_log_dir,
                preflight=list(self.preflight_checks),
                base=self._base_snapshot_dict(),
            )

    def _snapshot_json(self) -> str:
        return json.dumps(asdict(self._snapshot()), ensure_ascii=False, sort_keys=True)

    def _write_json(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _short_value(self, value: Any) -> str:
        if isinstance(value, dict):
            return json.dumps(value, ensure_ascii=False, sort_keys=True)[:240]
        text = str(value)
        return text if len(text) <= 240 else text[:237] + "..."


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Arachne grasp task server")
    parser.add_argument(
        "--dry-run-check",
        action="store_true",
        help="validate imports and entrypoint wiring without starting ROS services or hardware I/O",
    )
    args, ros_args = parser.parse_known_args(argv)
    if args.dry_run_check:
        print("grasp_task_server dry-run check ok")
        return

    rclpy.init(args=ros_args)
    node = GraspTaskServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.cancel_event.set()
        node.base_cancel_event.set()
        if rclpy.ok():
            node._publish_base_stop()
        node._terminate_process("shutdown")
        try:
            node._stop_idle_preview("shutdown")
        except Exception:
            pass
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node._stop_idle_preview("shutdown")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
