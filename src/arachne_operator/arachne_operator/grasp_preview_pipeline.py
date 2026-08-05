#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs.msg import JointState
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, ColorRGBA, Empty, Header, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
    from arachne_operator.aubo_move_joint_client import AuboMoveJointClient
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "arachne_operator"))
    from arachne_operator.real_hardware_acceptance_test import AuboI5Kinematics
    from arachne_operator.aubo_move_joint_client import AuboMoveJointClient

try:
    from arachne_hardware.aubo_tcp_driver import AuboDirectJsonRpc
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "arachne_hardware"))
    from arachne_hardware.aubo_tcp_driver import AuboDirectJsonRpc


END_EFFECTOR_FRAME = "grasp_frame"
PATH_PLAYBACK_PERIOD = 8.0
ARM_JOINT_NAMES = tuple(joint.name for joint in AuboI5Kinematics.JOINTS)
REAL_ARM_JOINT_NAMES = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
BASE_JOINT_DEFAULTS = (
    ("front_left_wheel", 0.0),
    ("rear_left_wheel", 0.0),
    ("front_right_wheel", 0.0),
    ("rear_right_wheel", 0.0),
)
GRIPPER_JOINT_DEFAULTS = {
    "ms42dc": (
        ("ms42dc_left_finger_joint", 0.0),
        ("ms42dc_right_finger_joint", 0.0),
    ),
    "ag95": (
        ("left_outer_knuckle_joint", 0.0),
        ("right_outer_knuckle_joint", 0.0),
        ("left_finger_joint", 0.0),
        ("right_finger_joint", 0.0),
        ("left_inner_knuckle_joint", 0.0),
        ("right_inner_knuckle_joint", 0.0),
    ),
}
GRIPPER_PREVIEW_PROFILES = {
    "ms42dc": {
        "names": ("ms42dc_left_finger_joint", "ms42dc_right_finger_joint"),
        "open": (0.0, 0.0),
        "closed": (0.6, -0.6),
    },
    "ag95": {
        "names": (
            "left_outer_knuckle_joint",
            "right_outer_knuckle_joint",
            "left_finger_joint",
            "right_finger_joint",
            "left_inner_knuckle_joint",
            "right_inner_knuckle_joint",
        ),
        "open": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        "closed": (0.93, 0.93, 0.93, 0.93, 0.93, 0.93),
    },
}
DEFAULT_ARM_JOINTS = (
    -1.5707963267949,
    0.201570428261868,
    1.65970467002488,
    0.485178041391533,
    1.67675136677345,
    0.76432946885334,
)
TEACH_ARM_STREAM_RATE_HZ = 80.0
LOCKED_PLAYBACK_RATE = TEACH_ARM_STREAM_RATE_HZ
PREVIEW_IK_TOLERANCE_M = 0.006
PREVIEW_IK_ORIENTATION_TOLERANCE_RAD = 0.01
PREVIEW_IK_DAMPING = 0.08
PREVIEW_IK_MAX_ITERATIONS = 180
PREVIEW_IK_MAX_STEP = 0.05
PREVIEW_IK_ORIENTATION_WEIGHT = 0.5
PREVIEW_MAX_JOINT_SPEED_RAD_SEC = 0.85
PREVIEW_MAX_JOINT_ACCEL_RAD_SEC2 = 4.50
PREVIEW_MAX_JOINT_JERK_RAD_SEC3 = 100.0
PREVIEW_SMOOTHING_TAU_SEC = 0.04
COLLISION_PENALTY = 1_000_000.0
DEFAULT_AUBO_TEACH_FLAG_PATH = "/tmp/arachne_aubo_teach_mode"
DEFAULT_AUBO_CONTROL_OWNER_PATH = "/tmp/arachne_aubo_control_owner"


def _control_owner_payload(owner: str) -> str:
    return json.dumps(
        {"owner": owner, "pid": os.getpid(), "created_at": time.time()},
        separators=(",", ":"),
    ) + "\n"


def _parse_control_owner(text: str) -> tuple[str, int | None]:
    text = text.strip()
    if not text:
        return "", None
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return text.splitlines()[0].strip(), None
    owner = str(data.get("owner", "")).strip()
    pid_value = data.get("pid")
    try:
        pid = int(pid_value) if pid_value is not None else None
    except (TypeError, ValueError):
        pid = None
    return owner, pid


def _pid_alive(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _claim_control_owner(path: Path, owner: str) -> tuple[bool, str]:
    owner = owner.strip() or "grasp_task_server"
    for _attempt in range(2):
        try:
            fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError as exc:
                return False, f"unreadable owner file {path}: {exc}"
            active_owner, pid = _parse_control_owner(text)
            if active_owner == owner and pid == os.getpid():
                return True, f"already owned by {owner}"
            if pid is not None and not _pid_alive(pid):
                try:
                    path.unlink(missing_ok=True)
                except OSError as exc:
                    return False, f"stale owner {active_owner or text!r} could not be cleared: {exc}"
                continue
            pid_text = str(pid) if pid is not None else "unknown"
            return False, f"owned by {active_owner or text.strip() or 'unknown'} pid={pid_text}"
        except OSError as exc:
            return False, f"could not create owner file {path}: {exc}"

        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                handle.write(_control_owner_payload(owner))
        except OSError as exc:
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass
            return False, f"could not write owner file {path}: {exc}"
        return True, f"owned by {owner}"
    return False, f"could not claim owner file {path}"


def _release_control_owner(path: Path, owner: str) -> None:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return
    except OSError:
        return
    active_owner, pid = _parse_control_owner(text)
    if active_owner != (owner.strip() or "grasp_task_server"):
        return
    if pid is not None and pid != os.getpid():
        return
    try:
        path.unlink(missing_ok=True)
    except OSError:
        return


@dataclass
class Detection:
    label: str
    class_id: int
    confidence: float
    xyxy: tuple[float, float, float, float]
    mask_xy: tuple[tuple[float, float], ...] | None = None
    mask_area_px: float = 0.0


@dataclass(frozen=True)
class CartesianSegment:
    kind: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    control: tuple[float, float, float] | None = None
    duration: float = 0.0


@dataclass(frozen=True)
class JointTrajectoryFrame:
    time_from_start: float
    positions: tuple[float, float, float, float, float, float]
    velocities: tuple[float, float, float, float, float, float]
    accelerations: tuple[float, float, float, float, float, float]


@dataclass
class MoveItPlanAttempt:
    response: object | None
    message: str


@dataclass(frozen=True)
class CollisionBox:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class PointCloudGraspShape:
    center_base: tuple[float, float, float]
    major_axis_base: tuple[float, float, float]
    minor_axis_base: tuple[float, float, float]
    visual_axis_base: tuple[float, float, float] | None
    extent_major_m: float
    extent_minor_m: float
    extent_z_m: float
    axis_confidence: float
    visual_axis_confidence: float
    point_count: int
    z_bias_m: float


@dataclass
class GraspPreview:
    detection: Detection
    depth_m: float
    grasp_xyz: tuple[float, float, float]
    pregrasp_xyz: tuple[float, float, float]
    lift_xyz: tuple[float, float, float]
    retreat_xyz: tuple[float, float, float]
    roi_points: list[tuple[float, float, float]]
    roi_source: str
    bbox_points: list[tuple[float, float, float]]
    base_path_xyz: list[tuple[float, float, float]]
    base_trajectory_segments: list[CartesianSegment]
    base_waypoints: list[tuple[str, tuple[float, float, float]]]
    arm_trajectory_frames: list[JointTrajectoryFrame]
    basket_safe: bool
    base_grasp_xyz: tuple[float, float, float] | None
    pointcloud_shape: PointCloudGraspShape | None
    snapshot_reason: str
    ik_message: str


@dataclass
class RealSdkWarmSession:
    generation: int
    rpc: AuboDirectJsonRpc
    owner_owned: bool
    gate_owned: bool
    started_at: float
    ready_at: float
    ip: str
    port: int


def _add_venv_site_packages(venv: Path) -> None:
    pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        venv / "lib" / pyver / "site-packages",
        venv / "lib64" / pyver / "site-packages",
    ]
    for candidate in candidates:
        if candidate.exists():
            sys.path.insert(0, str(candidate))


def _load_yolo(venv: Path, model_path: Path, task: str, initial_device: str = ""):
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    model_path = model_path.expanduser()
    if not model_path.is_absolute():
        model_path = Path.cwd() / model_path
    if not model_path.is_file():
        if os.environ.get("ARACHNE_GRASP_ALLOW_MODEL_DOWNLOAD", "false").lower() != "true":
            raise FileNotFoundError(
                f"YOLO model file not found: {model_path}. "
                "Refusing to let Ultralytics auto-download official weights; "
                "set ARACHNE_GRASP_YOLO_MODEL to a local trained weight, "
                "or set ARACHNE_GRASP_ALLOW_MODEL_DOWNLOAD=true intentionally."
            )
    _add_venv_site_packages(venv)
    from ultralytics import YOLO

    kwargs = {"task": task} if task else {}
    model = YOLO(str(model_path), **kwargs)
    if initial_device:
        model.overrides["device"] = initial_device
    return model


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview detect-depth-grasp path from Gemini335 RGB-D in RViz."
    )
    hidden = argparse.SUPPRESS
    parser.add_argument("--model", default="yolo_workspace/weights/yolo26n.onnx", help=hidden)
    parser.add_argument("--venv", default="yolo_workspace/.venv", help=hidden)
    parser.add_argument("--yolo-task", default="detect", help=hidden)
    parser.add_argument("--classes", default="")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--min-detection-mask-area-px", type=float, default=1200.0, help=hidden)
    parser.add_argument("--detection-edge-margin-ratio", type=float, default=0.06, help=hidden)
    parser.add_argument("--detection-min-center-y-ratio", type=float, default=0.38, help=hidden)
    parser.add_argument("--detection-max-center-y-ratio", type=float, default=0.86, help=hidden)
    parser.add_argument("--reject-label-keywords", default="", help=hidden)
    parser.add_argument(
        "--preferred-label-keywords",
        default="bottle,carton,can,cup,container,jar,box",
        help=hidden,
    )
    parser.add_argument("--imgsz", type=int, default=640, help=hidden)
    parser.add_argument("--device-id", default="0", help=hidden)
    parser.add_argument(
        "--onnx-device",
        default=os.environ.get("ARACHNE_GRASP_ONNX_DEVICE", "cpu"),
        help=hidden,
    )
    parser.add_argument("--gripper-type", choices=("ms42dc", "ag95"), default="ms42dc", help=hidden)
    parser.add_argument("--inference-period", type=float, default=0.45, help=hidden)
    parser.add_argument("--max-color-frame-age-sec", type=float, default=0.75, help=hidden)
    parser.add_argument("--snapshot-iou-threshold", type=float, default=0.35, help=hidden)
    parser.add_argument("--snapshot-center-shift", type=float, default=0.12, help=hidden)
    parser.add_argument("--lost-frame-threshold", type=int, default=5, help=hidden)
    parser.add_argument("--locked-visual-rate", type=float, default=10.0, help=hidden)
    parser.add_argument("--restart-search-topic", default="/arachne/grasp_preview/restart_search")
    parser.add_argument(
        "--real-search-scan-control-topic",
        default="/arachne/grasp_preview/real_search_scan",
        help=hidden,
    )
    parser.add_argument("--detection-topic", default="/arachne/perception/taco_instances", help=hidden)
    parser.add_argument("--joint-states-topic", default="/arachne/display/joint_states", help=hidden)
    parser.add_argument("--color-topic", default="/camera/color/image_raw", help=hidden)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw", help=hidden)
    parser.add_argument("--color-info-topic", default="/camera/color/camera_info", help=hidden)
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info", help=hidden)
    parser.add_argument("--depth-scale", type=float, default=0.001, help=hidden)
    parser.add_argument("--min-depth", type=float, default=0.12, help=hidden)
    parser.add_argument("--max-depth", type=float, default=2.5, help=hidden)
    parser.add_argument("--depth-percentile", type=float, default=50.0, help=hidden)
    parser.add_argument("--depth-band", type=float, default=0.08, help=hidden)
    parser.add_argument("--yolo-depth-filter", action=argparse.BooleanOptionalAction, default=False, help=hidden)
    parser.add_argument("--yolo-depth-min-base-z", type=float, default=-0.20, help=hidden)
    parser.add_argument("--yolo-depth-max-base-z", type=float, default=0.0, help=hidden)
    parser.add_argument("--roi-shrink", type=float, default=0.65, help=hidden)
    parser.add_argument("--roi-decimation", type=int, default=3, help=hidden)
    parser.add_argument(
        "--depth-projection-flip-x",
        dest="depth_projection_flip_x",
        action="store_true",
        default=False,
        help=hidden,
    )
    parser.add_argument(
        "--no-depth-projection-flip-x",
        dest="depth_projection_flip_x",
        action="store_false",
        help=hidden,
    )
    parser.add_argument(
        "--depth-projection-flip-y",
        dest="depth_projection_flip_y",
        action="store_true",
        default=False,
        help=hidden,
    )
    parser.add_argument(
        "--no-depth-projection-flip-y",
        dest="depth_projection_flip_y",
        action="store_false",
        help=hidden,
    )
    parser.add_argument("--approach-distance", type=float, default=0.18, help=hidden)
    parser.add_argument("--grasp-standoff", type=float, default=0.035, help=hidden)
    parser.add_argument("--grasp-tcp-offset-m", type=float, default=0.0, help=hidden)
    parser.add_argument("--grasp-base-offset", default="0,0,0", help=hidden)
    parser.add_argument("--disable-pointcloud-grasp-shape", action="store_true", help=hidden)
    parser.add_argument("--vertical-approach", action=argparse.BooleanOptionalAction, default=True, help=hidden)
    parser.add_argument(
        "--lock-grasp-orientation",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=hidden,
    )
    parser.add_argument("--pointcloud-grasp-min-points", type=int, default=24, help=hidden)
    parser.add_argument("--pointcloud-axis-confidence-threshold", type=float, default=0.10, help=hidden)
    parser.add_argument("--visual-axis-confidence-threshold", type=float, default=0.12, help=hidden)
    parser.add_argument("--pointcloud-visible-upper-half-z-bias-ratio", type=float, default=0.65, help=hidden)
    parser.add_argument("--pointcloud-visible-upper-half-z-bias-max", type=float, default=0.030, help=hidden)
    parser.add_argument("--lift-distance", type=float, default=0.10, help=hidden)
    parser.add_argument("--base-frame", default="base_link", help=hidden)
    parser.add_argument("--aubo-base-frame", default="grasp_preview_aubo_base_link", help=hidden)
    parser.add_argument("--basket-release-base", default="0.545,0.0,0.18", help=hidden)
    parser.add_argument("--basket-approach-base", default="0.545,0.0,0.34", help=hidden)
    parser.add_argument("--basket-keepout-min-base", default="0.4215,-0.11,-0.1235", help=hidden)
    parser.add_argument("--basket-keepout-max-base", default="0.6655,0.11,0.0635", help=hidden)
    parser.add_argument("--rear-rack-keepout-min-base", default="-0.41,-0.22,0.04", help=hidden)
    parser.add_argument("--rear-rack-keepout-max-base", default="0.09,0.22,0.82", help=hidden)
    parser.add_argument("--basket-clearance", type=float, default=0.04, help=hidden)
    parser.add_argument("--gripper-radius", type=float, default=0.055, help=hidden)
    parser.add_argument("--arm-collision-radius", type=float, default=0.035, help=hidden)
    parser.add_argument("--arm-collision-samples-per-link", type=int, default=6, help=hidden)
    parser.add_argument("--collision-margin", type=float, default=0.015, help=hidden)
    parser.add_argument("--rear-rack-collision-margin", type=float, default=0.005, help=hidden)
    parser.add_argument("--ground-min-z-base", type=float, default=-0.22, help=hidden)
    parser.add_argument("--grasp-max-z-base", type=float, default=999.0, help=hidden)
    parser.add_argument("--min-grasp-z-base", type=float, default=-0.135, help=hidden)
    parser.add_argument("--grasp-final-z-offset-m", type=float, default=-0.055, help=hidden)
    parser.add_argument("--fixed-grasp-z-base", type=float, default=float("nan"), help=hidden)
    parser.add_argument("--ground-clearance", type=float, default=0.02, help=hidden)
    parser.add_argument("--tool-ground-clearance", type=float, default=0.015, help=hidden)
    parser.add_argument("--allow-colliding-best-effort", action="store_true", help=hidden)
    parser.add_argument("--release-tool-tilt-deg", type=float, default=12.0, help=hidden)
    parser.add_argument("--transit-height", type=float, default=0.36, help=hidden)
    parser.add_argument("--safe-mid-lift-height", type=float, default=0.50, help=hidden)
    parser.add_argument("--transit-arc-height", type=float, default=0.16, help=hidden)
    parser.add_argument("--arc-samples", type=int, default=24, help=hidden)
    parser.add_argument("--line-samples", type=int, default=6, help=hidden)
    parser.add_argument("--basket-descent-samples", type=int, default=8, help=hidden)
    parser.add_argument("--playback-period", type=float, default=0.0, help=hidden)
    parser.add_argument("--playback-rate", type=float, default=LOCKED_PLAYBACK_RATE, help=hidden)
    parser.add_argument("--trajectory-cartesian-step", type=float, default=0.025, help=hidden)
    parser.add_argument("--trajectory-joint-tolerance", type=float, default=0.004, help=hidden)
    parser.add_argument("--trajectory-max-duration", type=float, default=90.0, help=hidden)
    parser.add_argument("--planning-key-waypoints", default="approach,grasp,safe_mid,basket_over", help=hidden)
    parser.add_argument("--planner-backend", choices=("moveit", "remote", "local", "none"), default="local", help=hidden)
    parser.add_argument(
        "--remote-planner-url",
        default=os.environ.get("ARACHNE_REMOTE_PLANNER_URL", "http://127.0.0.1:8767"),
        help=hidden,
    )
    parser.add_argument("--remote-planner-timeout", type=float, default=2.0, help=hidden)
    parser.add_argument("--moveit-plan-service", default="/plan_kinematic_path", help=hidden)
    parser.add_argument(
        "--moveit-planners",
        default="RRTConnectkConfigDefault",
        help=hidden,
    )
    parser.add_argument("--moveit-planning-time", type=float, default=0.5, help=hidden)
    parser.add_argument("--moveit-planning-attempts", type=int, default=1, help=hidden)
    parser.add_argument("--moveit-max-goal-waypoints", type=int, default=3, help=hidden)
    parser.add_argument("--moveit-position-tolerance", type=float, default=0.015, help=hidden)
    parser.add_argument("--moveit-orientation-tolerance", type=float, default=1.20, help=hidden)
    parser.add_argument("--moveit-release-orientation-tolerance", type=float, default=0.55, help=hidden)
    parser.add_argument("--moveit-joint-goal-tolerance", type=float, default=0.025, help=hidden)
    parser.add_argument("--moveit-ik-orientation-tolerance", type=float, default=0.35, help=hidden)
    parser.add_argument("--moveit-ik-max-iterations", type=int, default=90, help=hidden)
    parser.add_argument("--moveit-service-timeout-padding", type=float, default=0.25, help=hidden)
    parser.add_argument("--moveit-soft-waypoint-position-tolerance", type=float, default=0.08, help=hidden)
    parser.add_argument("--moveit-soft-waypoint-orientation-tolerance", type=float, default=0.50, help=hidden)
    parser.add_argument("--moveit-local-fallback-position-tolerance", type=float, default=0.04, help=hidden)
    parser.add_argument("--moveit-local-fallback-orientation-tolerance", type=float, default=0.50, help=hidden)
    parser.add_argument("--moveit-grasp-fallback-position-tolerance", type=float, default=0.025, help=hidden)
    parser.add_argument("--moveit-grasp-fallback-orientation-tolerance", type=float, default=0.65, help=hidden)
    parser.add_argument("--release-shoulder-elbow-motion-weight", type=float, default=4.0, help=hidden)
    parser.add_argument("--release-wrist-motion-weight", type=float, default=0.35, help=hidden)
    parser.add_argument("--release-configuration-weight", type=float, default=4.0, help=hidden)
    parser.add_argument("--moveit-pose-goal-on-ik-failure", action=argparse.BooleanOptionalAction, default=True, help=hidden)
    parser.add_argument(
        "--local-strict-ik",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=hidden,
    )
    parser.add_argument("--local-position-tolerance", type=float, default=0.035, help=hidden)
    parser.add_argument("--local-orientation-tolerance", type=float, default=0.35, help=hidden)
    parser.add_argument("--local-planning-timeout-sec", type=float, default=0.0, help=hidden)
    parser.add_argument("--local-ik-max-iterations", type=int, default=60, help=hidden)
    parser.add_argument(
        "--moveit-local-first",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=hidden,
    )
    parser.add_argument("--gripper-close-progress-start", type=float, default=0.30, help=hidden)
    parser.add_argument("--gripper-close-progress-end", type=float, default=0.42, help=hidden)
    parser.add_argument("--gripper-open-progress-start", type=float, default=0.84, help=hidden)
    parser.add_argument("--gripper-open-progress-end", type=float, default=0.94, help=hidden)
    parser.add_argument("--gripper-close-waypoint", default="grasp", help=hidden)
    parser.add_argument("--gripper-open-waypoint", default="basket_over", help=hidden)
    parser.add_argument("--gripper-transition-progress", type=float, default=0.025, help=hidden)
    parser.add_argument("--execute-real", action="store_true", help=hidden)
    parser.add_argument(
        "--execute-real-confirm",
        default=os.environ.get("ARACHNE_CONFIRM_GRASP_EXECUTE_REAL", ""),
        help=hidden,
    )
    parser.add_argument(
        "--real-execute-backend",
        choices=("sdk_move_joint", "follow_joint_trajectory"),
        default="sdk_move_joint",
        help=hidden,
    )
    parser.add_argument(
        "--real-follow-action",
        default="/joint_trajectory_controller/follow_joint_trajectory",
        help=hidden,
    )
    parser.add_argument("--real-joint-states-topic", default="/arachne/teach_visualization/joint_states", help=hidden)
    parser.add_argument(
        "--real-arm-joint-names",
        default=",".join(REAL_ARM_JOINT_NAMES),
        help=hidden,
    )
    parser.add_argument(
        "--real-start-joints",
        default=os.environ.get("ARACHNE_GRASP_ARM_JOINTS", ""),
        help=hidden,
    )
    parser.add_argument("--real-start-tolerance-rad", type=float, default=0.08, help=hidden)
    parser.add_argument("--real-start-state-timeout", type=float, default=3.0, help=hidden)
    parser.add_argument("--real-goal-time-margin", type=float, default=2.0, help=hidden)
    parser.add_argument("--real-action-timeout-padding", type=float, default=8.0, help=hidden)
    parser.add_argument("--real-trajectory-point-stride", type=int, default=4, help=hidden)
    parser.add_argument("--real-allow-partial", action="store_true", help=hidden)
    parser.add_argument("--real-sdk-ip", default=os.environ.get("AUBO_ROBOT_IP", "192.168.127.128"), help=hidden)
    parser.add_argument("--real-sdk-rpc-port", type=int, default=30004, help=hidden)
    parser.add_argument("--real-sdk-rpc-timeout", type=float, default=2.0, help=hidden)
    parser.add_argument("--real-sdk-prewarm", action=argparse.BooleanOptionalAction, default=True, help=hidden)
    parser.add_argument("--real-sdk-prewarm-wait-sec", type=float, default=0.25, help=hidden)
    parser.add_argument("--real-sdk-teach-flag-path", default=DEFAULT_AUBO_TEACH_FLAG_PATH, help=hidden)
    parser.add_argument(
        "--real-sdk-control-owner-path",
        default=DEFAULT_AUBO_CONTROL_OWNER_PATH,
        help=hidden,
    )
    parser.add_argument(
        "--real-sdk-control-owner-name",
        default="grasp_task_server",
        help=hidden,
    )
    parser.add_argument("--real-sdk-gate-settle-sec", type=float, default=0.12, help=hidden)
    parser.add_argument("--real-sdk-move-speed", type=float, default=0.25, help=hidden)
    parser.add_argument("--real-sdk-move-accel", type=float, default=0.45, help=hidden)
    parser.add_argument("--real-sdk-blend-radius", type=float, default=0.0, help=hidden)
    parser.add_argument("--real-sdk-segment-duration-scale", type=float, default=0.0, help=hidden)
    parser.add_argument("--real-sdk-min-segment-duration", type=float, default=0.35, help=hidden)
    parser.add_argument("--real-sdk-goal-tolerance-rad", type=float, default=0.045, help=hidden)
    parser.add_argument("--real-sdk-arrival-timeout-padding", type=float, default=5.0, help=hidden)
    parser.add_argument("--real-sdk-max-segment-joint-delta", type=float, default=0.55, help=hidden)
    parser.add_argument("--real-sdk-max-targets", type=int, default=8, help=hidden)
    parser.add_argument("--real-sdk-semantic-targets-only", action=argparse.BooleanOptionalAction, default=True, help=hidden)
    parser.add_argument("--real-search-scan", action=argparse.BooleanOptionalAction, default=False, help=hidden)
    parser.add_argument("--real-search-scan-period-sec", type=float, default=5.0, help=hidden)
    parser.add_argument("--real-search-scan-shoulder-deg", type=float, default=4.0, help=hidden)
    parser.add_argument("--real-search-scan-wrist3-deg", type=float, default=8.0, help=hidden)
    parser.add_argument("--real-search-scan-left-delta", default="0.42,0.04,0.04,-0.06,-0.02,0.34", help=hidden)
    parser.add_argument("--real-search-scan-right-delta", default="-0.08,-0.02,-0.04,0.08,0.02,-0.10", help=hidden)
    parser.add_argument("--real-search-scan-speed", type=float, default=0.08, help=hidden)
    parser.add_argument("--real-search-scan-accel", type=float, default=0.20, help=hidden)
    parser.add_argument("--perception-only-restart-sec", type=float, default=1.0, help=hidden)
    parser.add_argument("--aubo-move-joint-action-name", default="/arachne/aubo/move_joint", help=hidden)
    parser.add_argument(
        "--prefer-aubo-move-joint-action",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--aubo-move-joint-fallback-internal",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=hidden,
    )
    parser.add_argument("--aubo-move-joint-wait-server-sec", type=float, default=0.5, help=hidden)
    parser.add_argument(
        "--real-return-home",
        dest="real_return_home",
        action="store_true",
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--no-real-return-home",
        dest="real_return_home",
        action="store_false",
        help=hidden,
    )
    parser.add_argument(
        "--real-home-joints",
        default=os.environ.get(
            "ARACHNE_AUBO_HOME_JOINTS_RAD",
            ",".join(str(value) for value in DEFAULT_ARM_JOINTS),
        ),
        help=hidden,
    )
    parser.add_argument("--real-home-duration", type=float, default=2.5, help=hidden)
    parser.add_argument("--real-fixed-post-grasp", action="store_true", help=hidden)
    parser.add_argument("--real-fixed-lift-joints", default="", help=hidden)
    parser.add_argument("--real-fixed-basket-joints", default="", help=hidden)
    parser.add_argument("--real-fixed-search-joints", default="", help=hidden)
    parser.add_argument("--real-fixed-post-grasp-duration", type=float, default=1.2, help=hidden)
    parser.add_argument(
        "--real-execute-gripper",
        action="store_true",
        default=True,
        help=hidden,
    )
    parser.add_argument(
        "--no-real-execute-gripper",
        dest="real_execute_gripper",
        action="store_false",
        help=hidden,
    )
    parser.add_argument("--real-gripper-command-topic", default="/arachne/gripper/command", help=hidden)
    parser.add_argument("--real-execute-trigger-topic", default="", help=hidden)
    parser.add_argument("--real-execute-trigger-service", default="/arachne/grasp_preview/execute_real_now", help=hidden)
    parser.add_argument("--real-gripper-settle-sec", type=float, default=2.0, help=hidden)
    parser.add_argument("--real-preopen-gripper", action=argparse.BooleanOptionalAction, default=False, help=hidden)
    parser.add_argument("--real-gripper-status-topic", default="/arachne/hardware/gripper_status", help=hidden)
    parser.add_argument(
        "--real-gripper-require-capture",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=hidden,
    )
    parser.add_argument("--real-gripper-capture-timeout", type=float, default=1.2, help=hidden)
    parser.add_argument("--real-gripper-capture-min-angle", type=float, default=450.0, help=hidden)
    parser.add_argument("--real-gripper-empty-close-angle", type=float, default=350.0, help=hidden)
    parser.add_argument(
        "--real-gripper-capture-unknown-ok",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=hidden,
    )
    parser.add_argument("--tool-orientation-limit-deg", type=float, default=35.0, help=hidden)
    parser.add_argument("--grasp-topdown-max-tilt-deg", type=float, default=65.0, help=hidden)
    parser.add_argument("--topdown-flange-rpy-rad", default="3.14,0.03,-1.063", help=hidden)
    parser.add_argument("--max-grasp-orientation-candidates", type=int, default=8, help=hidden)
    parser.add_argument("--grasp-orientation-yaw-offsets-deg", default="0,20,-20,35,-35", help=hidden)
    parser.add_argument("--transit-orientation-yaw-offsets-deg", default="0,25,-25,35,-35", help=hidden)
    parser.add_argument("--grasp-orientation-tilt-offsets-deg", default="0,15,-15", help=hidden)
    parser.add_argument("--moveit-use-orientation-path-constraint", action="store_true", help=hidden)
    parser.add_argument("--moveit-max-tool0-reach", type=float, default=1.03, help=hidden)
    parser.add_argument("--moveit-velocity-scale", type=float, default=0.50, help=hidden)
    parser.add_argument("--moveit-accel-scale", type=float, default=0.80, help=hidden)
    parser.add_argument("--loop-playback", action="store_true", help=hidden)
    parser.add_argument(
        "--preview-max-joint-speed",
        type=float,
        default=PREVIEW_MAX_JOINT_SPEED_RAD_SEC,
        help=hidden,
    )
    parser.add_argument(
        "--preview-max-joint-accel",
        type=float,
        default=PREVIEW_MAX_JOINT_ACCEL_RAD_SEC2,
        help=hidden,
    )
    parser.add_argument(
        "--preview-max-joint-jerk",
        type=float,
        default=PREVIEW_MAX_JOINT_JERK_RAD_SEC3,
        help=hidden,
    )
    parser.add_argument(
        "--preview-smoothing-tau",
        type=float,
        default=PREVIEW_SMOOTHING_TAU_SEC,
        help=hidden,
    )
    parser.add_argument("--save-dir", default="yolo_workspace/runs/grasp_preview", help=hidden)
    return parser.parse_args()


def _class_ids(model, spec: str) -> list[int] | None:
    tokens = [token.strip() for token in spec.split(",") if token.strip()]
    if not tokens:
        return None
    names = getattr(model, "names", {}) or {}
    name_to_id = {str(name).lower(): int(idx) for idx, name in names.items()}
    ids: list[int] = []
    for token in tokens:
        if token.isdigit():
            ids.append(int(token))
            continue
        key = token.lower()
        if key not in name_to_id:
            raise ValueError(f"unknown YOLO class {token!r}; known classes: {list(names.values())}")
        ids.append(name_to_id[key])
    return ids


def _xyz(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.replace(" ", ",").split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError(f"expected xyz triplet, got {value!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def _float_values(value: str, expected: int, label: str) -> list[float]:
    parts = [
        part.strip()
        for part in str(value).replace(";", ",").replace(" ", ",").split(",")
        if part.strip()
    ]
    if len(parts) != expected:
        raise ValueError(f"{label} must contain {expected} values, got {len(parts)}")
    return [float(part) for part in parts]


def _float_list(value: str, label: str) -> list[float]:
    parts = [
        part.strip()
        for part in str(value).replace(";", ",").replace(" ", ",").split(",")
        if part.strip()
    ]
    if not parts:
        raise ValueError(f"{label} must contain at least one value")
    return [float(part) for part in parts]


def _header(stamp, frame_id: str) -> Header:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def _point(xyz: Iterable[float]) -> Point:
    x, y, z = xyz
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def _image_to_bgr(msg: Image) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8", "mono8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    channels = 1 if msg.encoding == "mono8" else 3
    row_pixels = msg.step // channels
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if channels == 1:
        image = data.reshape(msg.height, msg.step)[:, : msg.width].copy()
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image = data.reshape(msg.height, row_pixels, channels)[:, : msg.width, :].copy()
    if msg.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def _image_to_depth(msg: Image) -> np.ndarray:
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    row_pixels = msg.step // 2
    data = np.frombuffer(msg.data, dtype=np.uint16)
    return data.reshape(msg.height, row_pixels)[:, : msg.width].copy()


def _image_from_bgr(image: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    contiguous = np.ascontiguousarray(image)
    msg.step = int(contiguous.shape[1] * 3)
    msg.data = contiguous.tobytes()
    return msg


class GraspPreviewNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("grasp_preview_pipeline")
        self.args = args
        self.model_path = Path(args.model)
        self.venv = Path(args.venv)
        self.save_dir = Path(args.save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        initial_device = (
            str(args.onnx_device).strip()
            if self.model_path.suffix.lower() == ".onnx"
            else str(args.device_id)
        )
        self.model = _load_yolo(
            self.venv, self.model_path, str(args.yolo_task), initial_device
        )
        self.class_ids = _class_ids(self.model, args.classes)
        self.base_frame = str(args.base_frame)
        self.aubo_base_frame = str(args.aubo_base_frame)
        self.grasp_base_offset = _xyz(args.grasp_base_offset)
        self.basket_release_base = _xyz(args.basket_release_base)
        self.basket_approach_base = _xyz(args.basket_approach_base)
        self.basket_keepout_min = _xyz(args.basket_keepout_min_base)
        self.basket_keepout_max = _xyz(args.basket_keepout_max_base)
        self.rear_rack_keepout_min = _xyz(args.rear_rack_keepout_min_base)
        self.rear_rack_keepout_max = _xyz(args.rear_rack_keepout_max_base)
        self._clamp_basket_points_above_keepout()
        self.collision_boxes = self._make_collision_boxes()
        self.kinematics = AuboI5Kinematics()

        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.color_info: CameraInfo | None = None
        self.depth_info: CameraInfo | None = None
        self.current_arm_joints = np.asarray(DEFAULT_ARM_JOINTS, dtype=float)
        self.current_joint_positions: dict[str, float] = {}
        self.real_joint_positions: dict[str, float] = {}
        self.real_joint_state_time = 0.0
        self.real_execution_started = False
        self.real_execution_requested = not str(args.real_execute_trigger_topic).strip()
        self.real_execution_lock = threading.Lock()
        self.real_execution_thread: threading.Thread | None = None
        self.real_sdk_warm_lock = threading.Lock()
        self.real_sdk_warm_generation = 0
        self.real_sdk_warm_session: RealSdkWarmSession | None = None
        self.real_sdk_warm_thread: threading.Thread | None = None
        self.real_gripper_preopened = False
        self.real_gripper_preopen_time = 0.0
        self.last_restart_search_time = 0.0
        self.last_real_execute_request_time = 0.0
        self.real_scan_thread: threading.Thread | None = None
        self._last_real_scan_warning = ""
        self._last_real_scan_warning_time = 0.0
        self.real_search_scan_enabled = bool(args.real_search_scan)
        self.preview_ik_joints = np.asarray(DEFAULT_ARM_JOINTS, dtype=float)
        self.preview_ik_velocity = np.zeros(6, dtype=float)
        self.preview_ik_accel = np.zeros(6, dtype=float)
        self.preview_ik_last_update = time.monotonic()
        self.preview_ik_last_progress = 0.0
        self.preview_target_rotation = self.kinematics.fk(self.preview_ik_joints)[:3, :3]
        self.preview_ik_message = "waiting for plan"
        self.last_preview: GraspPreview | None = None
        self.depth_wait_detection: Detection | None = None
        self.depth_wait_reason = ""
        self.depth_wait_header: Header | None = None
        self.depth_wait_last_publish = 0.0
        self.snapshot_detection: Detection | None = None
        self.snapshot_header: Header | None = None
        self.snapshot_time = 0.0
        self.snapshot_count = 0
        self.last_saved_detection_key: tuple[int, str] | None = None
        self.inference_count = 0
        self.inference_paused = False
        self.planning_thread: threading.Thread | None = None
        self.planning_generation = 0
        self.planning_lock = threading.Lock()
        self.preview_output_lock = threading.Lock()
        self.stopping = False
        self.plan_lock_time = 0.0
        self.tool_to_grasp_matrix: np.ndarray | None = None
        self.tool_to_grasp_frames: tuple[str, str] | None = None
        self.gripper_status_lock = threading.Lock()
        self.latest_real_gripper_status: tuple[float, str] | None = None
        self.locked_visual_last_publish = 0.0
        self.missing_frames = 0
        self.last_log_time = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, args.color_topic, self._color_cb, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.color_info_topic, self._color_info_cb, 10)
        self.create_subscription(CameraInfo, args.depth_info_topic, self._depth_info_cb, 10)
        self.create_subscription(Empty, args.restart_search_topic, self._restart_search_cb, 10)
        self.create_subscription(
            Bool,
            str(args.real_search_scan_control_topic),
            self._real_search_scan_control_cb,
            10,
        )
        self.create_subscription(JointState, args.joint_states_topic, self._joint_state_cb, 10)
        if bool(args.execute_real):
            self.create_service(Trigger, str(args.real_execute_trigger_service), self._real_execute_trigger_srv)
            if str(args.real_execute_trigger_topic).strip():
                self.create_subscription(
                    Empty,
                    str(args.real_execute_trigger_topic),
                    self._real_execute_trigger_cb,
                    10,
                )
            self.create_subscription(
                JointState,
                args.real_joint_states_topic,
                self._real_joint_state_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                args.real_gripper_status_topic,
                self._real_gripper_status_cb,
                10,
            )
        self.moveit_plan_client = (
            self.create_client(GetMotionPlan, str(args.moveit_plan_service))
            if str(args.planner_backend) == "moveit"
            else None
        )
        self.real_arm_action_client = (
            ActionClient(self, FollowJointTrajectory, str(args.real_follow_action))
            if bool(args.execute_real)
            and str(args.real_execute_backend) == "follow_joint_trajectory"
            else None
        )
        self.aubo_move_joint_client = (
            AuboMoveJointClient(self, str(args.aubo_move_joint_action_name))
            if bool(args.execute_real)
            and str(args.real_execute_backend) == "sdk_move_joint"
            and bool(args.prefer_aubo_move_joint_action)
            else None
        )
        self.real_gripper_pub = (
            self.create_publisher(String, str(args.real_gripper_command_topic), 10)
            if bool(args.execute_real)
            else None
        )

        self.markers_pub = self.create_publisher(
            MarkerArray, "/arachne/grasp_preview/markers", 10
        )
        self.cloud_pub = self.create_publisher(PointCloud2, "/arachne/grasp_preview/roi_cloud", 10)
        self.path_pub = self.create_publisher(PathMsg, "/arachne/grasp_preview/path", 10)
        self.image_pub = self.create_publisher(
            Image, "/arachne/grasp_preview/annotated_image", 10
        )
        self.detection_pub = self.create_publisher(String, str(args.detection_topic), 10)
        self.arm_preview_pub = self.create_publisher(
            JointState, "/arachne/grasp_preview/joint_states", 10
        )

        period = max(float(args.inference_period), 0.05)
        playback_rate = max(float(args.playback_rate), 1.0)
        self.create_timer(period, self._tick)
        self.create_timer(1.0 / playback_rate, self._playback_tick)
        self.get_logger().info(
            "grasp preview ready: "
            f"model={self.model_path} classes={args.classes or 'all'} "
            f"inference_device={self._yolo_device()} "
            f"planner={args.planner_backend} vertical_approach={bool(args.vertical_approach)} "
            f"lock_orientation={bool(args.lock_grasp_orientation)} "
            "markers=/arachne/grasp_preview/markers "
            "roi_cloud=/arachne/grasp_preview/roi_cloud "
            f"detections={args.detection_topic} "
            f"restart_topic={args.restart_search_topic}"
        )
        if bool(args.execute_real):
            self.get_logger().warn(
                "real execution armed; motion will be sent only after confirmation and start-state checks "
                f"backend={args.real_execute_backend}"
            )
        self.real_scan_thread = threading.Thread(target=self._real_search_scan_loop, daemon=True)
        self.real_scan_thread.start()

    def _clamp_basket_points_above_keepout(self) -> None:
        top = (
            self.basket_keepout_max[2]
            + max(float(self.args.gripper_radius), 0.0)
            + float(self.args.basket_clearance)
        )
        approach_top = top + 0.12
        rx, ry, rz = self.basket_release_base
        ax, ay, az = self.basket_approach_base
        self.basket_release_base = (rx, ry, max(rz, top))
        self.basket_approach_base = (ax, ay, max(az, approach_top))

    def _make_collision_boxes(self) -> list[CollisionBox]:
        margin = max(float(self.args.collision_margin), 0.0)

        def padded(
            size: tuple[float, float, float], margin_m: float = margin
        ) -> tuple[float, float, float]:
            pad = max(float(margin_m), 0.0)
            return tuple(float(value) + 2.0 * pad for value in size)  # type: ignore[return-value]

        def box_from_min_max(
            name: str,
            minimum: tuple[float, float, float],
            maximum: tuple[float, float, float],
            margin_m: float = margin,
        ) -> CollisionBox | None:
            mins = np.asarray(minimum, dtype=np.float64)
            maxs = np.asarray(maximum, dtype=np.float64)
            if bool(np.any(maxs <= mins)):
                return None
            center = tuple(float(value) for value in (mins + maxs) * 0.5)
            size = tuple(float(value) for value in (maxs - mins))
            return CollisionBox(name, center, padded(size, margin_m))

        basket_box = box_from_min_max(
            "front_basket_keepout", self.basket_keepout_min, self.basket_keepout_max
        )
        rear_box = box_from_min_max(
            "rear_sensor_rack",
            self.rear_rack_keepout_min,
            self.rear_rack_keepout_max,
            max(float(self.args.rear_rack_collision_margin), 0.0),
        )
        boxes = [
            CollisionBox("scout_base_main", (0.0, 0.0, 0.008), padded((0.925, 0.380, 0.210))),
            CollisionBox(
                "scout_base_center_ridge",
                (0.0, 0.0, 0.210 / 6.0),
                padded((0.925 / 6.0, 0.380 * 1.65, 0.210 / 3.0)),
            ),
        ]
        if basket_box is not None:
            boxes.append(basket_box)
        if rear_box is not None:
            boxes.append(rear_box)
        return boxes

    def _color_cb(self, msg: Image) -> None:
        self.latest_color = msg

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth = msg

    def _color_info_cb(self, msg: CameraInfo) -> None:
        self.color_info = msg

    def _depth_info_cb(self, msg: CameraInfo) -> None:
        self.depth_info = msg

    def _joint_state_cb(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.current_joint_positions[name] = float(msg.position[index])

        values: list[float] = []
        for name in ARM_JOINT_NAMES:
            if name in msg.name:
                index = msg.name.index(name)
                if index < len(msg.position):
                    values.append(float(msg.position[index]))
        if len(values) == 6:
            self.current_arm_joints = np.asarray(values, dtype=float)

    def _real_joint_state_cb(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.real_joint_positions[name] = float(msg.position[index])
        self.real_joint_state_time = time.monotonic()

    def _real_gripper_status_cb(self, msg: String) -> None:
        with self.gripper_status_lock:
            self.latest_real_gripper_status = (time.monotonic(), str(msg.data))

    def _moveit_start_state_joint_values(self, q_start: np.ndarray) -> tuple[list[str], list[float]]:
        joint_values: dict[str, float] = {
            name: float(value) for name, value in zip(ARM_JOINT_NAMES, q_start)
        }
        gripper_type = str(getattr(self.args, "gripper_type", "ms42dc"))
        for name, default in (
            tuple(BASE_JOINT_DEFAULTS) + tuple(GRIPPER_JOINT_DEFAULTS.get(gripper_type, ()))
        ):
            joint_values[name] = float(self.current_joint_positions.get(name, default))
        for name, value in self.current_joint_positions.items():
            if name not in joint_values and name not in ARM_JOINT_NAMES:
                joint_values[name] = float(value)
        return list(joint_values.keys()), list(joint_values.values())

    def _moveit_unreachable_note(self, target_base: tuple[float, float, float]) -> str:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return ""
        target_aubo = np.asarray(self._transform_point(transform, target_base), dtype=float)
        radius = float(np.linalg.norm(target_aubo))
        max_reach = max(float(self.args.moveit_max_tool0_reach), 0.1)
        tolerance = max(float(self.args.moveit_position_tolerance), 0.0)
        if radius <= max_reach + tolerance:
            return ""
        ax, ay, az = target_aubo
        return (
            f"aubo_target=({ax:.3f},{ay:.3f},{az:.3f}) "
            f"radius={radius:.3f}m exceeds tool0 reach {max_reach:.3f}m"
        )

    def _restart_search_cb(self, _msg: Empty) -> None:
        self._restart_search("restart-topic")

    def _real_execute_trigger_cb(self, _msg: Empty) -> None:
        self._request_real_execution()

    def _real_execute_trigger_srv(self, _request, response):
        self._request_real_execution()
        response.success = True
        response.message = "real execution requested"
        return response

    def _request_real_execution(self) -> None:
        now = time.monotonic()
        with self.real_execution_lock:
            if self.real_execution_requested or self.real_execution_started:
                return
            self.real_execution_requested = True
            self.last_real_execute_request_time = now
        self.get_logger().info("REAL execution trigger received")
        self.get_logger().info("grasp_timing trigger accepted")
        self._preopen_real_gripper_for_request()
        self._start_real_sdk_prewarm_for_request()
        preview = self.last_preview
        if preview is not None and preview.arm_trajectory_frames:
            with self.real_execution_lock:
                thread = self.real_execution_thread
                if thread is not None and thread.is_alive():
                    return
                self.real_execution_thread = threading.Thread(
                    target=self._maybe_execute_real_trajectory,
                    args=(preview,),
                    daemon=True,
                )
                self.real_execution_thread.start()

    def _real_search_scan_control_cb(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self.real_search_scan_enabled:
            return
        self.real_search_scan_enabled = enabled
        self.get_logger().info(f"REAL search scan control: enabled={self.real_search_scan_enabled}")

    def _reset_preview_stream(self, message: str) -> None:
        self.preview_ik_joints = np.asarray(self.current_arm_joints, dtype=float)
        self.preview_ik_velocity = np.zeros(6, dtype=float)
        self.preview_ik_accel = np.zeros(6, dtype=float)
        self.preview_ik_last_update = time.monotonic()
        self.preview_ik_last_progress = 0.0
        self.preview_target_rotation = self.kinematics.fk(self.preview_ik_joints)[:3, :3]
        self.preview_ik_message = message

    def _restart_search(self, reason: str) -> None:
        self.last_restart_search_time = time.monotonic()
        self.inference_paused = False
        self.plan_lock_time = 0.0
        self.last_preview = None
        self.depth_wait_detection = None
        self.depth_wait_reason = ""
        self.depth_wait_header = None
        self.depth_wait_last_publish = 0.0
        self.planning_generation += 1
        with self.real_execution_lock:
            pending_execute = self.real_execution_requested and not self.real_execution_started
            self.real_execution_started = False
            self.real_execution_requested = pending_execute or not str(
                self.args.real_execute_trigger_topic
            ).strip()
        self._reset_preview_stream("waiting for plan")
        self.snapshot_detection = None
        self.snapshot_header = None
        self.snapshot_time = 0.0
        self.missing_frames = 0
        header = self.latest_color.header if self.latest_color is not None else Header()
        self._clear_markers(header)
        self.get_logger().info(f"restart 2D search: {reason}")

    def _color_frame_age_sec(self) -> float | None:
        if self.latest_color is None:
            return None
        stamp = self.latest_color.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0

    def _color_frame_is_stale(self) -> bool:
        max_age = max(float(self.args.max_color_frame_age_sec), 0.0)
        if max_age <= 0.0:
            return False
        age = self._color_frame_age_sec()
        if age is None or age <= max_age:
            return False
        self._throttled_log(f"color frame stale: age={age:.2f}s > {max_age:.2f}s; skipping YOLO")
        return True

    def _tick(self) -> None:
        tick_start = time.monotonic()
        if self.latest_color is None:
            self._throttled_log("waiting for color topic")
            return
        if self.depth_wait_detection is not None:
            self._tick_waiting_for_depth()
            return
        if self.inference_paused and self.last_preview is not None:
            now = time.monotonic()
            locked_period = 1.0 / max(float(self.args.locked_visual_rate), 1.0)
            if now - self.locked_visual_last_publish < locked_period:
                return
            try:
                color = _image_to_bgr(self.latest_color)
            except ValueError as exc:
                self._throttled_log(str(exc))
                return
            self._publish_locked_annotation(color, self.latest_color.header)
            return
        if self._color_frame_is_stale():
            if self.real_execution_requested and not self.real_execution_started:
                since_trigger = (
                    tick_start - self.last_real_execute_request_time
                    if self.last_real_execute_request_time > 0.0
                    else -1.0
                )
                self.get_logger().info(
                    f"grasp_timing tick_skip reason=stale since_trigger={since_trigger:.3f}s"
                )
            return
        try:
            color = _image_to_bgr(self.latest_color)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return
        color_done = time.monotonic()

        model_input = self._yolo_depth_filtered_input(color)
        if model_input is None:
            annotated = color.copy()
            cv2.putText(
                annotated,
                "waiting for depth height mask",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 190, 255),
                2,
                cv2.LINE_AA,
            )
            self.image_pub.publish(_image_from_bgr(annotated, self.latest_color.header))
            self._throttled_log("waiting for depth height mask before YOLO")
            return
        filter_done = time.monotonic()

        result = self._predict(model_input)
        predict_done = time.monotonic()
        detection = self._best_detection(result)
        select_done = time.monotonic()
        annotated = color.copy()
        header = self.latest_color.header
        if detection is not None:
            self._draw_detection_overlay(annotated, detection, "Grasp_ROI")
        if detection is None:
            self.missing_frames += 1
            cv2.putText(
                annotated,
                "no target detection",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self.image_pub.publish(_image_from_bgr(annotated, header))
            if self.missing_frames >= max(int(self.args.lost_frame_threshold), 1):
                self.last_preview = None
                self.snapshot_detection = None
                self.snapshot_header = None
                self._clear_markers(header)
            self._throttled_log("no target detection")
            if self.real_execution_requested and not self.real_execution_started:
                since_trigger = (
                    select_done - self.last_real_execute_request_time
                    if self.last_real_execute_request_time > 0.0
                    else -1.0
                )
                self.get_logger().info(
                    "grasp_timing detect_none "
                    f"since_trigger={since_trigger:.3f}s "
                    f"color={color_done - tick_start:.3f}s "
                    f"depth_filter={filter_done - color_done:.3f}s "
                    f"yolo={predict_done - filter_done:.3f}s "
                    f"select={select_done - predict_done:.3f}s"
                )
            return

        self.missing_frames = 0
        self._publish_detection_event(detection, header)
        needs_snapshot, reason = self._needs_depth_snapshot(detection, color.shape)
        depth_done = select_done
        preview_done = select_done
        if needs_snapshot:
            if self.latest_depth is None or self.depth_info is None:
                self.depth_wait_detection = detection
                self.depth_wait_reason = reason
                self.depth_wait_header = header
                self.inference_paused = True
                self._publish_wait_depth_annotation(color, header)
                self._throttled_log(
                    f"2D target locked; YOLO paused; waiting for {self._missing_depth_text()}"
                )
                return
            try:
                depth = _image_to_depth(self.latest_depth)
            except ValueError as exc:
                self._throttled_log(str(exc))
                return
            depth_done = time.monotonic()
            preview = self._make_preview(detection, depth, reason)
            preview_done = time.monotonic()
        elif self.last_preview is not None:
            preview = replace(self.last_preview, detection=detection, snapshot_reason="cached")
        else:
            preview = None

        if preview is None:
            cv2.putText(
                annotated,
                "detection has no valid depth",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self.image_pub.publish(_image_from_bgr(annotated, header))
            self._clear_markers(header)
            self._throttled_log("detection found, but ROI depth is invalid")
            if needs_snapshot:
                since_trigger = (
                    preview_done - self.last_real_execute_request_time
                    if self.last_real_execute_request_time > 0.0
                    else -1.0
                )
                self.get_logger().info(
                    "grasp_timing preview_failed "
                    f"label={detection.label} reason={reason} "
                    f"since_trigger={since_trigger:.3f}s "
                    f"yolo={predict_done - filter_done:.3f}s "
                    f"select={select_done - predict_done:.3f}s "
                    f"depth={depth_done - select_done:.3f}s "
                    f"make_preview={preview_done - depth_done:.3f}s"
                )
            return

        if needs_snapshot:
            self.snapshot_header = self.latest_depth.header if self.latest_depth is not None else header
            self.snapshot_detection = detection
            self.snapshot_time = time.monotonic()
            self.snapshot_count += 1
            self.inference_paused = True
            self.plan_lock_time = self.snapshot_time
            self._reset_preview_stream("plan locked")
        preview_header = self.snapshot_header or (
            self.latest_depth.header if self.latest_depth is not None else header
        )
        self.last_preview = preview
        if needs_snapshot:
            plan_dispatch_start = time.monotonic()
            self._start_arm_planning(preview, preview_header)
            plan_dispatch_done = time.monotonic()
            output_queue_start = time.monotonic()
            self._publish_and_save_preview_async(preview, preview_header, annotated, color, header)
            output_queue_done = time.monotonic()
            since_restart = (
                plan_dispatch_done - self.last_restart_search_time
                if self.last_restart_search_time > 0.0
                else -1.0
            )
            since_trigger = (
                plan_dispatch_done - self.last_real_execute_request_time
                if self.last_real_execute_request_time > 0.0
                else -1.0
            )
            self.get_logger().info(
                "grasp_timing preview_ready "
                f"label={preview.detection.label} reason={reason} "
                f"since_restart={since_restart:.3f}s "
                f"since_trigger={since_trigger:.3f}s "
                f"tick_total={plan_dispatch_done - tick_start:.3f}s "
                f"color={color_done - tick_start:.3f}s "
                f"depth_filter={filter_done - color_done:.3f}s "
                f"yolo={predict_done - filter_done:.3f}s "
                f"select={select_done - predict_done:.3f}s "
                f"depth={depth_done - select_done:.3f}s "
                f"make_preview={preview_done - depth_done:.3f}s "
                f"planning_dispatch={plan_dispatch_done - plan_dispatch_start:.3f}s "
                f"output_queue={output_queue_done - output_queue_start:.3f}s "
                "publish=async save=async "
                f"roi_points={len(preview.roi_points)}"
            )
        else:
            self._publish_preview(preview, preview_header)
            self._publish_annotated(annotated, preview, header)
            self._save_latest(annotated, color, preview)

    def _yolo_depth_filtered_input(self, color: np.ndarray) -> np.ndarray | None:
        if not bool(self.args.yolo_depth_filter):
            return color
        if self.latest_depth is None or self.depth_info is None:
            return None
        try:
            depth = _image_to_depth(self.latest_depth)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return None
        depth_header = self.latest_depth.header
        if not depth_header.frame_id:
            return None
        base_from_depth = self._base_from_depth_matrix(depth_header, timeout_sec=0.02)
        if base_from_depth is None:
            return None

        depth_m = depth.astype(np.float32) * float(self.args.depth_scale)
        valid = (
            np.isfinite(depth_m)
            & (depth_m >= float(self.args.min_depth))
            & (depth_m <= float(self.args.max_depth))
        )
        keep = np.zeros(depth.shape[:2], dtype=np.uint8)
        if np.any(valid):
            height, width = depth.shape[:2]
            uu, vv = np.meshgrid(
                np.arange(width, dtype=np.float32),
                np.arange(height, dtype=np.float32),
            )
            points = self._pixels_to_xyz(uu[valid], vv[valid], depth_m[valid])
            if points.size:
                base_z = points @ base_from_depth[2, :3] + base_from_depth[2, 3]
                keep[valid] = (
                    (base_z >= float(self.args.yolo_depth_min_base_z))
                    & (base_z <= float(self.args.yolo_depth_max_base_z))
                ).astype(np.uint8)
        keep = cv2.dilate(keep, np.ones((3, 3), dtype=np.uint8), iterations=1)
        if keep.shape[:2] != color.shape[:2]:
            keep = cv2.resize(keep, (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)
        filtered = color.copy()
        filtered[keep == 0] = 0
        return filtered

    def _base_from_depth_transform(self, header: Header, timeout_sec: float):
        source_frame = header.frame_id
        if not source_frame:
            raise TransformException("depth image has no frame_id")
        if source_frame == self.base_frame:
            return None
        return self._lookup_depth_transform(
            self.base_frame,
            source_frame,
            header.stamp,
            timeout_sec,
        )

    def _base_from_depth_matrix(
        self, header: Header | None = None, timeout_sec: float = 0.02
    ) -> np.ndarray | None:
        if header is None:
            if self.latest_depth is None:
                return None
            header = self.latest_depth.header
        if header.frame_id == self.base_frame:
            return np.eye(4, dtype=np.float64)
        try:
            transform = self._base_from_depth_transform(header, timeout_sec)
        except TransformException as exc:
            self._throttled_log(f"waiting for depth TF {self.base_frame} <- {header.frame_id}: {exc}")
            return None
        if transform is None:
            return np.eye(4, dtype=np.float64)
        return self._matrix_from_transform(transform)

    def _lookup_depth_transform(
        self,
        target_frame: str,
        source_frame: str,
        stamp_msg,
        timeout_sec: float,
    ):
        stamp = rclpy.time.Time.from_msg(stamp_msg)
        stamps = [stamp]
        if stamp.nanoseconds:
            stamps.append(rclpy.time.Time())
        last_error = None
        for stamp in stamps:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    stamp,
                    timeout=Duration(seconds=timeout_sec),
                )
            except TransformException as exc:
                last_error = exc
        assert last_error is not None
        raise last_error

    def _missing_depth_text(self) -> str:
        missing = []
        if self.latest_depth is None:
            missing.append(str(self.args.depth_topic))
        if self.depth_info is None:
            missing.append(str(self.args.depth_info_topic))
        return ", ".join(missing) if missing else "valid ROI depth"

    def _tick_waiting_for_depth(self) -> None:
        detection = self.depth_wait_detection
        if detection is None:
            return
        try:
            color = _image_to_bgr(self.latest_color)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return
        header = self.latest_color.header
        if self.latest_depth is None or self.depth_info is None:
            now = time.monotonic()
            if now - self.depth_wait_last_publish >= 0.5:
                self._publish_wait_depth_annotation(color, header)
                self._throttled_log(
                    f"YOLO paused on locked target; waiting for {self._missing_depth_text()}"
                )
            return
        try:
            depth = _image_to_depth(self.latest_depth)
        except ValueError as exc:
            self._throttled_log(str(exc))
            return

        preview = self._make_preview(detection, depth, self.depth_wait_reason or "new-target")
        if preview is None:
            now = time.monotonic()
            if now - self.depth_wait_last_publish >= 0.5:
                self._publish_wait_depth_annotation(
                    color, header, "locked target; waiting for valid ROI depth"
                )
                self._throttled_log("YOLO paused on locked target; waiting for valid ROI depth")
            return

        self.depth_wait_detection = None
        self.depth_wait_reason = ""
        self.depth_wait_header = None
        self.snapshot_header = self.latest_depth.header
        self.snapshot_detection = detection
        self.snapshot_time = time.monotonic()
        self.snapshot_count += 1
        self.plan_lock_time = self.snapshot_time
        self._reset_preview_stream("planning pending")
        preview_header = self.snapshot_header
        self.last_preview = preview
        annotated = color.copy()
        self._draw_detection_overlay(annotated, detection, "LOCKED")
        self._start_arm_planning(preview, preview_header)
        self._publish_and_save_preview_async(preview, preview_header, annotated, color, header)

    def _draw_detection_overlay(
        self, image: np.ndarray, detection: Detection, prefix: str, footer: str | None = None
    ) -> None:
        x1, y1, x2, y2 = (int(round(v)) for v in detection.xyxy)
        green = (0, 255, 80)
        if detection.mask_xy:
            polygon = np.asarray(detection.mask_xy, dtype=np.int32).reshape((-1, 1, 2))
            overlay = image.copy()
            cv2.fillPoly(overlay, [polygon], green)
            cv2.addWeighted(overlay, 0.22, image, 0.78, 0, image)
            cv2.polylines(image, [polygon], isClosed=True, color=green, thickness=2)
        cv2.rectangle(image, (x1, y1), (x2, y2), green, 2)
        cv2.putText(
            image,
            f"Grasp_ROI {detection.label} {detection.confidence:.2f}",
            (max(x1, 8), max(y1 - 8, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            green,
            2,
            cv2.LINE_AA,
        )
        if footer:
            cv2.putText(
                image,
                footer,
                (12, max(image.shape[0] - 18, 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 190, 255),
                2,
                cv2.LINE_AA,
            )

    def _publish_wait_depth_annotation(
        self, color: np.ndarray, header: Header, message: str | None = None
    ) -> None:
        detection = self.depth_wait_detection
        if detection is None:
            return
        self.depth_wait_last_publish = time.monotonic()
        annotated = color.copy()
        text = message or f"YOLO paused; waiting for {self._missing_depth_text()}"
        self._draw_detection_overlay(annotated, detection, "LOCKED_2D", text)
        cv2.putText(
            annotated,
            text,
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 180, 255),
            2,
            cv2.LINE_AA,
        )
        self.image_pub.publish(_image_from_bgr(annotated, header))

    def _start_arm_planning(self, preview: GraspPreview, preview_header: Header) -> None:
        if str(self.args.planner_backend) == "none":
            perception = replace(
                preview,
                base_path_xyz=self._sample_trajectory_for_display(preview.base_trajectory_segments),
                ik_message="perception_only: shape target ready; planning skipped",
            )
            self.last_preview = perception
            self.preview_ik_message = perception.ik_message
            self.plan_lock_time = time.monotonic()
            self._publish_preview(perception, preview_header)
            self.get_logger().info("3D snapshot published; arm planning skipped")
            self._restart_perception_only_search_later()
            return
        with self.planning_lock:
            if self.planning_thread is not None and self.planning_thread.is_alive():
                return
            self.planning_generation += 1
            generation = self.planning_generation
            self.preview_ik_message = "planning trajectory"
            self.planning_thread = threading.Thread(
                target=self._arm_planning_worker,
                args=(generation, preview, preview_header),
                daemon=True,
            )
            self.planning_thread.start()
        self.get_logger().info("3D snapshot ready; arm trajectory planning started")

    def _arm_planning_worker(
        self, generation: int, preview: GraspPreview, preview_header: Header
    ) -> None:
        start = time.monotonic()
        try:
            arm_frames, ik_message = self._make_constrained_arm_trajectory(preview)
        except Exception as exc:
            arm_frames = []
            ik_message = f"planning exception: {type(exc).__name__}: {exc}"
            self.get_logger().error(ik_message)
        solve_done = time.monotonic()
        if self.stopping or not rclpy.ok():
            return
        planned_path = self._arm_trajectory_grasp_path_base(arm_frames) if arm_frames else []
        path_done = time.monotonic()
        planned = replace(
            preview,
            base_path_xyz=planned_path,
            arm_trajectory_frames=arm_frames,
            ik_message=ik_message,
        )
        with self.planning_lock:
            if generation != self.planning_generation:
                return
            self.last_preview = planned
            self.preview_ik_message = ik_message
            if arm_frames:
                self.plan_lock_time = time.monotonic()
        if self.stopping or not rclpy.ok():
            return
        ready_done = time.monotonic()
        self._publish_preview_async(planned, preview_header, label="planned")
        elapsed = ready_done - start
        self.get_logger().info(
            "grasp_timing planning_done "
            f"total={elapsed:.3f}s "
            f"solve={solve_done - start:.3f}s "
            f"path={path_done - solve_done:.3f}s "
            "publish=async "
            f"frames={len(arm_frames)}"
        )
        if arm_frames and "partial" not in ik_message:
            self.get_logger().info(
                f"arm trajectory ready: frames={len(arm_frames)} "
                f"duration={arm_frames[-1].time_from_start:.2f}s "
                f"planning_wall={elapsed:.2f}s {ik_message}"
            )
            self._maybe_execute_real_trajectory(planned)
        elif arm_frames:
            self.get_logger().warn(
                f"arm trajectory partial after {elapsed:.2f}s: frames={len(arm_frames)} "
                f"duration={arm_frames[-1].time_from_start:.2f}s {ik_message}"
            )
            self._maybe_execute_real_trajectory(planned)
        else:
            self._discard_real_sdk_warm_session("planning unavailable", stop_joint=True)
            self.get_logger().warn(f"arm trajectory unavailable after {elapsed:.2f}s: {ik_message}")

    def stop(self) -> None:
        self.stopping = True
        self._discard_real_sdk_warm_session("node stop", stop_joint=True)
        with self.planning_lock:
            self.planning_generation += 1
            thread = self.planning_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.3)
        with self.real_sdk_warm_lock:
            sdk_thread = self.real_sdk_warm_thread
        if sdk_thread is not None and sdk_thread.is_alive():
            sdk_thread.join(timeout=0.3)

    def _maybe_execute_real_trajectory(self, preview: GraspPreview) -> None:
        if not bool(self.args.execute_real):
            return
        with self.real_execution_lock:
            if not self.real_execution_requested:
                self.get_logger().info("REAL trajectory ready; waiting for execute trigger")
                return
            if self.real_execution_started:
                return
            self.real_execution_started = True
        self.get_logger().info("REAL trajectory ready; executing requested grasp now")

        try:
            self._execute_real_trajectory(preview)
        except Exception as exc:
            self._discard_real_sdk_warm_session("real execution blocked", stop_joint=True)
            self.get_logger().error(f"real execution blocked: {exc}")
        finally:
            self._discard_real_sdk_warm_session("real execution finished", stop_joint=True)

    def _execute_real_trajectory(self, preview: GraspPreview) -> None:
        if not preview.arm_trajectory_frames:
            raise RuntimeError("planned trajectory is empty")
        if "partial" in preview.ik_message and not bool(self.args.real_allow_partial):
            raise RuntimeError(f"refusing partial trajectory: {preview.ik_message}")
        if str(self.args.execute_real_confirm).strip().upper() != "YES":
            raise RuntimeError(
                "set ARACHNE_CONFIRM_GRASP_EXECUTE_REAL=YES or pass --execute-real-confirm YES"
            )
        actual_start, actual_source = self._real_start_values()
        if actual_start is None:
            return
        preview = self._rebase_preview_start_to_real(preview, actual_start, actual_source)
        if not self._real_start_state_matches(
            preview.arm_trajectory_frames[0],
            actual_values=actual_start,
            source=actual_source,
        ):
            return
        backend = str(self.args.real_execute_backend)
        if backend == "sdk_move_joint":
            self._execute_real_sdk_move_joint(preview)
            return
        if backend == "follow_joint_trajectory":
            self._execute_real_follow_joint_trajectory(preview)
            return
        raise RuntimeError(f"unsupported real execution backend: {backend}")

    def _real_trigger_elapsed(self) -> float:
        if self.last_real_execute_request_time <= 0.0:
            return 0.0
        return max(time.monotonic() - self.last_real_execute_request_time, 0.0)

    def _real_sdk_prewarm_enabled(self) -> bool:
        return (
            bool(self.args.execute_real)
            and bool(self.args.real_sdk_prewarm)
            and str(self.args.real_execute_backend) == "sdk_move_joint"
            and not bool(self.args.prefer_aubo_move_joint_action)
            and str(self.args.execute_real_confirm).strip().upper() == "YES"
        )

    def _start_real_sdk_prewarm_for_request(self) -> None:
        if not self._real_sdk_prewarm_enabled() or self.stopping:
            return
        ip = str(self.args.real_sdk_ip)
        port = int(self.args.real_sdk_rpc_port)
        timeout = max(float(self.args.real_sdk_rpc_timeout), 0.1)
        with self.real_sdk_warm_lock:
            if self.real_sdk_warm_session is not None:
                return
            thread = self.real_sdk_warm_thread
            if thread is not None and thread.is_alive():
                return
            self.real_sdk_warm_generation += 1
            generation = self.real_sdk_warm_generation
            thread = threading.Thread(
                target=self._real_sdk_prewarm_worker,
                args=(generation, ip, port, timeout),
                daemon=True,
            )
            self.real_sdk_warm_thread = thread
        self.get_logger().info(
            "grasp_timing sdk_prewarm_start "
            f"since_trigger={self._real_trigger_elapsed():.3f}s rpc={ip}:{port}"
        )
        thread.start()

    def _real_sdk_prewarm_is_current(self, generation: int) -> bool:
        with self.real_sdk_warm_lock:
            return (
                generation == self.real_sdk_warm_generation
                and not self.stopping
                and rclpy.ok()
            )

    def _real_sdk_prewarm_worker(
        self, generation: int, ip: str, port: int, timeout: float
    ) -> None:
        start = time.monotonic()
        rpc: AuboDirectJsonRpc | None = None
        owner_owned = False
        gate_owned = False
        try:
            rpc = AuboDirectJsonRpc(ip, port, timeout)
            rpc.connect()
            connect_done = time.monotonic()
            self.get_logger().info(f"Aubo SDK prewarm connect ok: robot={rpc.robot_name} rpc={ip}:{port}")
            if not self._real_sdk_prewarm_is_current(generation):
                self._close_real_sdk_handles(rpc, owner_owned, gate_owned, "stale prewarm", stop_joint=False)
                return
            self._real_sdk_require_running(rpc)
            mode_done = time.monotonic()
            if not self._real_sdk_prewarm_is_current(generation):
                self._close_real_sdk_handles(rpc, owner_owned, gate_owned, "stale prewarm", stop_joint=False)
                return
            owner_owned = self._real_sdk_enter_control_owner()
            owner_done = time.monotonic()
            if not self._real_sdk_prewarm_is_current(generation):
                self._close_real_sdk_handles(rpc, owner_owned, gate_owned, "stale prewarm", stop_joint=False)
                return
            gate_owned = self._real_sdk_enter_gate()
            gate_done = time.monotonic()
            if not self._real_sdk_prewarm_is_current(generation):
                self._close_real_sdk_handles(rpc, owner_owned, gate_owned, "stale prewarm", stop_joint=True)
                return
            self._real_sdk_exit_servo_mode(rpc)
            servo_done = time.monotonic()
            self._real_sdk_stop_joint(rpc, "prewarm cleanup", warn_only=True)
            ready_done = time.monotonic()
            session = RealSdkWarmSession(
                generation=generation,
                rpc=rpc,
                owner_owned=owner_owned,
                gate_owned=gate_owned,
                started_at=start,
                ready_at=ready_done,
                ip=ip,
                port=port,
            )
            with self.real_sdk_warm_lock:
                if (
                    generation == self.real_sdk_warm_generation
                    and self.real_sdk_warm_session is None
                    and not self.stopping
                    and rclpy.ok()
                ):
                    self.real_sdk_warm_session = session
                    rpc = None
                else:
                    session = None  # type: ignore[assignment]
            if session is None:
                self._close_real_sdk_handles(
                    rpc, owner_owned, gate_owned, "stale prewarm", stop_joint=True
                )
                return
            self.get_logger().info(
                "grasp_timing sdk_prewarm_ready "
                f"total={ready_done - start:.3f}s "
                f"since_trigger={self._real_trigger_elapsed():.3f}s "
                f"connect={connect_done - start:.3f}s "
                f"mode={mode_done - connect_done:.3f}s "
                f"owner={owner_done - mode_done:.3f}s "
                f"gate={gate_done - owner_done:.3f}s "
                f"servo={servo_done - gate_done:.3f}s "
                f"stop={ready_done - servo_done:.3f}s"
            )
        except Exception as exc:
            elapsed = time.monotonic() - start
            if rpc is not None:
                self._close_real_sdk_handles(
                    rpc, owner_owned, gate_owned, "failed prewarm", stop_joint=gate_owned
                )
            self.get_logger().warn(
                "grasp_timing sdk_prewarm_failed "
                f"total={elapsed:.3f}s since_trigger={self._real_trigger_elapsed():.3f}s "
                f"error={exc}"
            )

    def _take_real_sdk_warm_session(self) -> RealSdkWarmSession | None:
        if not self._real_sdk_prewarm_enabled():
            return None
        start = time.monotonic()
        wait_sec = max(float(self.args.real_sdk_prewarm_wait_sec), 0.0)
        deadline = start + wait_sec
        thread_alive = False
        while rclpy.ok() and not self.stopping:
            with self.real_sdk_warm_lock:
                session = self.real_sdk_warm_session
                if session is not None:
                    self.real_sdk_warm_session = None
                    waited = time.monotonic() - start
                    self.get_logger().info(
                        "grasp_timing sdk_prewarm_take "
                        f"wait={waited:.3f}s "
                        f"age={time.monotonic() - session.ready_at:.3f}s "
                        f"since_trigger={self._real_trigger_elapsed():.3f}s "
                        f"generation={session.generation}"
                    )
                    return session
                thread = self.real_sdk_warm_thread
                thread_alive = thread is not None and thread.is_alive()
            if time.monotonic() >= deadline or not thread_alive:
                break
            time.sleep(0.01)
        waited = time.monotonic() - start
        if thread_alive:
            with self.real_sdk_warm_lock:
                self.real_sdk_warm_generation += 1
        self.get_logger().info(
            "grasp_timing sdk_prewarm_miss "
            f"wait={waited:.3f}s thread_alive={thread_alive} "
            f"since_trigger={self._real_trigger_elapsed():.3f}s"
        )
        return None

    def _discard_real_sdk_warm_session(self, reason: str, *, stop_joint: bool) -> None:
        with self.real_sdk_warm_lock:
            self.real_sdk_warm_generation += 1
            session = self.real_sdk_warm_session
            self.real_sdk_warm_session = None
        if session is None:
            return
        self.get_logger().info(
            "grasp_timing sdk_prewarm_discard "
            f"reason={reason} age={time.monotonic() - session.ready_at:.3f}s"
        )
        self._close_real_sdk_warm_session(session, reason, stop_joint=stop_joint)

    def _close_real_sdk_warm_session(
        self, session: RealSdkWarmSession, reason: str, *, stop_joint: bool
    ) -> None:
        self._close_real_sdk_handles(
            session.rpc,
            session.owner_owned,
            session.gate_owned,
            reason,
            stop_joint=stop_joint,
        )

    def _close_real_sdk_handles(
        self,
        rpc: AuboDirectJsonRpc | None,
        owner_owned: bool,
        gate_owned: bool,
        reason: str,
        *,
        stop_joint: bool,
    ) -> None:
        try:
            if rpc is not None and gate_owned and stop_joint:
                self._real_sdk_stop_joint(rpc, reason, warn_only=True)
        finally:
            try:
                self._real_sdk_exit_gate(gate_owned)
            finally:
                try:
                    self._real_sdk_exit_control_owner(owner_owned)
                finally:
                    if rpc is not None:
                        rpc.close()

    def _execute_real_sdk_move_joint(self, preview: GraspPreview) -> None:
        frames = preview.arm_trajectory_frames
        targets = self._real_sdk_move_targets(preview, frames)
        if not targets:
            raise RuntimeError("real SDK moveJoint target list is empty")

        if bool(self.args.prefer_aubo_move_joint_action):
            try:
                if self._execute_real_aubo_move_joint_action(preview, targets):
                    return
            except Exception as exc:
                if not bool(self.args.aubo_move_joint_fallback_internal):
                    raise
                self.get_logger().warn(
                    "AuboMoveJoint action path failed; falling back to guarded internal "
                    f"SDK moveJoint: {exc}"
                )
            else:
                if not bool(self.args.aubo_move_joint_fallback_internal):
                    raise TimeoutError(
                        f"AuboMoveJoint action server unavailable: {self.args.aubo_move_joint_action_name}"
                    )
                self.get_logger().warn(
                    "AuboMoveJoint action server unavailable; falling back to guarded internal "
                    "SDK moveJoint"
                )

        ip = str(self.args.real_sdk_ip)
        port = int(self.args.real_sdk_rpc_port)
        timeout = max(float(self.args.real_sdk_rpc_timeout), 0.1)
        speed = max(float(self.args.real_sdk_move_speed), 0.01)
        accel = max(float(self.args.real_sdk_move_accel), 0.05)
        blend_radius = max(float(self.args.real_sdk_blend_radius), 0.0)
        duration_scale = max(float(self.args.real_sdk_segment_duration_scale), 0.0)
        warm_session = self._take_real_sdk_warm_session()
        prewarmed = warm_session is not None
        owner_owned = False
        gate_owned = False
        rpc: AuboDirectJsonRpc | None = None
        self.get_logger().warn(
            f"sending REAL arm motion through Aubo SDK moveJoint: targets={len(targets)} "
            f"rpc={ip}:{port} speed={speed:.3f}rad/s accel={accel:.3f}rad/s2 "
            f"prewarmed={prewarmed}"
        )

        try:
            setup_start = time.monotonic()
            if warm_session is not None:
                rpc = warm_session.rpc
                owner_owned = warm_session.owner_owned
                gate_owned = warm_session.gate_owned
                setup_done = time.monotonic()
                self.get_logger().info(
                    "grasp_timing sdk_motion_setup "
                    "source=prewarmed "
                    f"total={setup_done - setup_start:.3f}s "
                    f"age={setup_done - warm_session.ready_at:.3f}s "
                    f"since_trigger={self._real_trigger_elapsed():.3f}s"
                )
            else:
                rpc = AuboDirectJsonRpc(ip, port, timeout)
                rpc.connect()
                connect_done = time.monotonic()
                self.get_logger().info(f"Aubo SDK connect ok: robot={rpc.robot_name} rpc={ip}:{port}")
                self._real_sdk_require_running(rpc)
                mode_done = time.monotonic()
                owner_owned = self._real_sdk_enter_control_owner()
                owner_done = time.monotonic()
                gate_owned = self._real_sdk_enter_gate()
                gate_done = time.monotonic()
                self._real_sdk_exit_servo_mode(rpc)
                servo_done = time.monotonic()
                self._real_sdk_stop_joint(rpc, "pre-move cleanup", warn_only=True)
                cleanup_done = time.monotonic()
                self.get_logger().info(
                    "grasp_timing sdk_motion_setup "
                    "source=fresh "
                    f"total={cleanup_done - setup_start:.3f}s "
                    f"connect={connect_done - setup_start:.3f}s "
                    f"mode={mode_done - connect_done:.3f}s "
                    f"owner={owner_done - mode_done:.3f}s "
                    f"gate={gate_done - owner_done:.3f}s "
                    f"servo={servo_done - gate_done:.3f}s "
                    f"stop={cleanup_done - servo_done:.3f}s "
                    f"since_trigger={self._real_trigger_elapsed():.3f}s"
                )
            self._open_real_gripper_before_motion()

            previous_time = 0.0
            closed = False
            opened = False
            (
                _close_start,
                _close_end,
                _open_start,
                _open_end,
                close_label,
                open_label,
            ) = self._gripper_event_progresses(preview)
            for index, (label, frame) in enumerate(targets, start=1):
                if self.stopping or not rclpy.ok():
                    return
                target = [float(value) for value in frame.positions]
                segment_dt = max(float(frame.time_from_start) - previous_time, 0.0)
                duration = 0.0
                if duration_scale > 0.0:
                    duration = max(
                        segment_dt * duration_scale,
                        max(float(self.args.real_sdk_min_segment_duration), 0.0),
                    )
                result = rpc.robot_call(
                    "MotionControl.moveJoint",
                    [target, accel, speed, blend_radius, duration],
                )
                if result not in (0, None):
                    raise RuntimeError(f"Aubo SDK moveJoint failed at {label}: result={result}")
                self.get_logger().warn(
                    f"REAL moveJoint {index}/{len(targets)} {label}: "
                    f"t={frame.time_from_start:.2f}s duration={duration:.2f}s"
                )
                self._real_sdk_wait_arrival(rpc, np.asarray(target, dtype=float), label, segment_dt)
                previous_time = float(frame.time_from_start)

                if bool(self.args.real_execute_gripper) and not closed:
                    if label == close_label:
                        self._publish_real_gripper("close")
                        closed = True
                        settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                        if settle > 0.0:
                            time.sleep(settle)
                        self._require_real_gripper_capture_or_raise()
                if bool(self.args.real_execute_gripper) and not opened:
                    if label == open_label:
                        self._publish_real_gripper("open")
                        opened = True
                        settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                        if settle > 0.0:
                            time.sleep(settle)
            if not self.stopping and rclpy.ok():
                self.get_logger().info("REAL arm SDK moveJoint sequence complete")
        finally:
            if rpc is not None:
                try:
                    if gate_owned:
                        self._real_sdk_stop_joint(rpc, "post-move cleanup", warn_only=True)
                finally:
                    try:
                        self._real_sdk_exit_gate(gate_owned)
                    finally:
                        try:
                            self._real_sdk_exit_control_owner(owner_owned)
                        finally:
                            rpc.close()

    def _real_search_scan_loop(self) -> None:
        ip = str(self.args.real_sdk_ip)
        port = int(self.args.real_sdk_rpc_port)
        timeout = max(float(self.args.real_sdk_rpc_timeout), 0.1)
        speed = max(float(self.args.real_search_scan_speed), 0.01)
        accel = max(float(self.args.real_search_scan_accel), 0.05)
        period = max(float(self.args.real_search_scan_period_sec), 1.0)
        shoulder = math.radians(float(self.args.real_search_scan_shoulder_deg))
        wrist3 = math.radians(float(self.args.real_search_scan_wrist3_deg))
        left_delta = np.asarray(
            _float_values(str(self.args.real_search_scan_left_delta), 6, "--real-search-scan-left-delta"),
            dtype=float,
        )
        right_delta = np.asarray(
            _float_values(str(self.args.real_search_scan_right_delta), 6, "--real-search-scan-right-delta"),
            dtype=float,
        )
        if float(np.max(np.abs(left_delta))) < 1e-6 and float(np.max(np.abs(right_delta))) < 1e-6:
            left_delta = np.array([-shoulder, 0.0, 0.0, 0.0, 0.0, -wrist3], dtype=float)
            right_delta = np.array([shoulder, 0.0, 0.0, 0.0, 0.0, wrist3], dtype=float)
        owner_owned = False
        gate_owned = False
        center: np.ndarray | None = None
        phase = 0
        rpc: AuboDirectJsonRpc | None = None
        while rclpy.ok() and not self.stopping:
            active = self._real_search_scan_active()
            try:
                if not active:
                    if rpc is not None:
                        if gate_owned:
                            self._real_sdk_stop_joint(rpc, "search scan pause", warn_only=True)
                            if center is not None:
                                result = rpc.robot_call(
                                    "MotionControl.moveJoint",
                                    [[float(value) for value in center], accel, speed, 0.0, 0.0],
                                )
                                self.get_logger().info(
                                    f"REAL search scan return center result={result} error=None"
                                )
                    self._real_sdk_exit_gate(gate_owned)
                    self._real_sdk_exit_control_owner(owner_owned)
                    gate_owned = False
                    owner_owned = False
                    center = None
                    if rpc is not None:
                        rpc.close()
                        rpc = None
                    time.sleep(0.2)
                    continue
                if rpc is None:
                    rpc = AuboDirectJsonRpc(ip, port, timeout)
                    rpc.connect()
                    self._real_sdk_require_running(rpc)
                    owner_owned = self._real_sdk_enter_control_owner()
                    gate_owned = self._real_sdk_enter_gate()
                    self._real_sdk_exit_servo_mode(rpc)
                    self._real_sdk_stop_joint(rpc, "search scan start", warn_only=True)
                    center = self._real_sdk_joint_positions(rpc)
                    self.get_logger().info(
                        "REAL search scan active: "
                        f"left_delta={left_delta.tolist()} right_delta={right_delta.tolist()}"
                    )
                if center is None:
                    center = self._real_sdk_joint_positions(rpc)
                # ponytail: copy sim's joint-space scan deltas around the live center pose.
                target = np.asarray(center, dtype=float) + (left_delta if phase % 2 == 0 else right_delta)
                result = rpc.robot_call(
                    "MotionControl.moveJoint",
                    [[float(value) for value in target], accel, speed, 0.0, 0.0],
                )
                self.get_logger().info(
                    f"REAL search scan moveJoint phase={phase % 2} result={result} error=None"
                )
                if result not in (0, None):
                    time.sleep(1.0)
                    continue
                deadline = time.monotonic() + period * 0.5
                while time.monotonic() < deadline and self._real_search_scan_active():
                    time.sleep(0.05)
                phase += 1
                time.sleep(max(period * 0.10, 0.05))
            except Exception as exc:  # pragma: no cover - live robot path.
                self._warn_real_search_scan_stopped(str(exc))
                try:
                    if rpc is not None and gate_owned:
                        self._real_sdk_stop_joint(rpc, "search scan error", warn_only=True)
                finally:
                    self._real_sdk_exit_gate(gate_owned)
                    self._real_sdk_exit_control_owner(owner_owned)
                    gate_owned = False
                    owner_owned = False
                    center = None
                    if rpc is not None:
                        rpc.close()
                        rpc = None
                time.sleep(1.0)
        if rpc is not None:
            try:
                if gate_owned:
                    self._real_sdk_stop_joint(rpc, "search scan shutdown", warn_only=True)
            finally:
                self._real_sdk_exit_gate(gate_owned)
                self._real_sdk_exit_control_owner(owner_owned)
                rpc.close()

    def _warn_real_search_scan_stopped(self, message: str) -> None:
        now = time.monotonic()
        if message == self._last_real_scan_warning and now - self._last_real_scan_warning_time < 10.0:
            return
        self._last_real_scan_warning = message
        self._last_real_scan_warning_time = now
        self.get_logger().warning(f"REAL search scan stopped: {message}")

    def _real_search_scan_active(self) -> bool:
        with self.real_execution_lock:
            real_started = self.real_execution_started
        planning = self.planning_thread
        return bool(self.real_search_scan_enabled) and (
            self.latest_color is not None
            and not self.inference_paused
            and self.depth_wait_detection is None
            and (planning is None or not planning.is_alive())
            and not real_started
        )

    def _restart_perception_only_search_later(self) -> None:
        delay = max(float(self.args.perception_only_restart_sec), 0.0)
        if delay <= 0.0:
            return

        def worker() -> None:
            time.sleep(delay)
            if self.stopping or not rclpy.ok():
                return
            if str(self.args.planner_backend) == "none" and self.inference_paused:
                self._restart_search("perception-only auto restart")

        threading.Thread(target=worker, daemon=True).start()

    def _execute_real_aubo_move_joint_action(
        self,
        preview: GraspPreview,
        targets: list[tuple[str, JointTrajectoryFrame]],
    ) -> bool:
        client = self.aubo_move_joint_client
        if client is None:
            return False
        wait_sec = max(float(self.args.aubo_move_joint_wait_server_sec), 0.0)
        if not client.wait_for_server(wait_sec):
            return False

        speed = max(float(self.args.real_sdk_move_speed), 0.01)
        accel = max(float(self.args.real_sdk_move_accel), 0.05)
        blend_radius = max(float(self.args.real_sdk_blend_radius), 0.0)
        duration_scale = max(float(self.args.real_sdk_segment_duration_scale), 0.0)
        tolerance = max(float(self.args.real_sdk_goal_tolerance_rad), 0.001)
        timeout_padding = max(float(self.args.real_sdk_arrival_timeout_padding), 0.0)
        self.get_logger().warn(
            f"sending REAL arm motion through /arachne/aubo/move_joint: targets={len(targets)} "
            f"speed={speed:.3f}rad/s accel={accel:.3f}rad/s2"
        )

        self._open_real_gripper_before_motion()

        previous_time = 0.0
        closed = False
        opened = False
        if preview.arm_trajectory_frames:
            previous_target = np.asarray(preview.arm_trajectory_frames[0].positions, dtype=float)
        else:
            previous_target = np.asarray(targets[0][1].positions, dtype=float)
        (
            _close_start,
            _close_end,
            _open_start,
            _open_end,
            close_label,
            open_label,
        ) = self._gripper_event_progresses(preview)
        for index, (label, frame) in enumerate(targets, start=1):
            if self.stopping or not rclpy.ok():
                return True
            target = [float(value) for value in frame.positions]
            segment_dt = max(float(frame.time_from_start) - previous_time, 0.0)
            duration = 0.0
            if duration_scale > 0.0:
                duration = max(
                    segment_dt * duration_scale,
                    max(float(self.args.real_sdk_min_segment_duration), 0.0),
                )
            max_delta = float(
                np.max(np.abs(self._joint_delta(np.asarray(target, dtype=float), previous_target)))
            )
            timeout_sec = max(segment_dt, duration, max_delta / speed, 0.5) + timeout_padding
            success, message, final_error = client.move_joint(
                target,
                label=label,
                speed_rad_sec=speed,
                accel_rad_sec2=accel,
                blend_radius=blend_radius,
                duration_sec=duration,
                goal_tolerance_rad=tolerance,
                timeout_sec=timeout_sec,
            )
            previous_target = np.asarray(target, dtype=float)
            if not success:
                raise RuntimeError(
                    f"AuboMoveJoint action failed at {label}: {message} "
                    f"final_error={final_error:.3f}rad"
                )
            self.get_logger().warn(
                f"REAL moveJoint {index}/{len(targets)} {label}: "
                f"t={frame.time_from_start:.2f}s duration={duration:.2f}s "
                f"final_error={final_error:.3f}rad"
            )
            previous_time = float(frame.time_from_start)

            if bool(self.args.real_execute_gripper) and not closed:
                if label == close_label:
                    self._publish_real_gripper("close")
                    closed = True
                    settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                    if settle > 0.0:
                        time.sleep(settle)
                    self._require_real_gripper_capture_or_raise()
            if bool(self.args.real_execute_gripper) and not opened:
                if label == open_label:
                    self._publish_real_gripper("open")
                    opened = True
                    settle = max(float(self.args.real_gripper_settle_sec), 0.0)
                    if settle > 0.0:
                        time.sleep(settle)
        if not self.stopping and rclpy.ok():
            self.get_logger().warn("REAL arm AuboMoveJoint action sequence complete")
        return True

    def _execute_real_follow_joint_trajectory(self, preview: GraspPreview) -> None:
        if self.real_arm_action_client is None:
            raise RuntimeError("real FollowJointTrajectory action client is not configured")
        action_name = str(self.args.real_follow_action)
        if not self.real_arm_action_client.wait_for_server(timeout_sec=2.0):
            raise TimeoutError(f"real arm action server unavailable: {action_name}")

        frames = preview.arm_trajectory_frames
        if not bool(self.args.real_execute_gripper):
            self._send_real_follow_joint_segment(frames, 0, len(frames) - 1, "full")
            self.get_logger().warn("REAL arm trajectory complete")
            return

        close_start, _close_end, open_start, _open_end, _close_label, _open_label = (
            self._gripper_event_progresses(preview)
        )
        semantic_indices = self._real_sdk_semantic_frame_indices(preview, frames)
        close_index = semantic_indices.get(str(self.args.gripper_close_waypoint).strip())
        open_index = semantic_indices.get(str(self.args.gripper_open_waypoint).strip())
        if close_index is None:
            close_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * close_start
            )
        if open_index is None:
            open_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * open_start
            )
        close_index = min(max(int(close_index), 1), len(frames) - 1)
        open_index = min(max(int(open_index), close_index), len(frames) - 1)

        settle = max(float(self.args.real_gripper_settle_sec), 0.0)
        self._open_real_gripper_before_motion()
        self._send_real_follow_joint_segment(frames, 0, close_index, "to_grasp")
        self._publish_real_gripper("close")
        if settle > 0.0:
            time.sleep(settle)
        self._require_real_gripper_capture_or_raise()
        self._send_real_follow_joint_segment(frames, close_index, open_index, "to_release")
        self._publish_real_gripper("open")
        if settle > 0.0:
            time.sleep(settle)
        self._send_real_follow_joint_segment(frames, open_index, len(frames) - 1, "finish")
        self.get_logger().warn("REAL arm trajectory complete")

    def _send_real_follow_joint_segment(
        self,
        frames: list[JointTrajectoryFrame],
        start_index: int,
        end_index: int,
        label: str,
    ) -> None:
        start_index = min(max(int(start_index), 0), len(frames) - 1)
        end_index = min(max(int(end_index), start_index), len(frames) - 1)
        if end_index <= start_index:
            return
        segment = self._rebased_trajectory_frames(frames[start_index : end_index + 1])
        trajectory = self._real_joint_trajectory_msg(segment)
        duration = float(segment[-1].time_from_start)
        action_name = str(self.args.real_follow_action)
        self.get_logger().warn(
            f"sending REAL arm trajectory segment {label}: points={len(trajectory.points)} "
            f"duration={duration:.2f}s action={action_name}"
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        margin = max(float(self.args.real_goal_time_margin), 0.0)
        goal.goal_time_tolerance.sec = int(margin)
        goal.goal_time_tolerance.nanosec = int((margin % 1.0) * 1e9)

        goal_future = self.real_arm_action_client.send_goal_async(goal)
        if not self._wait_future(goal_future, 3.0):
            raise TimeoutError(f"real arm action segment {label} goal response timed out")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"real arm action segment {label} goal rejected")

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        timeout = duration + max(float(self.args.real_action_timeout_padding), 0.0)
        while rclpy.ok() and not self.stopping and not result_future.done():
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                cancel_future = goal_handle.cancel_goal_async()
                self._wait_future(cancel_future, 1.0)
                raise TimeoutError(
                    f"real arm action segment {label} result timed out after {elapsed:.2f}s"
                )
            time.sleep(0.02)

        if self.stopping or not rclpy.ok():
            return
        result_response = result_future.result()
        result = result_response.result if result_response is not None else None
        if result is None:
            raise RuntimeError(f"real arm action segment {label} returned empty result")
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"real arm action segment {label} failed: "
                f"code={result.error_code} {result.error_string}"
            )

    def _real_sdk_move_targets(
        self, preview: GraspPreview, frames: list[JointTrajectoryFrame]
    ) -> list[tuple[str, JointTrajectoryFrame]]:
        if len(frames) <= 1:
            return []
        close_start, _close_end, open_start, _open_end, close_label, open_label = (
            self._gripper_event_progresses(preview)
        )
        semantic_indices = self._real_sdk_semantic_frame_indices(preview, frames)
        close_index = semantic_indices.get(str(self.args.gripper_close_waypoint).strip())
        open_index = semantic_indices.get(str(self.args.gripper_open_waypoint).strip())
        if close_index is None:
            close_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * close_start
            )
        if open_index is None:
            open_index = self._frame_index_at_or_after(
                frames, float(frames[-1].time_from_start) * open_start
            )
        close_index = min(max(int(close_index), 1), len(frames) - 1)
        open_index = min(max(int(open_index), close_index), len(frames) - 1)
        if open_index <= close_index and close_index < len(frames) - 1:
            open_index = close_index + 1
        selected: dict[int, str] = {}
        planned_names = {
            name for name, _point, _progress in self._planning_target_samples(preview)
        }
        semantic_labels = {
            "approach": "approach",
            "grasp": close_label,
            "safe_mid": "safe_mid",
            "basket_over": open_label,
            "drop": "drop",
        }
        for name, label in semantic_labels.items():
            if name not in planned_names:
                continue
            index = semantic_indices.get(name)
            if index is not None and index > 0:
                if name == "grasp":
                    index = close_index
                elif name == str(self.args.gripper_open_waypoint).strip():
                    index = open_index
                selected[min(max(int(index), 1), len(frames) - 1)] = label
        selected[close_index] = close_label
        selected[open_index] = open_label

        if bool(self.args.real_fixed_post_grasp):
            return self._real_fixed_post_grasp_targets(frames[0], frames[close_index], close_label, open_label)

        max_delta = max(float(self.args.real_sdk_max_segment_joint_delta), 0.05)
        max_targets = max(int(self.args.real_sdk_max_targets), 2)
        if not bool(self.args.real_sdk_semantic_targets_only):
            last_q = np.asarray(frames[0].positions, dtype=float)
            safe_mid_index = semantic_indices.get("safe_mid", open_index)
            for index, frame in enumerate(frames[1:], start=1):
                if index in selected:
                    last_q = np.asarray(frame.positions, dtype=float)
                    continue
                if close_index < index < safe_mid_index:
                    continue
                delta = float(
                    np.max(np.abs(self._joint_delta(np.asarray(frame.positions, dtype=float), last_q)))
                )
                if delta >= max_delta:
                    selected[index] = f"segment_{len(selected) + 1}"
                    last_q = np.asarray(frame.positions, dtype=float)
                if len(selected) >= max_targets:
                    break

        if (len(frames) - 1) not in selected:
            selected[len(frames) - 1] = "final"
        ordered = sorted(selected.items(), key=lambda item: item[0])
        if len(ordered) > max_targets:
            keep = {
                index
                for index, label in ordered
                if label in {close_label, open_label, "safe_mid", "final"}
            }
            remaining = [index for index, _label in ordered if index not in keep]
            budget = max(max_targets - len(keep), 0)
            if budget > 0 and remaining:
                step = max(int(math.ceil(len(remaining) / float(budget))), 1)
                keep.update(remaining[::step][:budget])
            ordered = [(index, selected[index]) for index in sorted(keep)]
        targets = [(label, frames[index]) for index, label in ordered if index > 0]
        if bool(self.args.real_return_home):
            targets.append(("home", self._real_home_frame(frames[-1], frames[0].positions)))
        return targets

    def _real_fixed_post_grasp_targets(
        self,
        start_frame: JointTrajectoryFrame,
        grasp_frame: JointTrajectoryFrame,
        close_label: str,
        open_label: str,
    ) -> list[tuple[str, JointTrajectoryFrame]]:
        basket_joints = self._optional_real_fixed_joints(
            str(self.args.real_fixed_basket_joints), "--real-fixed-basket-joints"
        )
        if basket_joints is None:
            raise RuntimeError("--real-fixed-post-grasp requires --real-fixed-basket-joints")
        duration = max(float(self.args.real_fixed_post_grasp_duration), 0.2)
        targets: list[tuple[str, JointTrajectoryFrame]] = [(close_label, grasp_frame)]
        previous = grasp_frame
        for label, joints in (
            (
                "lift_safe",
                self._optional_real_fixed_joints(
                    str(self.args.real_fixed_lift_joints), "--real-fixed-lift-joints"
                ),
            ),
            (open_label, basket_joints),
            (
                "search_return",
                self._optional_real_fixed_joints(
                    str(self.args.real_fixed_search_joints), "--real-fixed-search-joints"
                ),
            ),
        ):
            if joints is None:
                continue
            previous = self._joint_target_after(previous, joints, duration)
            targets.append((label, previous))
        if bool(self.args.real_return_home):
            targets.append(("home", self._real_home_frame(previous, start_frame.positions)))
        return targets

    def _optional_real_fixed_joints(self, raw: str, label: str) -> tuple[float, ...] | None:
        if not str(raw).strip():
            return None
        values = tuple(_float_values(str(raw), 6, label))
        return values  # type: ignore[return-value]

    def _joint_target_after(
        self,
        previous_frame: JointTrajectoryFrame,
        positions: tuple[float, ...],
        duration: float,
    ) -> JointTrajectoryFrame:
        zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return JointTrajectoryFrame(
            float(previous_frame.time_from_start) + duration,
            positions,  # type: ignore[arg-type]
            zeros,
            zeros,
        )

    def _real_sdk_semantic_frame_indices(
        self, preview: GraspPreview, frames: list[JointTrajectoryFrame]
    ) -> dict[str, int]:
        waypoint_map = {name: xyz for name, xyz in preview.base_waypoints}
        planned_names = {
            name for name, _point, _progress in self._planning_target_samples(preview)
        }
        names = [
            name
            for name in ["approach", "grasp", "safe_mid", "basket_over", "drop"]
            if name in planned_names
        ]
        targets = {name: waypoint_map[name] for name in names if name in waypoint_map}
        if not targets:
            return {}
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return {}
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return {}
        base_from_aubo = self._matrix_from_transform(transform)
        grasp_points = []
        for frame in frames:
            tool_in_aubo = self.kinematics.fk(np.asarray(frame.positions, dtype=float))
            grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
            grasp_points.append(np.asarray(grasp_in_base[:3, 3], dtype=float))
        result: dict[str, int] = {}
        previous_index = 1
        for name in names:
            target = targets.get(name)
            if target is None:
                continue
            target_vec = np.asarray(target, dtype=float)
            start = min(max(previous_index, 1), len(frames) - 1)
            distances = [
                float(np.linalg.norm(point - target_vec)) if index >= start else float("inf")
                for index, point in enumerate(grasp_points)
            ]
            index = int(np.argmin(distances))
            if not np.isfinite(distances[index]):
                continue
            result[name] = index
            previous_index = min(max(index + 1, previous_index), len(frames) - 1)
        return result

    def _real_home_frame(
        self,
        previous_frame: JointTrajectoryFrame,
        positions: tuple[float, float, float, float, float, float] | None = None,
    ) -> JointTrajectoryFrame:
        if positions is None:
            positions = tuple(_float_values(str(self.args.real_home_joints), 6, "--real-home-joints"))  # type: ignore[assignment]
        duration = max(float(self.args.real_home_duration), 0.0)
        zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return JointTrajectoryFrame(
            float(previous_frame.time_from_start) + duration,
            positions,  # type: ignore[arg-type]
            zeros,
            zeros,
        )

    def _frame_index_at_or_after(self, frames: list[JointTrajectoryFrame], target_time: float) -> int:
        for index, frame in enumerate(frames):
            if float(frame.time_from_start) >= float(target_time):
                return index
        return len(frames) - 1

    def _real_frame_progress(
        self, frames: list[JointTrajectoryFrame], frame: JointTrajectoryFrame
    ) -> float:
        duration = max(float(frames[-1].time_from_start), 1e-6)
        return min(max(float(frame.time_from_start) / duration, 0.0), 1.0)

    def _normalise_sdk_joints(self, value: object) -> list[float]:
        if isinstance(value, dict):
            for key in ("joint_positions", "jointPositions", "positions", "q", "data", "value"):
                if key in value:
                    return self._normalise_sdk_joints(value[key])
        if isinstance(value, (list, tuple)) and len(value) >= 6:
            joints = [float(item) for item in value[:6]]
            if max(abs(item) for item in joints) > 10.0:
                raise RuntimeError(f"Aubo SDK joint values do not look like radians: {joints}")
            return joints
        raise RuntimeError(f"cannot parse Aubo SDK joint positions from {value!r}")

    def _real_sdk_joint_positions(self, rpc: AuboDirectJsonRpc) -> np.ndarray:
        return np.asarray(
            self._normalise_sdk_joints(rpc.robot_call("RobotState.getJointPositions")),
            dtype=float,
        )

    def _real_sdk_require_running(self, rpc: AuboDirectJsonRpc) -> None:
        mode = str(rpc.robot_call("RobotState.getRobotModeType")).strip().lower()
        safety = str(rpc.robot_call("RobotState.getSafetyModeType")).strip().lower()
        self.get_logger().info(f"Aubo SDK mode check: mode={mode} safety={safety}")
        if mode != "running" or safety not in ("normal", "reducedmode"):
            raise RuntimeError(
                "Aubo SDK execution requires Running/Normal or ReducedMode: "
                f"mode={mode} safety={safety}"
            )

    def _real_sdk_enter_gate(self) -> bool:
        path = Path(str(self.args.real_sdk_teach_flag_path))
        if path.exists():
            try:
                text = path.read_text(encoding="utf-8", errors="replace").strip()
            except OSError as exc:
                raise RuntimeError(f"real SDK teach gate is unreadable: {path}: {exc}") from exc
            owner_path = Path(str(self.args.real_sdk_control_owner_path))
            owner = str(self.args.real_sdk_control_owner_name).strip() or "grasp_task_server"
            if text.startswith("1") and self._real_sdk_owner_is_self(owner_path, owner):
                try:
                    path.unlink(missing_ok=True)
                    self.get_logger().info(
                        f"cleared stale real SDK teach gate before taking control: {path}"
                    )
                except OSError as exc:
                    raise RuntimeError(
                        f"stale real SDK teach gate could not be cleared: {path}: {exc}"
                    ) from exc
            else:
                raise RuntimeError(
                    "real SDK teach gate is already active; turn teach/jog off before grasp "
                    f"path={path} value={text!r}"
                )
        if path.exists():
            raise RuntimeError(
                "real SDK teach gate is already active; turn teach/jog off before grasp "
                f"path={path}"
            )
        path.write_text("1\n", encoding="utf-8")
        settle = max(float(self.args.real_sdk_gate_settle_sec), 0.0)
        if settle > 0.0:
            time.sleep(settle)
        return True

    def _real_sdk_owner_is_self(self, path: Path, owner: str) -> bool:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return False
        active_owner, pid = _parse_control_owner(text)
        return active_owner == owner and pid == os.getpid()

    def _real_sdk_enter_control_owner(self) -> bool:
        path = Path(str(self.args.real_sdk_control_owner_path))
        owner = str(self.args.real_sdk_control_owner_name).strip() or "grasp_task_server"
        ok, message = _claim_control_owner(path, owner)
        if not ok:
            raise RuntimeError(
                "Aubo control is busy; stop teach/manual jog before real grasp: "
                f"{message}"
            )
        self.get_logger().info(f"REAL Aubo control owner acquired: {message}")
        return True

    def _real_sdk_exit_control_owner(self, owner_owned: bool) -> None:
        if not owner_owned:
            return
        path = Path(str(self.args.real_sdk_control_owner_path))
        owner = str(self.args.real_sdk_control_owner_name).strip() or "grasp_task_server"
        _release_control_owner(path, owner)

    def _real_sdk_exit_gate(self, gate_owned: bool) -> None:
        if not gate_owned:
            return
        path = Path(str(self.args.real_sdk_teach_flag_path))
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().error(f"failed to release real SDK teach gate {path}: {exc}")

    def _real_sdk_exit_servo_mode(self, rpc: AuboDirectJsonRpc) -> None:
        try:
            result = rpc.robot_call("MotionControl.setServoModeSelect", [0])
            self.get_logger().info(f"Aubo SDK setServoModeSelect(0): result={result} error=None")
            if result not in (0, None):
                self.get_logger().warn(f"Aubo SDK setServoModeSelect(0) result={result}")
            return
        except Exception as exc:
            self.get_logger().warn(
                f"Aubo SDK setServoModeSelect unavailable, trying setServoMode(false): {exc}"
            )
        result = rpc.robot_call("MotionControl.setServoMode", [False])
        self.get_logger().info(f"Aubo SDK setServoMode(false): result={result} error=None")
        if result not in (0, None):
            self.get_logger().warn(f"Aubo SDK setServoMode(false) result={result}")

    def _real_sdk_stop_joint(
        self, rpc: AuboDirectJsonRpc, reason: str, *, warn_only: bool = False
    ) -> None:
        accel = max(float(self.args.real_sdk_move_accel), 0.05)
        try:
            result = rpc.robot_call("MotionControl.stopJoint", [accel])
            self.get_logger().info(
                f"Aubo SDK stopJoint during {reason}: result={result} error=None"
            )
        except Exception as exc:
            if warn_only:
                self.get_logger().warn(f"Aubo SDK stopJoint failed during {reason}: {exc}")
                return
            raise
        if result not in (0, None):
            message = f"Aubo SDK stopJoint result={result} during {reason}"
            if warn_only:
                self.get_logger().warn(message)
            else:
                raise RuntimeError(message)

    def _real_sdk_wait_arrival(
        self,
        rpc: AuboDirectJsonRpc,
        target: np.ndarray,
        label: str,
        planned_segment_dt: float,
    ) -> None:
        tolerance = max(float(self.args.real_sdk_goal_tolerance_rad), 0.001)
        speed = max(float(self.args.real_sdk_move_speed), 0.01)
        current = self._real_sdk_joint_positions(rpc)
        max_delta = float(np.max(np.abs(self._joint_delta(target, current))))
        timeout = max(
            planned_segment_dt,
            max_delta / speed,
            0.5,
        ) + max(float(self.args.real_sdk_arrival_timeout_padding), 0.0)
        deadline = time.monotonic() + timeout
        last_error = max_delta
        while rclpy.ok() and not self.stopping and time.monotonic() < deadline:
            current = self._real_sdk_joint_positions(rpc)
            last_error = float(np.max(np.abs(self._joint_delta(target, current))))
            if last_error <= tolerance:
                return
            time.sleep(0.05)
        raise TimeoutError(
            f"Aubo SDK moveJoint arrival timeout at {label}: "
            f"max_error={last_error:.3f}rad tolerance={tolerance:.3f}rad"
        )

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while rclpy.ok() and not self.stopping and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return bool(future.done())

    def _real_arm_joint_names(self) -> list[str]:
        names = [
            token.strip()
            for token in str(self.args.real_arm_joint_names).replace(";", ",").split(",")
            if token.strip()
        ]
        if len(names) != 6:
            raise ValueError("--real-arm-joint-names must contain 6 joint names")
        return names

    def _real_joint_value(self, name: str) -> float | None:
        if name in self.real_joint_positions:
            return float(self.real_joint_positions[name])
        alias = f"aubo_{name}" if not name.startswith("aubo_") else name.removeprefix("aubo_")
        if alias in self.real_joint_positions:
            return float(self.real_joint_positions[alias])
        return None

    def _real_synced_start_values(self) -> list[float] | None:
        raw = str(self.args.real_start_joints).strip()
        if not raw:
            return None
        try:
            return _float_values(raw, 6, "--real-start-joints")
        except ValueError as exc:
            self.get_logger().error(f"invalid synchronized real start joints: {exc}")
            return None

    def _real_start_values(self) -> tuple[list[float] | None, str]:
        names = self._real_arm_joint_names()
        timeout = max(float(self.args.real_start_state_timeout), 0.0)
        deadline = time.monotonic() + timeout
        values: list[float | None] = []
        while rclpy.ok() and not self.stopping and time.monotonic() <= deadline:
            values = [self._real_joint_value(name) for name in names]
            if all(value is not None for value in values):
                break
            time.sleep(0.05)
        if values and all(value is not None for value in values):
            return [float(value) for value in values], str(self.args.real_joint_states_topic)

        missing = [name for name, value in zip(names, values) if value is None] if values else names
        synced = self._real_synced_start_values()
        if synced is not None:
            self.get_logger().warn(
                "real joint states incomplete; using synchronized start pose from "
                f"ARACHNE_GRASP_ARM_JOINTS. missing_on_{self.args.real_joint_states_topic}={missing}"
            )
            return synced, "ARACHNE_GRASP_ARM_JOINTS"
        try:
            with AuboDirectJsonRpc(
                str(self.args.real_sdk_ip),
                int(self.args.real_sdk_rpc_port),
                max(float(self.args.real_sdk_rpc_timeout), 0.1),
            ) as rpc:
                sdk_values = self._real_sdk_joint_positions(rpc)
            self.get_logger().warn(
                "real joint states incomplete; using Aubo SDK read-only current joints "
                f"for start check. missing_on_{self.args.real_joint_states_topic}={missing}"
            )
            return [float(value) for value in sdk_values[:6]], "Aubo SDK RobotState.getJointPositions"
        except Exception as exc:
            self.get_logger().warn(f"Aubo SDK read-only joint fallback failed: {exc}")
        self.get_logger().error(
            "real execution blocked: missing real joint states "
            f"on {self.args.real_joint_states_topic}: {missing}"
        )
        return None, str(self.args.real_joint_states_topic)

    def _rebase_preview_start_to_real(
        self,
        preview: GraspPreview,
        actual_values: list[float],
        source: str,
    ) -> GraspPreview:
        frames = list(preview.arm_trajectory_frames)
        if not frames:
            return preview
        planned = np.asarray(frames[0].positions, dtype=float)
        actual = np.asarray(actual_values, dtype=float)
        deltas = np.abs([self._wrap_angle(float(p - a)) for p, a in zip(planned, actual)])
        max_delta = float(np.max(deltas))
        if max_delta <= max(float(self.args.real_start_tolerance_rad), 0.0):
            return preview
        self.get_logger().warn(
            "real trajectory start rebased to current arm state "
            f"source={source} planned_start_delta={max_delta:.3f}rad"
        )
        zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        frames[0] = JointTrajectoryFrame(
            0.0,
            tuple(float(value) for value in actual_values),  # type: ignore[arg-type]
            zero,
            zero,
        )
        return replace(preview, arm_trajectory_frames=frames)

    def _real_start_state_matches(
        self,
        frame: JointTrajectoryFrame,
        *,
        actual_values: list[float] | None = None,
        source: str | None = None,
    ) -> bool:
        if actual_values is None:
            actual_values, source = self._real_start_values()
        if actual_values is None:
            return False
        source = source or str(self.args.real_joint_states_topic)
        planned = np.asarray(frame.positions, dtype=float)
        actual = np.asarray(actual_values, dtype=float)
        deltas = np.abs([self._wrap_angle(float(p - a)) for p, a in zip(planned, actual)])
        max_delta = float(np.max(deltas))
        tolerance = max(float(self.args.real_start_tolerance_rad), 0.0)
        if max_delta > tolerance:
            names = self._real_arm_joint_names()
            detail = ", ".join(
                f"{name}={delta:.3f}" for name, delta in zip(names, deltas)
            )
            self.get_logger().error(
                "real execution blocked: real arm is not at planned start "
                f"source={source} max_delta={max_delta:.3f}rad "
                f"tolerance={tolerance:.3f}rad deltas=[{detail}]"
            )
            return False
        self.get_logger().warn(
            f"real execution start-state check passed via {source}: max_delta={max_delta:.3f}rad"
        )
        return True

    def _real_joint_trajectory_msg(
        self, frames: list[JointTrajectoryFrame]
    ) -> JointTrajectory:
        stride = max(int(self.args.real_trajectory_point_stride), 1)
        selected = list(frames[::stride])
        if selected[-1] is not frames[-1]:
            selected.append(frames[-1])

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self._real_arm_joint_names()
        for frame in selected:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in frame.positions]
            point.velocities = [float(value) for value in frame.velocities]
            point.accelerations = [float(value) for value in frame.accelerations]
            point.time_from_start.sec = int(frame.time_from_start)
            point.time_from_start.nanosec = int((frame.time_from_start % 1.0) * 1e9)
            trajectory.points.append(point)
        return trajectory

    def _rebased_trajectory_frames(
        self, frames: list[JointTrajectoryFrame]
    ) -> list[JointTrajectoryFrame]:
        if not frames:
            return []
        start_time = float(frames[0].time_from_start)
        rebased: list[JointTrajectoryFrame] = []
        for index, frame in enumerate(frames):
            time_from_start = max(float(frame.time_from_start) - start_time, 0.0)
            velocities = tuple(float(value) for value in frame.velocities)
            accelerations = tuple(float(value) for value in frame.accelerations)
            if index == 0 or index == len(frames) - 1:
                velocities = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                accelerations = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            rebased.append(
                JointTrajectoryFrame(
                    time_from_start,
                    tuple(float(value) for value in frame.positions),  # type: ignore[arg-type]
                    velocities,  # type: ignore[arg-type]
                    accelerations,  # type: ignore[arg-type]
                )
            )
        return rebased

    def _publish_real_gripper(self, command: str) -> None:
        if self.real_gripper_pub is None:
            return
        msg = String()
        msg.data = command
        self.real_gripper_pub.publish(msg)
        normalized = command.strip().lower()
        if normalized == "open":
            self.real_gripper_preopened = True
            self.real_gripper_preopen_time = time.monotonic()
        elif normalized in {"close", "stop"}:
            self.real_gripper_preopened = False
            self.real_gripper_preopen_time = 0.0
        self.get_logger().warn(f"REAL gripper command: {command}")

    def _real_preopen_gripper_enabled(self) -> bool:
        return (
            bool(self.args.execute_real)
            and bool(self.args.real_execute_gripper)
            and bool(self.args.real_preopen_gripper)
            and str(self.args.execute_real_confirm).strip().upper() == "YES"
        )

    def _preopen_real_gripper_for_request(self) -> None:
        if not self._real_preopen_gripper_enabled():
            return
        self.get_logger().info("REAL gripper preopen requested")
        self._publish_real_gripper("open")

    def _open_real_gripper_before_motion(self) -> None:
        if not bool(self.args.real_execute_gripper):
            return
        settle = max(float(self.args.real_gripper_settle_sec), 0.0)
        if self._real_preopen_gripper_enabled() and self.real_gripper_preopened:
            elapsed = max(time.monotonic() - self.real_gripper_preopen_time, 0.0)
            remaining = settle - elapsed
            if remaining > 0.0:
                self.get_logger().warn(
                    f"REAL gripper preopen settling before arm motion: remaining={remaining:.2f}s"
                )
                time.sleep(remaining)
            else:
                self.get_logger().warn("REAL gripper preopen settled; skipping pre-motion open wait")
            return
        self._publish_real_gripper("open")
        if settle > 0.0:
            time.sleep(settle)

    def _latest_gripper_status(self) -> tuple[float, str] | None:
        with self.gripper_status_lock:
            return self.latest_real_gripper_status

    def _wait_real_gripper_capture(self) -> tuple[bool, str]:
        if not bool(self.args.real_gripper_require_capture):
            return True, "capture check disabled"
        timeout = max(float(self.args.real_gripper_capture_timeout), 0.0)
        deadline = time.monotonic() + timeout
        last_status = ""
        last_decision: tuple[bool | None, str] = (None, "waiting for gripper feedback")
        while rclpy.ok() and not self.stopping and time.monotonic() <= deadline:
            self._publish_real_gripper("status")
            time.sleep(0.12)
            item = self._latest_gripper_status()
            if item is None:
                continue
            age = time.monotonic() - item[0]
            if age > max(timeout, 0.5) + 0.5:
                continue
            last_status = item[1]
            decision, message = self._classify_real_gripper_capture(last_status)
            last_decision = (decision, message)
            if decision is not None:
                return bool(decision), message
        decision, message = last_decision
        if decision is not None:
            return bool(decision), message
        if bool(self.args.real_gripper_capture_unknown_ok):
            return True, f"capture feedback unknown; assuming captured: {last_status or message}"
        return False, f"capture feedback unavailable: {last_status or message}"

    def _classify_real_gripper_capture(self, status: str) -> tuple[bool | None, str]:
        text = str(status).strip()
        if not text:
            return None, "empty gripper status"
        if "feedback_timeout" in text or "feedback_bad_checksum" in text:
            return None, text
        reached = self._status_number(text, "reached")
        speed = self._status_number(text, "speed")
        angle = self._status_number(text, "angle")
        if angle is None and reached is None and speed is None:
            return None, text
        min_angle = float(self.args.real_gripper_capture_min_angle)
        empty_angle = float(self.args.real_gripper_empty_close_angle)
        if reached == 0 and speed == 0:
            return True, f"gripper early-stop capture: {text}"
        if angle is not None and angle >= min_angle:
            return True, f"gripper stopped before empty-close angle: {text}"
        if reached == 1 and angle is not None and angle <= empty_angle:
            return False, f"gripper_capture_failed: empty close detected: {text}"
        if speed is not None and speed > 0:
            return None, f"gripper still moving: {text}"
        return None, text

    def _status_number(self, text: str, name: str) -> float | None:
        match = re.search(rf"\b{name}=(-?\d+(?:\.\d+)?)", text)
        if match is None:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _require_real_gripper_capture_or_raise(self) -> None:
        ok, message = self._wait_real_gripper_capture()
        if ok:
            self.get_logger().warn(f"REAL gripper capture check ok: {message}")
            return
        self.get_logger().error(message)
        self._publish_real_gripper("open")
        raise RuntimeError(message)

    def _draw_locked_detection(self, image: np.ndarray, preview: GraspPreview) -> None:
        self._draw_detection_overlay(image, preview.detection, "LOCKED")
        cv2.putText(
            image,
            f"YOLO paused; yolo_calls={self.inference_count}; publish restart_search to search again",
            (12, max(image.shape[0] - 18, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (0, 190, 255),
            2,
            cv2.LINE_AA,
        )

    def _publish_locked_annotation(self, color: np.ndarray, header: Header) -> None:
        if self.last_preview is None:
            return
        now = time.monotonic()
        self.locked_visual_last_publish = now
        annotated = color.copy()
        self._draw_locked_detection(annotated, self.last_preview)
        self._publish_annotated(annotated, self.last_preview, header)

    def _predict(self, frame: np.ndarray):
        self.inference_count += 1
        device = self._yolo_device()
        kwargs = {
            "imgsz": int(self.args.imgsz),
            "conf": float(self.args.conf),
            "device": device,
            "verbose": False,
        }
        if self.class_ids is not None:
            kwargs["classes"] = self.class_ids
        return self.model.predict(frame, **kwargs)[0]

    def _yolo_device(self) -> str:
        if self.model_path.suffix.lower() == ".onnx":
            device = str(self.args.onnx_device).strip()
            if device:
                return device
        return str(self.args.device_id)

    def _best_detection(self, result) -> Detection | None:
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return None
        confs = boxes.conf.cpu().numpy()
        xyxys = boxes.xyxy.cpu().numpy()
        classes = boxes.cls.cpu().numpy()
        masks = getattr(result, "masks", None)
        polygons = getattr(masks, "xy", None) if masks is not None else None
        image_h, image_w = getattr(result, "orig_shape", (0, 0))[:2]
        edge_margin = max(float(self.args.detection_edge_margin_ratio), 0.0)
        edge_x = float(image_w) * edge_margin
        edge_y = float(image_h) * edge_margin
        min_center_y = float(self.args.detection_min_center_y_ratio)
        max_center_y = float(self.args.detection_max_center_y_ratio)
        min_mask_area = max(float(self.args.min_detection_mask_area_px), 0.0)
        reject_words = [
            token.strip().lower()
            for token in str(self.args.reject_label_keywords).split(",")
            if token.strip()
        ]
        preferred_words = [
            token.strip().lower()
            for token in str(self.args.preferred_label_keywords).split(",")
            if token.strip()
        ]

        reject_reasons: dict[str, int] = {}
        reject_examples: list[str] = []
        best: tuple[float, int, tuple[tuple[float, float], ...] | None, float] | None = None
        for index in range(len(boxes)):
            xyxy = tuple(float(v) for v in xyxys[index])
            class_id = int(classes[index])
            confidence = float(confs[index])
            label = str(result.names.get(class_id, class_id))
            label_l = label.lower()

            def reject(reason: str, detail: str) -> None:
                reject_reasons[reason] = reject_reasons.get(reason, 0) + 1
                if len(reject_examples) < 2:
                    reject_examples.append(
                        f"{label}:{confidence:.2f} {reason} {detail}".strip()
                    )

            matched_reject_word = next((word for word in reject_words if word in label_l), "")
            if matched_reject_word:
                reject("label", f"matched={matched_reject_word}")
                continue
            x1, y1, x2, y2 = xyxy
            if image_w > 0 and image_h > 0:
                if x1 < edge_x or y1 < edge_y or x2 > float(image_w) - edge_x or y2 > float(image_h) - edge_y:
                    reject("edge", f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})")
                    continue
            mask_xy: tuple[tuple[float, float], ...] | None = None
            mask_area_px = 0.0
            if polygons is not None and len(polygons) > index:
                points = np.asarray(polygons[index], dtype=np.float32)
                if points.ndim == 2 and points.shape[0] >= 3 and points.shape[1] >= 2:
                    points = points[:, :2]
                    mask_xy = tuple((float(x), float(y)) for x, y in points)
                    mask_area_px = float(abs(cv2.contourArea(points)))
            box_area = max((x2 - x1) * (y2 - y1), 0.0)
            area_px = max(mask_area_px, box_area)
            if area_px < min_mask_area:
                reject("area", f"{area_px:.0f}px<{min_mask_area:.0f}px")
                continue
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            if image_h > 0:
                center_y_ratio = cy / float(image_h)
                if center_y_ratio < min_center_y or center_y_ratio > max_center_y:
                    reject(
                        "center_y",
                        f"{center_y_ratio:.2f} not {min_center_y:.2f}-{max_center_y:.2f}",
                    )
                    continue
            center_penalty = 0.0
            if image_w > 0 and image_h > 0:
                center_penalty = (
                    abs(cx - image_w * 0.5) / max(float(image_w), 1.0)
                    + abs(cy - image_h * 0.5) / max(float(image_h), 1.0)
                )
            if "bottle" in label_l and "cap" not in label_l:
                label_bonus = 0.55
            elif "carton" in label_l:
                label_bonus = 0.20
            else:
                label_bonus = 0.15 if any(word in label_l for word in preferred_words) else 0.0
            score = confidence + label_bonus + min(area_px / 12000.0, 0.25) - 0.25 * center_penalty
            if best is None or score > best[0]:
                best = (score, index, mask_xy, mask_area_px)
        if best is None:
            rejected = sum(reject_reasons.values())
            reasons = ",".join(f"{key}={value}" for key, value in sorted(reject_reasons.items()))
            examples = "; ".join(reject_examples)
            message = f"YOLO detections filtered out: total={len(boxes)} rejected={rejected}"
            if reasons:
                message += f" reasons={reasons}"
            if examples:
                message += f" examples=[{examples}]"
            self._throttled_log(message)
            return None
        _score, best_idx, mask_xy, mask_area_px = best
        xyxy = tuple(float(v) for v in xyxys[best_idx])
        class_id = int(classes[best_idx])
        confidence = float(confs[best_idx])
        label = str(result.names.get(class_id, class_id))
        return Detection(
            label=label,
            class_id=class_id,
            confidence=confidence,
            xyxy=xyxy,
            mask_xy=mask_xy,
            mask_area_px=mask_area_px,
        )

    def _publish_detection_event(self, detection: Detection, header: Header) -> None:
        payload = {
            "source": "grasp_preview_yolo_detect",
            "has_3d": False,
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "instances": [
                {
                    "label": detection.label,
                    "class_name": detection.label,
                    "taco_class": detection.label,
                    "class_id": int(detection.class_id),
                    "confidence": float(detection.confidence),
                    "bbox_xyxy": [float(value) for value in detection.xyxy],
                    "has_mask": detection.mask_xy is not None,
                    "mask_area_px": float(detection.mask_area_px),
                }
            ],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.detection_pub.publish(msg)

    def _publish_preview_detection_event(self, preview: GraspPreview, header: Header) -> None:
        instance = {
            "label": preview.detection.label,
            "class_name": preview.detection.label,
            "taco_class": preview.detection.label,
            "class_id": int(preview.detection.class_id),
            "confidence": float(preview.detection.confidence),
            "bbox_xyxy": [float(value) for value in preview.detection.xyxy],
            "has_mask": preview.detection.mask_xy is not None,
            "mask_area_px": float(preview.detection.mask_area_px),
            "has_3d": True,
            "depth_m": float(preview.depth_m),
            "grasp_camera_xyz": [float(value) for value in preview.grasp_xyz],
            "base_grasp_xyz": (
                [float(value) for value in preview.base_grasp_xyz]
                if preview.base_grasp_xyz is not None
                else []
            ),
            "waypoints_base": [
                {"name": name, "xyz": [float(value) for value in xyz]}
                for name, xyz in preview.base_waypoints
            ],
            "planning_key_waypoints": [
                {"name": name, "xyz": [float(value) for value in xyz]}
                for name, xyz, _progress in self._planning_target_samples(preview)
            ],
            "pointcloud_grasp_shape": self._pointcloud_shape_payload(preview.pointcloud_shape),
            "trajectory_message": str(preview.ik_message),
        }
        payload = {
            "source": "grasp_preview_3d",
            "has_3d": True,
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "instances": [instance],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.detection_pub.publish(msg)

    def _needs_depth_snapshot(
        self, detection: Detection, image_shape: tuple[int, ...]
    ) -> tuple[bool, str]:
        if self.last_preview is None:
            return True, "new-target"
        previous = self.snapshot_detection or self.last_preview.detection
        if detection.class_id != previous.class_id:
            return True, "class-change"
        iou = self._bbox_iou(previous.xyxy, detection.xyxy)
        shift = self._normalized_center_shift(previous.xyxy, detection.xyxy, image_shape)
        if iou < float(self.args.snapshot_iou_threshold):
            return True, f"bbox-iou={iou:.2f}"
        if shift > float(self.args.snapshot_center_shift):
            return True, f"bbox-shift={shift:.2f}"
        return False, "cached"

    def _bbox_iou(
        self, a: tuple[float, float, float, float], b: tuple[float, float, float, float]
    ) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        inter = max(ix2 - ix1, 0.0) * max(iy2 - iy1, 0.0)
        area_a = max(ax2 - ax1, 0.0) * max(ay2 - ay1, 0.0)
        area_b = max(bx2 - bx1, 0.0) * max(by2 - by1, 0.0)
        union = area_a + area_b - inter
        return float(inter / union) if union > 1e-6 else 0.0

    def _normalized_center_shift(
        self,
        a: tuple[float, float, float, float],
        b: tuple[float, float, float, float],
        image_shape: tuple[int, ...],
    ) -> float:
        height = max(float(image_shape[0]), 1.0)
        width = max(float(image_shape[1]), 1.0)
        ax = 0.5 * (a[0] + a[2])
        ay = 0.5 * (a[1] + a[3])
        bx = 0.5 * (b[0] + b[2])
        by = 0.5 * (b[1] + b[3])
        diag = max((width * width + height * height) ** 0.5, 1.0)
        return float(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5 / diag)

    def _make_preview(
        self, detection: Detection, depth: np.ndarray, snapshot_reason: str
    ) -> GraspPreview | None:
        start = time.monotonic()
        info = self.depth_info
        if info is None:
            return None
        dh, dw = depth.shape[:2]
        cw = max(int(self.latest_color.width if self.latest_color is not None else dw), 1)
        ch = max(int(self.latest_color.height if self.latest_color is not None else dh), 1)
        x1, y1, x2, y2 = detection.xyxy
        sx = dw / float(cw)
        sy = dh / float(ch)
        x1d, x2d = x1 * sx, x2 * sx
        y1d, y2d = y1 * sy, y2 * sy
        mask_depth = self._detection_depth_mask(detection, dw, dh, cw, ch)
        mask_done = time.monotonic()

        cx = 0.5 * (x1d + x2d)
        cy = 0.5 * (y1d + y2d)
        shrink = min(max(float(self.args.roi_shrink), 0.15), 1.0)
        half_w = max(2.0, 0.5 * (x2d - x1d) * shrink)
        half_h = max(2.0, 0.5 * (y2d - y1d) * shrink)
        ix1 = int(np.clip(cx - half_w, 0, dw - 1))
        ix2 = int(np.clip(cx + half_w, ix1 + 1, dw))
        iy1 = int(np.clip(cy - half_h, 0, dh - 1))
        iy2 = int(np.clip(cy + half_h, iy1 + 1, dh))
        roi = depth[iy1:iy2, ix1:ix2].astype(np.float32) * float(self.args.depth_scale)
        valid_mask = (roi >= self.args.min_depth) & (roi <= self.args.max_depth)
        mask_roi = None
        if mask_depth is not None:
            candidate = mask_depth[iy1:iy2, ix1:ix2]
            if int(np.count_nonzero(candidate)) >= 8:
                mask_roi = candidate
                valid_mask &= mask_roi
        roi_source = "mask" if mask_roi is not None else "bbox"
        valid = roi[valid_mask]
        if valid.size < 8:
            return None
        roi_done = time.monotonic()

        depth_m = float(np.percentile(valid, float(self.args.depth_percentile)))
        percentile_done = time.monotonic()
        roi_points_raw = self._roi_points(
            depth, ix1, iy1, ix2, iy2, depth_m, mask_depth if mask_roi is not None else None
        )
        points_done = time.monotonic()
        roi_points = self._filter_roi_points_by_base_z(roi_points_raw)
        if not roi_points:
            return None
        filter_done = time.monotonic()
        visual_axis_camera, visual_axis_confidence = self._detection_visual_axis_camera(
            detection, depth_m, dw, dh, cw, ch
        )
        axis_done = time.monotonic()
        # Grasp target at the ROI point-cloud median (object geometry center).
        # Do NOT project the bbox center pixel: for low/flat objects seen from
        # above-front (e.g. a lying bottle), the visible contour is asymmetric
        # (near end-face visible, far end-face occluded), so the bbox center
        # lies on the near side of the object and the projected grasp would
        # land on its near end.
        roi_arr = np.asarray(roi_points, dtype=np.float64)
        grasp_x, grasp_y, grasp_z = (float(v) for v in np.median(roi_arr, axis=0))

        # NOTE: do NOT apply grasp_standoff here. This z is along the camera
        # optical axis (roughly horizontal, tilted down); subtracting standoff
        # would pull the target toward the AGV side by ~3cm for top-down grasps,
        # landing on the near edge of the object. The vertical "standoff from
        # surface" compensation is done in base frame via grasp_final_z_offset_m.
        grasp_z = max(grasp_z, self.args.min_depth)
        grasp = (grasp_x, grasp_y, grasp_z)
        pregrasp = (
            grasp_x,
            grasp_y,
            max(grasp_z - float(self.args.approach_distance), self.args.min_depth * 0.5),
        )
        lift = (grasp_x, grasp_y - float(self.args.lift_distance), grasp_z)
        retreat = (pregrasp[0], pregrasp[1] - float(self.args.lift_distance), pregrasp[2])
        bbox_points = [
            self._pixel_to_xyz(x1d, y1d, depth_m),
            self._pixel_to_xyz(x2d, y1d, depth_m),
            self._pixel_to_xyz(x2d, y2d, depth_m),
            self._pixel_to_xyz(x1d, y2d, depth_m),
        ]
        camera_points_done = time.monotonic()
        (
            _base_path,
            base_segments,
            base_waypoints,
            base_grasp,
            pointcloud_shape,
        ) = self._make_base_path(
            [pregrasp, grasp, lift, retreat],
            roi_points,
            visual_axis_camera,
            visual_axis_confidence,
        )
        base_path_done = time.monotonic()
        if base_grasp is not None:
            min_z = float(self.args.yolo_depth_min_base_z)
            max_z = float(self.args.yolo_depth_max_base_z)
            z_epsilon = 0.005
            if base_grasp[2] < min_z - z_epsilon:
                self._throttled_log(
                    f"reject detection below YOLO depth threshold: "
                    f"label={detection.label} base_z={base_grasp[2]:.3f} "
                    f"min_z={min_z:.3f}"
                )
                return None
            if base_grasp[2] > max_z + z_epsilon:
                self._throttled_log(
                    f"reject detection above YOLO depth threshold: "
                    f"label={detection.label} base_z={base_grasp[2]:.3f} "
                    f"max_z={max_z:.3f}"
                )
                return None
        self.get_logger().info(
            "grasp_timing make_preview "
            f"label={detection.label} reason={snapshot_reason} "
            f"total={base_path_done - start:.3f}s "
            f"mask={mask_done - start:.3f}s "
            f"roi_depth={roi_done - mask_done:.3f}s "
            f"percentile={percentile_done - roi_done:.3f}s "
            f"roi_points={points_done - percentile_done:.3f}s "
            f"base_z_filter={filter_done - points_done:.3f}s "
            f"visual_axis={axis_done - filter_done:.3f}s "
            f"camera_waypoints={camera_points_done - axis_done:.3f}s "
            f"base_path={base_path_done - camera_points_done:.3f}s "
            f"roi_raw={len(roi_points_raw)} roi_filtered={len(roi_points)}"
        )
        return GraspPreview(
            detection=detection,
            depth_m=depth_m,
            grasp_xyz=grasp,
            pregrasp_xyz=pregrasp,
            lift_xyz=lift,
            retreat_xyz=retreat,
            roi_points=roi_points,
            roi_source=roi_source,
            bbox_points=bbox_points,
            base_path_xyz=_base_path if str(self.args.planner_backend) == "none" else [],
            base_trajectory_segments=base_segments,
            base_waypoints=base_waypoints,
            arm_trajectory_frames=[],
            basket_safe=True,
            base_grasp_xyz=base_grasp,
            pointcloud_shape=pointcloud_shape,
            snapshot_reason=snapshot_reason,
            ik_message="planning pending",
        )

    def _detection_depth_mask(
        self, detection: Detection, depth_w: int, depth_h: int, color_w: int, color_h: int
    ) -> np.ndarray | None:
        if not detection.mask_xy:
            return None
        points = np.asarray(detection.mask_xy, dtype=np.float32)
        if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] < 2:
            return None
        points = points[:, :2].copy()
        points[:, 0] *= float(depth_w) / max(float(color_w), 1.0)
        points[:, 1] *= float(depth_h) / max(float(color_h), 1.0)
        points[:, 0] = np.clip(points[:, 0], 0, max(depth_w - 1, 0))
        points[:, 1] = np.clip(points[:, 1], 0, max(depth_h - 1, 0))
        mask = np.zeros((depth_h, depth_w), dtype=np.uint8)
        cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
        return mask.astype(bool)

    def _roi_points(
        self,
        depth: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        depth_m: float,
        mask: np.ndarray | None = None,
    ) -> list[tuple[float, float, float]]:
        step = max(int(self.args.roi_decimation), 1)
        band = max(float(self.args.depth_band), 0.01)
        xs = np.arange(x1, x2, step, dtype=np.float32)
        ys = np.arange(y1, y2, step, dtype=np.float32)
        if xs.size == 0 or ys.size == 0:
            return []
        uu, vv = np.meshgrid(xs, ys)
        xi = uu.astype(np.int32, copy=False)
        yi = vv.astype(np.int32, copy=False)
        z = depth[yi, xi].astype(np.float32) * float(self.args.depth_scale)
        valid = (
            np.isfinite(z)
            & (z >= float(self.args.min_depth))
            & (z <= float(self.args.max_depth))
            & (np.abs(z - float(depth_m)) <= band)
        )
        if mask is not None:
            valid &= mask[yi, xi]
        if not np.any(valid):
            return []
        points = self._pixels_to_xyz(uu[valid], vv[valid], z[valid])
        return [tuple(float(value) for value in point) for point in points]

    def _filter_roi_points_by_base_z(
        self, points: list[tuple[float, float, float]]
    ) -> list[tuple[float, float, float]]:
        if not bool(self.args.yolo_depth_filter) or not points or self.latest_depth is None:
            return points
        base_from_depth = self._base_from_depth_matrix(self.latest_depth.header, timeout_sec=0.02)
        if base_from_depth is None:
            return points
        arr = np.asarray(points, dtype=np.float32)
        base_z = arr @ base_from_depth[2, :3] + base_from_depth[2, 3]
        keep = (
            (base_z >= float(self.args.yolo_depth_min_base_z))
            & (base_z <= float(self.args.yolo_depth_max_base_z))
        )
        return [tuple(float(value) for value in point) for point in arr[keep]]

    def _detection_visual_axis_camera(
        self,
        detection: Detection,
        depth_m: float,
        depth_w: int,
        depth_h: int,
        color_w: int,
        color_h: int,
    ) -> tuple[np.ndarray | None, float]:
        if detection.mask_xy and len(detection.mask_xy) >= 6:
            pixels = np.asarray(detection.mask_xy, dtype=np.float64)
            pixels[:, 0] *= float(depth_w) / max(float(color_w), 1.0)
            pixels[:, 1] *= float(depth_h) / max(float(color_h), 1.0)
        else:
            x1, y1, x2, y2 = detection.xyxy
            sx = float(depth_w) / max(float(color_w), 1.0)
            sy = float(depth_h) / max(float(color_h), 1.0)
            pixels = np.asarray(
                [
                    (x1 * sx, y1 * sy),
                    (x2 * sx, y1 * sy),
                    (x2 * sx, y2 * sy),
                    (x1 * sx, y2 * sy),
                ],
                dtype=np.float64,
            )
        if pixels.ndim != 2 or pixels.shape[0] < 3:
            return None, 0.0
        center = np.mean(pixels[:, :2], axis=0)
        centered = pixels[:, :2] - center
        cov = np.cov(centered.T)
        if not np.all(np.isfinite(cov)):
            return None, 0.0
        eig_values, eig_vectors = np.linalg.eigh(cov)
        order = np.argsort(eig_values)[::-1]
        eig_values = eig_values[order]
        eig_vectors = eig_vectors[:, order]
        if float(eig_values[0]) <= 1e-6:
            return None, 0.0
        axis_px = eig_vectors[:, 0]
        axis_px /= max(float(np.linalg.norm(axis_px)), 1e-9)
        if axis_px[0] < -1e-9 or (abs(axis_px[0]) < 1e-9 and axis_px[1] < 0.0):
            axis_px *= -1.0
        spread = np.sqrt(max(float(eig_values[0]), 1.0))
        delta_px = float(np.clip(spread * 0.35, 6.0, 28.0))
        p0 = center
        p1 = center + axis_px * delta_px
        xyz0 = np.asarray(self._pixel_to_xyz(float(p0[0]), float(p0[1]), depth_m), dtype=float)
        xyz1 = np.asarray(self._pixel_to_xyz(float(p1[0]), float(p1[1]), depth_m), dtype=float)
        axis_camera = xyz1 - xyz0
        norm = float(np.linalg.norm(axis_camera))
        if norm < 1e-6:
            return None, 0.0
        anisotropy = float((eig_values[0] - eig_values[1]) / max(eig_values[0] + eig_values[1], 1e-9))
        return axis_camera / norm, float(np.clip(anisotropy, 0.0, 1.0))

    def _pixel_to_xyz(self, u: float, v: float, z: float) -> tuple[float, float, float]:
        points = self._pixels_to_xyz(
            np.asarray([u], dtype=np.float32),
            np.asarray([v], dtype=np.float32),
            np.asarray([z], dtype=np.float32),
        )
        if points.size == 0:
            return (0.0, 0.0, 0.0)
        return tuple(float(value) for value in points[0])

    def _pixels_to_xyz(self, u: np.ndarray, v: np.ndarray, z: np.ndarray) -> np.ndarray:
        info = self.depth_info
        if info is None:
            return np.empty((0, 3), dtype=np.float32)
        fx = float(info.k[0]) if info.k[0] else float(info.width) * 0.9
        fy = float(info.k[4]) if info.k[4] else fx
        cx = float(info.k[2]) if info.k[2] else (float(info.width) - 1.0) * 0.5
        cy = float(info.k[5]) if info.k[5] else (float(info.height) - 1.0) * 0.5
        pixel_x = cx - u if bool(self.args.depth_projection_flip_x) else u - cx
        pixel_y = cy - v if bool(self.args.depth_projection_flip_y) else v - cy
        x = pixel_x.astype(np.float32, copy=False) * z / fx
        y = pixel_y.astype(np.float32, copy=False) * z / fy
        return np.column_stack((x, y, z)).astype(np.float32, copy=False)

    def _publish_preview_async(
        self, preview: GraspPreview, source_header: Header, *, label: str
    ) -> None:
        threading.Thread(
            target=self._publish_preview_worker,
            args=(preview, source_header, label),
            daemon=True,
        ).start()

    def _publish_preview_worker(
        self, preview: GraspPreview, source_header: Header, label: str
    ) -> None:
        if self.stopping or not rclpy.ok():
            return
        start = time.monotonic()
        try:
            with self.preview_output_lock:
                self._publish_preview(preview, source_header)
        except RuntimeError as exc:
            if "publisher's context is invalid" in str(exc):
                return
            self.get_logger().error(f"async preview publish failed: {exc}")
            return
        except Exception as exc:
            self.get_logger().error(f"async preview publish failed: {type(exc).__name__}: {exc}")
            return
        done = time.monotonic()
        self.get_logger().info(
            f"grasp_timing preview_publish_done label={label} total={done - start:.3f}s"
        )

    def _publish_and_save_preview_async(
        self,
        preview: GraspPreview,
        preview_header: Header,
        annotated: np.ndarray,
        raw: np.ndarray,
        image_header: Header,
    ) -> None:
        threading.Thread(
            target=self._publish_and_save_preview_worker,
            args=(preview, preview_header, annotated, raw, image_header),
            daemon=True,
        ).start()

    def _publish_and_save_preview_worker(
        self,
        preview: GraspPreview,
        preview_header: Header,
        annotated: np.ndarray,
        raw: np.ndarray,
        image_header: Header,
    ) -> None:
        if self.stopping or not rclpy.ok():
            return
        start = time.monotonic()
        try:
            with self.preview_output_lock:
                publish_start = time.monotonic()
                self._publish_preview(preview, preview_header)
                self._publish_annotated(annotated, preview, image_header)
                publish_done = time.monotonic()
                self._save_latest(annotated, raw, preview)
                save_done = time.monotonic()
        except RuntimeError as exc:
            if "publisher's context is invalid" in str(exc):
                return
            self.get_logger().error(f"async preview output failed: {exc}")
            return
        except Exception as exc:
            self.get_logger().error(f"async preview output failed: {type(exc).__name__}: {exc}")
            return
        self.get_logger().info(
            "grasp_timing preview_output_done "
            f"label={preview.detection.label} reason={preview.snapshot_reason} "
            f"total={save_done - start:.3f}s "
            f"publish={publish_done - publish_start:.3f}s "
            f"save={save_done - publish_done:.3f}s"
        )

    def _publish_preview(self, preview: GraspPreview, source_header: Header) -> None:
        # ponytail: preview markers/clouds need current TF; keep coordinates in the camera frame.
        header = _header(self.get_clock().now().to_msg(), source_header.frame_id)
        if preview.base_path_xyz:
            preview.basket_safe = self._basket_path_safe(preview.base_path_xyz)
        self._publish_preview_detection_event(preview, header)
        self.markers_pub.publish(self._markers(preview, header))
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, preview.roi_points))
        self.path_pub.publish(self._path(preview, header))
        self._publish_preview_arm_state(preview)
        now = time.monotonic()
        if now - self.last_log_time > 0.75:
            self.last_log_time = now
            gx, gy, gz = preview.grasp_xyz
            shape_msg = "shape=none"
            if preview.pointcloud_shape is not None:
                shape = preview.pointcloud_shape
                shape_msg = (
                    f"shape=pc points={shape.point_count} conf={shape.axis_confidence:.2f} "
                    f"visual_conf={shape.visual_axis_confidence:.2f} "
                    f"extent=({shape.extent_major_m:.3f},{shape.extent_minor_m:.3f},{shape.extent_z_m:.3f}) "
                    f"z_bias={shape.z_bias_m:.3f}"
                )
            self.get_logger().info(
                f"{preview.detection.label} conf={preview.detection.confidence:.2f} "
                f"depth={preview.depth_m:.3f}m grasp_camera=({gx:.3f},{gy:.3f},{gz:.3f}) "
                f"base_offset=({self.grasp_base_offset[0]:.3f},{self.grasp_base_offset[1]:.3f},{self.grasp_base_offset[2]:.3f}) "
                f"roi={preview.roi_source} roi_points={len(preview.roi_points)} snapshot={preview.snapshot_reason} "
                f"{shape_msg} "
                f"planned_path_points={len(preview.base_path_xyz)} "
                f"stream_rate={max(float(self.args.playback_rate), 1.0):.1f}Hz "
                f"trajectory={preview.ik_message} "
                f"yolo_calls={self.inference_count} basket_safe={preview.basket_safe}"
            )

    def _playback_tick(self) -> None:
        if not self.inference_paused or self.last_preview is None:
            return
        if not self.last_preview.arm_trajectory_frames:
            return
        self._publish_preview_arm_state(self.last_preview)
        self.markers_pub.publish(MarkerArray(markers=self._basket_markers(self.last_preview)))

    def _publish_preview_arm_state(self, preview: GraspPreview) -> None:
        if not preview.arm_trajectory_frames:
            return
        frame = self._current_trajectory_frame(preview)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        gripper_names, gripper_positions = self._gripper_preview_positions(preview, frame)
        msg.name = list(ARM_JOINT_NAMES) + gripper_names
        msg.position = [float(value) for value in frame.positions] + gripper_positions
        msg.velocity = [float(value) for value in frame.velocities] + [0.0] * len(gripper_names)
        self.arm_preview_pub.publish(msg)

    def _gripper_preview_positions(
        self, preview: GraspPreview, frame: JointTrajectoryFrame
    ) -> tuple[list[str], list[float]]:
        profile = GRIPPER_PREVIEW_PROFILES.get(str(self.args.gripper_type))
        if not profile or not preview.arm_trajectory_frames:
            return [], []
        duration = max(float(preview.arm_trajectory_frames[-1].time_from_start), 1e-6)
        progress = min(max(float(frame.time_from_start) / duration, 0.0), 1.0)
        close_start, close_end, open_start, open_end, _close_label, _open_label = (
            self._gripper_event_progresses(preview)
        )

        if progress < close_start:
            amount = 0.0
        elif progress < open_start:
            amount = 1.0
        else:
            amount = 0.0

        open_positions = np.asarray(profile["open"], dtype=float)
        closed_positions = np.asarray(profile["closed"], dtype=float)
        positions = open_positions * (1.0 - amount) + closed_positions * amount
        return list(profile["names"]), [float(value) for value in positions]

    def _gripper_event_progresses(
        self, preview: GraspPreview
    ) -> tuple[float, float, float, float, str, str]:
        progress_by_name = {
            name: float(progress) for name, _point, progress in self._planning_target_samples(preview)
        }
        close_waypoint = str(self.args.gripper_close_waypoint).strip()
        open_waypoint = str(self.args.gripper_open_waypoint).strip()
        close_start = progress_by_name.get(
            close_waypoint,
            min(max(float(self.args.gripper_close_progress_start), 0.0), 1.0),
        )
        open_start = progress_by_name.get(
            open_waypoint,
            min(max(float(self.args.gripper_open_progress_start), 0.0), 1.0),
        )
        close_start = min(max(float(close_start), 0.0), 1.0)
        open_start = min(max(float(open_start), close_start), 1.0)
        transition = min(max(float(self.args.gripper_transition_progress), 1e-6), 0.25)
        close_end = min(max(close_start + transition, close_start + 1e-6), open_start)
        if open_start <= close_end:
            open_start = min(max(close_end, open_start), 1.0)
        open_end = min(max(open_start + transition, open_start + 1e-6), 1.0)
        close_label = f"{close_waypoint or 'grasp'}:close"
        open_label = f"{open_waypoint or 'basket_over'}:open"
        return close_start, close_end, open_start, open_end, close_label, open_label

    def _current_trajectory_frame(self, preview: GraspPreview) -> JointTrajectoryFrame:
        frames = preview.arm_trajectory_frames
        if not frames:
            q = tuple(float(value) for value in self.current_arm_joints)
            zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return JointTrajectoryFrame(0.0, q, zeros, zeros)  # type: ignore[arg-type]
        duration = max(float(frames[-1].time_from_start), 1e-6)
        period_override = float(self.args.playback_period)
        period = max(period_override, duration) if period_override > 0.0 else duration
        start_time = self.plan_lock_time or self.snapshot_time or time.monotonic()
        elapsed = max(time.monotonic() - start_time, 0.0)
        if bool(self.args.loop_playback):
            t = elapsed % max(period, 1e-6)
        else:
            t = min(elapsed, duration)
        frame = self._interpolated_trajectory_frame(frames, t)
        return frame

    def _interpolated_trajectory_frame(
        self, frames: list[JointTrajectoryFrame], t: float
    ) -> JointTrajectoryFrame:
        if len(frames) == 1 or t <= frames[0].time_from_start:
            return frames[0]
        for index, end in enumerate(frames[1:], start=1):
            start = frames[index - 1]
            if end.time_from_start >= t:
                span = max(end.time_from_start - start.time_from_start, 1e-9)
                local = (t - start.time_from_start) / span
                q0 = np.asarray(start.positions, dtype=float)
                q1 = np.asarray(end.positions, dtype=float)
                v0 = np.asarray(start.velocities, dtype=float)
                v1 = np.asarray(end.velocities, dtype=float)
                a0 = np.asarray(start.accelerations, dtype=float)
                a1 = np.asarray(end.accelerations, dtype=float)
                q = q0 * (1.0 - local) + q1 * local
                v = v0 * (1.0 - local) + v1 * local
                a = a0 * (1.0 - local) + a1 * local
                return JointTrajectoryFrame(
                    float(t),
                    tuple(float(value) for value in q),  # type: ignore[arg-type]
                    tuple(float(value) for value in v),  # type: ignore[arg-type]
                    tuple(float(value) for value in a),  # type: ignore[arg-type]
                )
        return frames[-1]

    def _arm_trajectory_grasp_path_base(
        self, frames: list[JointTrajectoryFrame]
    ) -> list[tuple[float, float, float]]:
        if not frames:
            return []
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return []
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._throttled_log(f"waiting for TF {self.base_frame} <- {self.aubo_base_frame}: {exc}")
            return []
        base_from_aubo = self._matrix_from_transform(transform)
        max_points = 500
        stride = max(int(math.ceil(len(frames) / max_points)), 1)
        selected = list(frames[::stride])
        if selected[-1] is not frames[-1]:
            selected.append(frames[-1])

        path: list[tuple[float, float, float]] = []
        for frame in selected:
            tool_in_aubo = self.kinematics.fk(np.asarray(frame.positions, dtype=float))
            grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
            path.append((float(grasp_in_base[0, 3]), float(grasp_in_base[1, 3]), float(grasp_in_base[2, 3])))
        return path

    def _current_grasp_position_base(self, preview: GraspPreview) -> tuple[float, float, float] | None:
        if not preview.arm_trajectory_frames:
            return None
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException:
            return None
        frame = self._current_trajectory_frame(preview)
        base_from_aubo = self._matrix_from_transform(transform)
        tool_in_aubo = self.kinematics.fk(np.asarray(frame.positions, dtype=float))
        grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
        return (float(grasp_in_base[0, 3]), float(grasp_in_base[1, 3]), float(grasp_in_base[2, 3]))

    def _current_grasp_origin_base(self) -> tuple[float, float, float] | None:
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        base_from_aubo = self._matrix_from_transform(transform)
        tool_in_aubo = self.kinematics.fk(np.asarray(self.current_arm_joints, dtype=float))
        grasp_in_base = base_from_aubo @ tool_in_aubo @ tool_to_grasp
        return (float(grasp_in_base[0, 3]), float(grasp_in_base[1, 3]), float(grasp_in_base[2, 3]))

    def _tool_to_grasp_matrix(self) -> tuple[np.ndarray | None, str]:
        if self.tool_to_grasp_matrix is not None:
            return self.tool_to_grasp_matrix, "cached"
        tcp_offset = float(self.args.grasp_tcp_offset_m)
        if tcp_offset > 0.0:
            self.tool_to_grasp_matrix = np.eye(4, dtype=np.float64)
            self.tool_to_grasp_matrix[2, 3] = tcp_offset
            self.tool_to_grasp_frames = ("tool0", f"virtual_grasp_tcp_z_{tcp_offset:.3f}m")
            return self.tool_to_grasp_matrix, self.tool_to_grasp_frames[1]
        last_error = ""
        for tool_frame in self._frame_candidates("tool0"):
            for grasp_frame in self._frame_candidates(END_EFFECTOR_FRAME):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        tool_frame,
                        grasp_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.02),
                    )
                except TransformException as exc:
                    last_error = str(exc)
                    continue
                self.tool_to_grasp_matrix = self._matrix_from_transform(transform)
                self.tool_to_grasp_frames = (tool_frame, grasp_frame)
                return self.tool_to_grasp_matrix, f"{tool_frame}<-{grasp_frame}"
        return None, f"waiting for TF tool0 <- {END_EFFECTOR_FRAME}: {last_error}"

    def _tool0_target_from_grasp_target(
        self,
        grasp_position_base: tuple[float, float, float],
        grasp_rotation_base: np.ndarray,
    ) -> tuple[tuple[float, float, float], np.ndarray] | None:
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is None:
            return None
        grasp_in_base = np.eye(4, dtype=np.float64)
        grasp_in_base[:3, :3] = np.asarray(grasp_rotation_base, dtype=np.float64)
        grasp_in_base[:3, 3] = np.asarray(grasp_position_base, dtype=np.float64)
        tool_in_base = grasp_in_base @ self._invert_rigid(tool_to_grasp)
        return (
            (float(tool_in_base[0, 3]), float(tool_in_base[1, 3]), float(tool_in_base[2, 3])),
            self._orthonormalize_rotation(tool_in_base[:3, :3]),
        )

    def _frame_candidates(self, frame_id: str) -> list[str]:
        prefix = self._display_frame_prefix()
        candidates = []
        if prefix and not frame_id.startswith(prefix):
            candidates.append(f"{prefix}{frame_id}")
        candidates.append(frame_id)
        return list(dict.fromkeys(candidates))

    def _display_frame_prefix(self) -> str:
        suffix = "aubo_base_link"
        if self.aubo_base_frame.endswith(suffix):
            return self.aubo_base_frame[: -len(suffix)]
        return ""

    def _joint_delta(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        raw = np.asarray(target, dtype=float) - np.asarray(current, dtype=float)
        return np.arctan2(np.sin(raw), np.cos(raw))

    def _wrap_joints(self, q: np.ndarray) -> np.ndarray:
        return np.arctan2(np.sin(q), np.cos(q))

    def _is_release_target(self, target_name: str) -> bool:
        return str(target_name).strip() in {"safe_mid", "lift", "basket_over", "drop"}

    def _joint_motion_cost(self, target_name: str, q_target: np.ndarray, q_current: np.ndarray) -> float:
        delta = np.abs(self._joint_delta(np.asarray(q_target, dtype=float), np.asarray(q_current, dtype=float)))
        if self._is_release_target(target_name):
            shoulder_elbow = max(float(self.args.release_shoulder_elbow_motion_weight), 0.0)
            wrist = max(float(self.args.release_wrist_motion_weight), 0.0)
            weights = np.asarray(
                [
                    shoulder_elbow,
                    shoulder_elbow,
                    max(shoulder_elbow * 0.8, wrist),
                    max(wrist, 0.01),
                    max(wrist, 0.01),
                    max(wrist, 0.01),
                ],
                dtype=float,
            )
            return float(np.linalg.norm(delta * weights))
        return float(np.linalg.norm(delta))

    def _release_configuration_penalty(
        self, target_name: str, q_target: np.ndarray, q_reference: np.ndarray
    ) -> float:
        if not self._is_release_target(target_name):
            return 0.0
        delta = np.abs(self._joint_delta(np.asarray(q_target, dtype=float), np.asarray(q_reference, dtype=float)))
        shoulder_elbow_motion = float(np.linalg.norm(delta[:3]))
        wrist_motion = float(np.linalg.norm(delta[3:]))
        return max(float(self.args.release_configuration_weight), 0.0) * shoulder_elbow_motion + 0.15 * wrist_motion

    def _make_constrained_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        if str(self.args.planner_backend) == "none":
            return [], "perception_only: planning skipped"
        if str(self.args.planner_backend) == "remote":
            return self._make_remote_arm_trajectory(preview)
        if str(self.args.planner_backend) == "moveit":
            return self._make_moveit_arm_trajectory(preview)
        return self._make_local_arm_trajectory(preview)

    def _make_remote_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        targets = self._planning_target_samples(preview)
        if not targets:
            return [], "remote: no planning key waypoints"
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return [], f"remote: waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"
        aubo_from_base = self._matrix_from_transform(transform)

        q_current = np.asarray(self.current_arm_joints, dtype=float)
        moveit_targets: list[dict[str, object]] = []
        reached_targets: list[str] = []
        notes: list[str] = []
        current_rotation_base = self._current_end_effector_rotation_base()

        for index, (target_name, target_base, progress) in enumerate(targets):
            position_tolerance, orientation_tolerance, use_orientation_constraint = (
                self._remote_moveit_goal_options(target_name)
            )
            target_rotations_base = self._target_orientation_candidates_base(
                target_base,
                progress,
                current_rotation_base,
                preview.pointcloud_shape,
                target_name,
            )
            alternatives: list[dict[str, object]] = []
            failure_messages: list[str] = []
            for orientation_index, target_rotation_base in enumerate(target_rotations_base):
                tool_target = self._tool0_target_from_grasp_target(
                    target_base, target_rotation_base
                )
                if tool_target is None:
                    _matrix, message = self._tool_to_grasp_matrix()
                    return [], f"remote: {message}"
                target_tool0_base, target_tool0_rotation_base = tool_target
                tool_ground_ok, tool_ground_clearance, tool_ground_hit = (
                    self._tool_target_ground_clearance_base(target_tool0_base, target_base)
                )
                if not tool_ground_ok:
                    failure_messages.append(
                        f"ori{orientation_index}: {tool_ground_hit} clearance={tool_ground_clearance:.3f}m"
                    )
                    continue
                unreachable_note = self._moveit_unreachable_note(target_tool0_base)
                if unreachable_note:
                    failure_messages.append(f"ori{orientation_index}: {unreachable_note}")
                    continue
                tool0_aubo = self._transform_matrix_point(
                    aubo_from_base, np.asarray(target_tool0_base, dtype=float)
                )
                tool0_rotation_aubo = np.asarray(aubo_from_base[:3, :3], dtype=float) @ np.asarray(
                    self._orthonormalize_rotation(target_tool0_rotation_base), dtype=float
                )
                tool0_quat_aubo = self._quat_msg_from_matrix(tool0_rotation_aubo)
                alternatives.append(
                    {
                        "name": f"{target_name}:ori{orientation_index}",
                        "orientation_index": int(orientation_index),
                        "tool0_xyz_aubo": [float(v) for v in tool0_aubo],
                        "tool0_quat_aubo": [
                            float(tool0_quat_aubo.x),
                            float(tool0_quat_aubo.y),
                            float(tool0_quat_aubo.z),
                            float(tool0_quat_aubo.w),
                        ],
                    }
                )
                if not use_orientation_constraint:
                    max_alternatives = 1
                else:
                    max_alternatives = 6 if target_name == "grasp" else 3
                if len(alternatives) >= max_alternatives:
                    break
            if not alternatives:
                x, y, z = target_base
                return (
                    [],
                    f"remote: no geometric pose candidate at {target_name} {index + 1}/{len(targets)} "
                    f"grasp_xyz=({x:.3f},{y:.3f},{z:.3f}); "
                    + " | ".join(failure_messages[-6:]),
                )
            reached_targets.append(target_name)
            orientation_note = "pose" if use_orientation_constraint else "position_only"
            notes.append(
                f"{target_name}:remote_{orientation_note}_alternatives={len(alternatives)}"
                f":pos_tol={position_tolerance:.3f}:ori_tol={orientation_tolerance:.3f}"
            )
            current_rotation_base = np.asarray(target_rotations_base[0], dtype=float)
            first = alternatives[0]
            moveit_targets.append(
                {
                    "name": target_name,
                    "tool0_xyz_aubo": first["tool0_xyz_aubo"],
                    "tool0_quat_aubo": first["tool0_quat_aubo"],
                    "position_tolerance": position_tolerance,
                    "orientation_tolerance": orientation_tolerance,
                    "use_orientation_constraint": use_orientation_constraint,
                    "alternatives": alternatives,
                }
            )

        payload = {
            "request_id": f"grasp_preview_{int(time.time() * 1000)}",
            "current_joints_rad": [float(v) for v in q_current],
            "targets": [
                {
                    "name": name,
                    "xyz_base": [float(v) for v in target_base],
                    "phase": name,
                }
                for name, target_base, _progress in targets
            ],
            "candidates": [
                {
                    "label": "remote_moveit_pose_goals",
                    "score": 0.0,
                    "waypoints_rad": [],
                    "events": [],
                }
            ],
            "moveit_targets": moveit_targets,
            "moveit": {
                "planner_id": str(self.args.moveit_planners).split(",")[0].strip()
                or "RRTConnectkConfigDefault",
                "planning_time": float(self.args.moveit_planning_time),
                "planning_attempts": int(self.args.moveit_planning_attempts),
                "velocity_scale": float(self.args.moveit_velocity_scale),
                "accel_scale": float(self.args.moveit_accel_scale),
                "service_timeout_padding": float(self.args.moveit_service_timeout_padding),
            },
            "constraints": {
                "ground_min_z_base": float(self.args.ground_min_z_base),
                "min_grasp_z_base": float(self.args.min_grasp_z_base),
                "tool_ground_clearance": float(self.args.tool_ground_clearance),
                "max_joint_speed_rad_s": float(self.args.preview_max_joint_speed),
                "max_joint_accel_rad_s2": float(self.args.preview_max_joint_accel),
                "max_segment_joint_delta_rad": float(self.args.real_sdk_max_segment_joint_delta),
                "min_segment_duration_sec": float(self.args.real_sdk_min_segment_duration),
            },
        }
        try:
            response = self._remote_planner_post(payload)
            frames = self._remote_trajectory_frames(response)
        except Exception as exc:
            return [], f"remote: planner request failed: {exc}"
        if not frames:
            return [], f"remote: empty trajectory response status={response.get('status')}"
        return (
            frames,
            "remote_joint_waypoint_plan "
            f"url={self._redacted_remote_url()} "
            f"targets={','.join(reached_targets)} "
            f"frames={len(frames)} duration={frames[-1].time_from_start:.2f}s "
            f"ik={';'.join(notes)} status={response.get('status')}",
        )

    def _remote_moveit_goal_options(self, target_name: str) -> tuple[float, float, bool]:
        name = str(target_name).strip()
        if name == "grasp":
            return (
                max(float(self.args.moveit_position_tolerance), 0.002),
                max(float(self.args.moveit_ik_orientation_tolerance), 0.01),
                True,
            )
        if name == "approach":
            return (
                max(float(self.args.moveit_soft_waypoint_position_tolerance), 0.0),
                max(float(self.args.moveit_soft_waypoint_orientation_tolerance), 0.01),
                True,
            )
        if name in {"safe_mid", "lift", "basket_over", "drop"}:
            return (
                max(float(self.args.moveit_soft_waypoint_position_tolerance), 0.0),
                max(float(self.args.moveit_release_orientation_tolerance), 0.01),
                False,
            )
        return (
            max(float(self.args.moveit_local_fallback_position_tolerance), 0.0),
            max(float(self.args.moveit_local_fallback_orientation_tolerance), 0.01),
            True,
        )

    def _remote_planner_post(self, payload: dict[str, object]) -> dict[str, object]:
        url = str(self.args.remote_planner_url).rstrip("/") + "/plan"
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        timeout = max(float(self.args.remote_planner_timeout), 0.1)
        target_count = len(payload.get("moveit_targets", [])) if isinstance(payload.get("moveit_targets"), list) else 0
        start = time.monotonic()
        self.get_logger().info(
            f"remote planner request: url={self._redacted_remote_url()} "
            f"moveit_targets={target_count} timeout={timeout:.1f}s"
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as handle:
                response = json.loads(handle.read().decode("utf-8"))
        except urllib.error.URLError as exc:
            raise RuntimeError(str(exc)) from exc
        elapsed = time.monotonic() - start
        self.get_logger().info(
            f"remote planner response: ok={bool(response.get('ok')) if isinstance(response, dict) else False} "
            f"wall={elapsed:.2f}s"
        )
        if not isinstance(response, dict):
            raise RuntimeError("non-object JSON response")
        if not bool(response.get("ok")):
            raise RuntimeError(
                f"{response.get('status')}: {response.get('constraint_issues') or response.get('message')}"
            )
        return response

    def _remote_trajectory_frames(self, response: dict[str, object]) -> list[JointTrajectoryFrame]:
        trajectory = response.get("trajectory")
        if not isinstance(trajectory, dict):
            return []
        frames_raw = trajectory.get("frames")
        if not isinstance(frames_raw, list):
            return []
        frames: list[JointTrajectoryFrame] = []
        for index, raw in enumerate(frames_raw):
            if not isinstance(raw, dict):
                continue
            positions = tuple(float(v) for v in raw.get("positions", []))
            if len(positions) != 6:
                continue
            velocities = tuple(float(v) for v in raw.get("velocities", [0.0] * 6))
            accelerations = tuple(float(v) for v in raw.get("accelerations", [0.0] * 6))
            if len(velocities) != 6:
                velocities = (0.0,) * 6
            if len(accelerations) != 6:
                accelerations = (0.0,) * 6
            frames.append(
                JointTrajectoryFrame(
                    float(raw.get("time_from_start", index * 0.2)),
                    positions,  # type: ignore[arg-type]
                    velocities,  # type: ignore[arg-type]
                    accelerations,  # type: ignore[arg-type]
                )
            )
        return frames

    def _redacted_remote_url(self) -> str:
        url = str(self.args.remote_planner_url)
        if "@" not in url:
            return url
        scheme, rest = url.split("://", 1) if "://" in url else ("", url)
        host = rest.split("@", 1)[1]
        return f"{scheme + '://' if scheme else ''}<redacted>@{host}"

    def _make_moveit_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        targets = self._planning_target_samples(preview)
        if not targets:
            return [], "moveit: no planning key waypoints"
        if self.moveit_plan_client is None:
            return [], "moveit: planner client not configured"
        if not self.moveit_plan_client.service_is_ready():
            self.moveit_plan_client.wait_for_service(timeout_sec=0.05)
        if not self.moveit_plan_client.service_is_ready():
            return [], f"moveit: waiting for {self.args.moveit_plan_service}"
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return [], f"moveit: waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"
        aubo_from_base = self._matrix_from_transform(transform)

        q_current = np.asarray(self.current_arm_joints, dtype=float)
        q_waypoints: list[np.ndarray] = [np.asarray(q_current, dtype=float)]
        planner_ids = [
            token.strip()
            for token in str(self.args.moveit_planners).split(",")
            if token.strip()
        ] or ["RRTConnectkConfigDefault"]
        planner_used: list[str] = []
        reached_targets: list[str] = []
        total_points = 0
        soft_waypoints = 0
        local_fallbacks = 0
        local_firsts = 0
        fallback_notes: list[str] = []
        current_rotation_base = self._current_end_effector_rotation_base()
        current_tool0_rotation_base = self._current_tool_rotation_base()
        base_from_aubo = self._invert_rigid(aubo_from_base)

        for index, (target_name, target_base, progress) in enumerate(targets):
            is_soft_waypoint = target_name != "grasp"
            target_rotations_base = self._target_orientation_candidates_base(
                target_base,
                progress,
                current_rotation_base,
                preview.pointcloud_shape,
                target_name,
            )
            response = None
            response_planner = ""
            failure_messages: list[str] = []
            fallback_candidate = None
            local_first_accepted = False
            for planner_id in planner_ids:
                for orientation_index, target_rotation_base in enumerate(target_rotations_base):
                    tool_target = self._tool0_target_from_grasp_target(
                        target_base, target_rotation_base
                    )
                    if tool_target is None:
                        _matrix, message = self._tool_to_grasp_matrix()
                        return [], f"moveit: {message}"
                    target_tool0_base, target_tool0_rotation_base = tool_target
                    tool_ground_ok, tool_ground_clearance, tool_ground_hit = (
                        self._tool_target_ground_clearance_base(target_tool0_base, target_base)
                    )
                    if not tool_ground_ok:
                        tx, ty, tz = target_tool0_base
                        failure_messages.append(
                            f"ori{orientation_index}/{target_name}: tool0=({tx:.3f},{ty:.3f},{tz:.3f}) "
                            f"{tool_ground_hit} clearance={tool_ground_clearance:.3f}m"
                        )
                        continue
                    unreachable_note = self._moveit_unreachable_note(target_tool0_base)
                    if unreachable_note:
                        x, y, z = target_base
                        tx, ty, tz = target_tool0_base
                        failure_messages.append(
                            f"ori{orientation_index}/{target_name}: grasp=({x:.3f},{y:.3f},{z:.3f}) "
                            f"tool0=({tx:.3f},{ty:.3f},{tz:.3f}) {unreachable_note}"
                        )
                        continue
                    ik_ok, q_goal, position_error, orientation_error = self._solve_tool0_goal_joints(
                        q_current,
                        aubo_from_base,
                        target_tool0_base,
                        target_tool0_rotation_base,
                    )
                    if not ik_ok and is_soft_waypoint:
                        ik_ok = (
                            position_error
                            <= max(float(self.args.moveit_soft_waypoint_position_tolerance), 0.0)
                            and orientation_error
                            <= max(float(self.args.moveit_soft_waypoint_orientation_tolerance), 0.01)
                        )
                        if ik_ok:
                            soft_waypoints += 1
                    q_unwrapped_goal = np.asarray(q_current, dtype=float) + self._joint_delta(
                        q_goal, q_current
                    )
                    endpoint_safe, endpoint_clearance, endpoint_hit_name = self._arm_collision_clearance_base(
                        q_unwrapped_goal, base_from_aubo
                    )
                    segment_safe, segment_clearance, segment_hit_name = (
                        self._joint_segment_collision_clearance_base(
                            q_current, q_unwrapped_goal, base_from_aubo
                        )
                    )
                    collision_free = bool(endpoint_safe and segment_safe)
                    clearance = min(float(endpoint_clearance), float(segment_clearance))
                    hit_name = endpoint_hit_name if not endpoint_safe else segment_hit_name
                    (
                        fallback_position_tolerance,
                        fallback_orientation_tolerance,
                    ) = self._moveit_fallback_tolerances(target_name, is_soft_waypoint)
                    fallback_ok = (
                        position_error <= fallback_position_tolerance
                        and orientation_error <= fallback_orientation_tolerance
                        and (collision_free or bool(self.args.allow_colliding_best_effort))
                    )
                    if fallback_ok:
                        joint_motion_cost = self._joint_motion_cost(
                            target_name, q_unwrapped_goal, q_current
                        )
                        release_config_penalty = self._release_configuration_penalty(
                            target_name, q_unwrapped_goal, q_current
                        )
                        fallback_score = (
                            100.0 * float(position_error)
                            + 5.0 * float(orientation_error)
                            + 0.25 * joint_motion_cost
                            + release_config_penalty
                            + (0.0 if collision_free else COLLISION_PENALTY)
                        )
                        candidate = (
                            fallback_score,
                            q_unwrapped_goal,
                            float(position_error),
                            float(orientation_error),
                            collision_free,
                            hit_name,
                            orientation_index,
                            target_rotation_base,
                            target_tool0_rotation_base,
                        )
                        if fallback_candidate is None or fallback_score < fallback_candidate[0]:
                            fallback_candidate = candidate
                    local_first_ok = (
                        bool(self.args.moveit_local_first)
                        and position_error
                        <= max(float(self.args.moveit_position_tolerance) * 2.0, 0.01)
                        and orientation_error
                        <= max(float(self.args.moveit_ik_orientation_tolerance), 0.01)
                        and collision_free
                        and (
                            not self._is_release_target(target_name)
                            or self._joint_motion_cost(target_name, q_unwrapped_goal, q_current)
                            <= max(float(self.args.real_sdk_max_segment_joint_delta), 0.05) * 4.0
                        )
                    )
                    if local_first_ok:
                        if np.max(np.abs(q_unwrapped_goal - q_waypoints[-1])) > 1e-5:
                            q_waypoints.append(np.asarray(q_unwrapped_goal, dtype=float))
                        q_current = np.asarray(q_waypoints[-1], dtype=float)
                        reached_targets.append(target_name)
                        planner_used.append(f"local_first_ori{orientation_index}")
                        local_firsts += 1
                        fallback_notes.append(
                            f"{target_name}/ori{orientation_index}:"
                            f"pos_err={position_error:.3f}m,"
                            f"ori_err={orientation_error:.3f}rad"
                        )
                        current_rotation_base = self._orthonormalize_rotation(
                            np.asarray(target_rotation_base, dtype=float)
                        )
                        current_tool0_rotation_base = self._orthonormalize_rotation(
                            np.asarray(target_tool0_rotation_base, dtype=float)
                        )
                        self._throttled_log(
                            f"MoveIt local-first accepted at {target_name}; "
                            f"pos_err={position_error:.3f}m ori_err={orientation_error:.3f}rad "
                            f"clearance={clearance:.3f}m"
                        )
                        local_first_accepted = True
                        break
                    if not ik_ok:
                        if not bool(self.args.moveit_pose_goal_on_ik_failure):
                            tx, ty, tz = target_tool0_base
                            failure_messages.append(
                                f"ik/ori{orientation_index} tool0=({tx:.3f},{ty:.3f},{tz:.3f}) "
                                f"pos_err={position_error:.3f}m ori_err={orientation_error:.3f}rad"
                            )
                            continue
                        attempt = self._call_moveit_pose_plan(
                            q_current,
                            target_tool0_base,
                            target_tool0_rotation_base,
                            planner_id,
                            max(float(self.args.moveit_ik_orientation_tolerance), 0.01),
                            current_tool0_rotation_base,
                        )
                    else:
                        attempt = self._call_moveit_plan(
                            q_current,
                            q_goal,
                            current_tool0_rotation_base,
                            planner_id,
                        )
                    if attempt.response is not None:
                        path_safe, path_message = self._moveit_response_ground_safe(
                            attempt.response, q_current, base_from_aubo
                        )
                        if not path_safe:
                            failure_messages.append(
                                f"{planner_id}/ori{orientation_index}: {path_message}"
                            )
                            continue
                        response = attempt.response
                        response_planner = planner_id
                        break
                    tx, ty, tz = target_tool0_base
                    failure_messages.append(
                        f"{planner_id}/ori{orientation_index} "
                        f"tool0=({tx:.3f},{ty:.3f},{tz:.3f}) "
                        f"q_goal_pos_err={position_error:.3f}m q_goal_ori_err={orientation_error:.3f}rad: "
                        f"{attempt.message}"
                    )
                if local_first_accepted:
                    break
                if response is not None:
                    break
            if local_first_accepted:
                continue
            if response is None:
                x, y, z = target_base
                failure_text = (
                    f"moveit: planning failed at {target_name} {index + 1}/{len(targets)} "
                    f"grasp_xyz=({x:.3f},{y:.3f},{z:.3f}); "
                    + " | ".join(failure_messages[-6:])
                )
                if fallback_candidate is not None:
                    (
                        _score,
                        q_fallback,
                        position_error,
                        orientation_error,
                        collision_free,
                        hit_name,
                        orientation_index,
                        fallback_rotation_base,
                        fallback_tool0_rotation_base,
                    ) = fallback_candidate
                    if not collision_free and not bool(self.args.allow_colliding_best_effort):
                        failure_text += f"; local fallback collision at {hit_name or 'vehicle'}"
                    else:
                        if np.max(np.abs(q_fallback - q_waypoints[-1])) > 1e-5:
                            q_waypoints.append(np.asarray(q_fallback, dtype=float))
                        q_current = np.asarray(q_waypoints[-1], dtype=float)
                        reached_targets.append(target_name)
                        planner_used.append(f"local_fallback_ori{orientation_index}")
                        local_fallbacks += 1
                        fallback_notes.append(
                            f"{target_name}/ori{orientation_index}:"
                            f"pos_err={position_error:.3f}m,"
                            f"ori_err={orientation_error:.3f}rad"
                        )
                        current_rotation_base = self._orthonormalize_rotation(
                            np.asarray(fallback_rotation_base, dtype=float)
                        )
                        current_tool0_rotation_base = self._orthonormalize_rotation(
                            np.asarray(fallback_tool0_rotation_base, dtype=float)
                        )
                        self._throttled_log(
                            f"MoveIt blocked at {target_name}; using local fallback "
                            f"pos_err={position_error:.3f}m ori_err={orientation_error:.3f}rad"
                        )
                        continue
                if len(q_waypoints) > 1:
                    frames, limit_message = self._generate_limited_joint_frames(q_waypoints)
                    return (
                        frames,
                        "moveit_ompl_joint_goal_partial "
                        f"partial_until={reached_targets[-1] if reached_targets else 'start'} "
                        f"blocked_at={target_name} "
                        f"planners={'+'.join(planner_used)} raw_points={total_points} "
                        f"targets={','.join(reached_targets)} soft_waypoints={soft_waypoints} "
                        f"local_firsts={local_firsts} local_fallbacks={local_fallbacks} "
                        f"fallbacks={';'.join(fallback_notes) if fallback_notes else 'none'} "
                        f"{limit_message}; {failure_text}",
                    )
                return (
                    [],
                    failure_text,
                )

            joint_trajectory = response.trajectory.joint_trajectory
            if not joint_trajectory.points:
                return [], f"moveit: empty trajectory from {response_planner}"
            name_to_index = {name: i for i, name in enumerate(joint_trajectory.joint_names)}
            for point in joint_trajectory.points[1:]:
                if not all(name in name_to_index for name in ARM_JOINT_NAMES):
                    return [], f"moveit: trajectory missing Aubo joints from {response_planner}"
                q_raw = np.asarray(
                    [point.positions[name_to_index[name]] for name in ARM_JOINT_NAMES],
                    dtype=float,
                )
                q_unwrapped = q_waypoints[-1] + self._joint_delta(q_raw, q_waypoints[-1])
                if np.max(np.abs(q_unwrapped - q_waypoints[-1])) > 1e-5:
                    q_waypoints.append(q_unwrapped)
            q_current = np.asarray(q_waypoints[-1], dtype=float)
            current_rotation_base = self._orthonormalize_rotation(
                np.asarray(target_rotation_base, dtype=float)
            )
            current_tool0_rotation_base = self._orthonormalize_rotation(
                np.asarray(target_tool0_rotation_base, dtype=float)
            )
            planner_used.append(response_planner)
            reached_targets.append(target_name)
            total_points += len(joint_trajectory.points)
            self._throttled_log(
                f"MoveIt segment ready: {target_name} "
                f"points={len(joint_trajectory.points)} planner={response_planner}"
            )

        frames, limit_message = self._generate_limited_joint_frames(q_waypoints)
        return (
            frames,
            "moveit_ompl_joint_goal "
            f"planners={'+'.join(planner_used)} raw_points={total_points} "
            f"targets={','.join(reached_targets)} "
            f"soft_waypoints={soft_waypoints} local_firsts={local_firsts} "
            f"local_fallbacks={local_fallbacks} "
            f"fallbacks={';'.join(fallback_notes) if fallback_notes else 'none'} "
            f"{limit_message}",
        )

    def _tool_target_ground_clearance_base(
        self,
        tool0_base: tuple[float, float, float],
        grasp_base: tuple[float, float, float],
    ) -> tuple[bool, float, str | None]:
        threshold = float(self.args.ground_min_z_base) + max(
            float(self.args.tool_ground_clearance), 0.0
        )
        min_clearance = float("inf")
        hit_name: str | None = None
        a = np.asarray(tool0_base, dtype=float)
        b = np.asarray(grasp_base, dtype=float)
        samples = max(int(self.args.arm_collision_samples_per_link), 2)
        for t in np.linspace(0.0, 1.0, samples):
            point = a * (1.0 - t) + b * t
            clearance = float(point[2] - threshold)
            if clearance < min_clearance:
                min_clearance = clearance
                hit_name = "ground/gripper_tcp"
            if clearance < 0.0:
                return False, clearance, hit_name
        return True, float(min_clearance if np.isfinite(min_clearance) else 0.0), hit_name

    def _moveit_response_ground_safe(
        self,
        response,
        q_start: np.ndarray,
        base_from_aubo: np.ndarray,
    ) -> tuple[bool, str]:
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            return False, "empty trajectory"
        name_to_index = {name: i for i, name in enumerate(trajectory.joint_names)}
        if not all(name in name_to_index for name in ARM_JOINT_NAMES):
            return False, "trajectory missing Aubo joints"
        q_previous = np.asarray(q_start, dtype=float)
        for point_index, point in enumerate(trajectory.points[1:], start=1):
            q_raw = np.asarray(
                [point.positions[name_to_index[name]] for name in ARM_JOINT_NAMES],
                dtype=float,
            )
            q_unwrapped = q_previous + self._joint_delta(q_raw, q_previous)
            safe, clearance, hit_name = self._arm_collision_clearance_base(
                q_unwrapped, base_from_aubo
            )
            if not safe:
                return (
                    False,
                    f"path collision at point {point_index}: {hit_name or 'vehicle'} "
                    f"clearance={clearance:.3f}m",
                )
            q_previous = q_unwrapped
        return True, "ok"

    def _joint_segment_collision_clearance_base(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        base_from_aubo: np.ndarray,
    ) -> tuple[bool, float, str | None]:
        delta = self._joint_delta(np.asarray(q_goal, dtype=float), np.asarray(q_start, dtype=float))
        samples = max(int(self.args.arm_collision_samples_per_link), 2)
        min_clearance = float("inf")
        min_hit_name: str | None = None
        for alpha in np.linspace(0.0, 1.0, samples):
            q_sample = np.asarray(q_start, dtype=float) + float(alpha) * delta
            safe, clearance, hit_name = self._arm_collision_clearance_base(
                q_sample, base_from_aubo
            )
            if clearance < min_clearance:
                min_clearance = float(clearance)
                min_hit_name = hit_name
            if not safe:
                return False, float(clearance), hit_name
        return (
            True,
            float(min_clearance if np.isfinite(min_clearance) else 0.0),
            min_hit_name,
        )

    def _moveit_fallback_tolerances(
        self, target_name: str, is_soft_waypoint: bool
    ) -> tuple[float, float]:
        if is_soft_waypoint:
            return (
                max(float(self.args.moveit_soft_waypoint_position_tolerance), 0.0),
                max(float(self.args.moveit_soft_waypoint_orientation_tolerance), 0.01),
            )
        if target_name == "grasp":
            return (
                max(float(self.args.moveit_grasp_fallback_position_tolerance), 0.0),
                max(float(self.args.moveit_grasp_fallback_orientation_tolerance), 0.01),
            )
        return (
            max(float(self.args.moveit_local_fallback_position_tolerance), 0.0),
            max(float(self.args.moveit_local_fallback_orientation_tolerance), 0.01),
        )

    def _planning_target_samples(
        self, preview: GraspPreview
    ) -> list[tuple[str, tuple[float, float, float], float]]:
        waypoint_map = {name: xyz for name, xyz in preview.base_waypoints}
        names = [
            token.strip()
            for token in str(self.args.planning_key_waypoints).split(",")
            if token.strip()
        ]
        selected: list[tuple[str, tuple[float, float, float]]] = []
        for name in names:
            if name in waypoint_map:
                selected.append((name, waypoint_map[name]))
        if not selected and preview.base_trajectory_segments:
            selected = [
                (f"segment_{index}", segment.end)
                for index, segment in enumerate(preview.base_trajectory_segments[:-1])
            ]
        if not selected:
            return []
        if len(selected) == 1:
            return [(selected[0][0], selected[0][1], 1.0)]
        return [
            (name, point, index / float(len(selected) - 1))
            for index, (name, point) in enumerate(selected)
        ]

    def _solve_tool0_goal_joints(
        self,
        q_start: np.ndarray,
        aubo_from_base: np.ndarray,
        target_tool0_base: tuple[float, float, float],
        target_tool0_rotation_base: np.ndarray,
    ) -> tuple[bool, np.ndarray, float, float]:
        target_transform = np.eye(4, dtype=np.float64)
        target_transform[:3, :3] = (
            np.asarray(aubo_from_base[:3, :3], dtype=np.float64)
            @ np.asarray(target_tool0_rotation_base, dtype=np.float64)
        )
        target_transform[:3, 3] = np.asarray(
            self._transform_matrix_point(aubo_from_base, np.asarray(target_tool0_base, dtype=float)),
            dtype=np.float64,
        )
        position_tolerance = max(float(self.args.moveit_position_tolerance), PREVIEW_IK_TOLERANCE_M)
        orientation_tolerance = max(float(self.args.moveit_ik_orientation_tolerance), 0.01)
        ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
            np.asarray(q_start, dtype=float),
            target_transform,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            damping=PREVIEW_IK_DAMPING,
            max_iterations=max(int(self.args.moveit_ik_max_iterations), 20),
            max_step=PREVIEW_IK_MAX_STEP,
            orientation_weight=0.25,
        )
        q_goal = np.asarray(q_start, dtype=float) + self._joint_delta(q_solution, q_start)
        return bool(ok), q_goal, float(position_error), float(orientation_error)

    def _call_moveit_pose_plan(
        self,
        q_start: np.ndarray,
        target_tool0_base: tuple[float, float, float],
        target_tool0_rotation_base: np.ndarray,
        planner_id: str,
        orientation_tolerance: float,
        reference_rotation_base: np.ndarray,
    ) -> MoveItPlanAttempt:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = "aubo_arm"
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = planner_id
        motion_request.num_planning_attempts = max(int(self.args.moveit_planning_attempts), 1)
        motion_request.allowed_planning_time = max(float(self.args.moveit_planning_time), 0.2)
        motion_request.max_velocity_scaling_factor = min(
            max(float(self.args.moveit_velocity_scale), 0.01), 1.0
        )
        motion_request.max_acceleration_scaling_factor = min(
            max(float(self.args.moveit_accel_scale), 0.01), 1.0
        )
        start_names, start_positions = self._moveit_start_state_joint_values(q_start)
        motion_request.start_state.joint_state.name = start_names
        motion_request.start_state.joint_state.position = start_positions
        motion_request.start_state.is_diff = False
        motion_request.workspace_parameters.header.frame_id = self.base_frame
        motion_request.workspace_parameters.min_corner.x = -1.0
        motion_request.workspace_parameters.min_corner.y = -1.0
        motion_request.workspace_parameters.min_corner.z = -0.3
        motion_request.workspace_parameters.max_corner.x = 1.6
        motion_request.workspace_parameters.max_corner.y = 1.0
        motion_request.workspace_parameters.max_corner.z = 1.2
        motion_request.goal_constraints = [
            self._moveit_pose_goal_constraint(
                target_tool0_base,
                target_tool0_rotation_base,
                orientation_tolerance,
            )
        ]
        if bool(self.args.moveit_use_orientation_path_constraint):
            motion_request.path_constraints = self._moveit_orientation_path_constraint(
                reference_rotation_base
            )

        future = self.moveit_plan_client.call_async(request)
        deadline = time.monotonic() + max(
            float(self.args.moveit_planning_time)
            + max(float(self.args.moveit_service_timeout_padding), 0.0),
            0.3,
        )
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return MoveItPlanAttempt(None, "pose goal service timeout")
        result = future.result()
        if result is None:
            return MoveItPlanAttempt(None, "pose goal empty service result")
        response = result.motion_plan_response
        if int(response.error_code.val) != 1:
            return MoveItPlanAttempt(
                None,
                f"pose goal {self._moveit_error_message(int(response.error_code.val))}",
            )
        return MoveItPlanAttempt(response, "pose goal success")

    def _moveit_joint_goal_constraint(self, q_goal: np.ndarray) -> Constraints:
        constraints = Constraints()
        constraints.name = "grasp_preview_joint_goal"
        tolerance = max(float(self.args.moveit_joint_goal_tolerance), 1e-4)
        for name, position in zip(ARM_JOINT_NAMES, np.asarray(q_goal, dtype=float)):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _call_moveit_plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        reference_rotation_base: np.ndarray,
        planner_id: str,
    ) -> MoveItPlanAttempt:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = "aubo_arm"
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = planner_id
        motion_request.num_planning_attempts = max(int(self.args.moveit_planning_attempts), 1)
        motion_request.allowed_planning_time = max(float(self.args.moveit_planning_time), 0.2)
        motion_request.max_velocity_scaling_factor = min(
            max(float(self.args.moveit_velocity_scale), 0.01), 1.0
        )
        motion_request.max_acceleration_scaling_factor = min(
            max(float(self.args.moveit_accel_scale), 0.01), 1.0
        )
        start_names, start_positions = self._moveit_start_state_joint_values(q_start)
        motion_request.start_state.joint_state.name = start_names
        motion_request.start_state.joint_state.position = start_positions
        motion_request.start_state.is_diff = False
        motion_request.workspace_parameters.header.frame_id = self.base_frame
        motion_request.workspace_parameters.min_corner.x = -1.0
        motion_request.workspace_parameters.min_corner.y = -1.0
        motion_request.workspace_parameters.min_corner.z = -0.3
        motion_request.workspace_parameters.max_corner.x = 1.6
        motion_request.workspace_parameters.max_corner.y = 1.0
        motion_request.workspace_parameters.max_corner.z = 1.2
        motion_request.goal_constraints = [self._moveit_joint_goal_constraint(q_goal)]
        if bool(self.args.moveit_use_orientation_path_constraint):
            motion_request.path_constraints = self._moveit_orientation_path_constraint(
                reference_rotation_base
            )

        future = self.moveit_plan_client.call_async(request)
        deadline = time.monotonic() + max(
            float(self.args.moveit_planning_time)
            + max(float(self.args.moveit_service_timeout_padding), 0.0),
            0.3,
        )
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        if not future.done():
            return MoveItPlanAttempt(None, "service timeout")
        result = future.result()
        if result is None:
            return MoveItPlanAttempt(None, "empty service result")
        response = result.motion_plan_response
        if int(response.error_code.val) != 1:
            return MoveItPlanAttempt(
                None,
                self._moveit_error_message(int(response.error_code.val)),
            )
        return MoveItPlanAttempt(response, "success")

    def _moveit_pose_goal_constraint(
        self,
        target_base: tuple[float, float, float],
        target_rotation_base: np.ndarray,
        orientation_tolerance: float,
    ) -> Constraints:
        constraints = Constraints()
        constraints.name = "grasp_preview_pose_goal"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [max(float(self.args.moveit_position_tolerance), 0.002)]
        pose = Pose()
        pose.position.x = float(target_base[0])
        pose.position.y = float(target_base[1])
        pose.position.z = float(target_base[2])
        pose.orientation.w = 1.0

        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = "tool0"
        position.constraint_region.primitives.append(primitive)
        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = "tool0"
        orientation.orientation = self._quat_msg_from_matrix(target_rotation_base)
        tolerance = min(max(float(orientation_tolerance), 0.01), self._tool_orientation_limit_rad())
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 0.8
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _moveit_orientation_path_constraint(self, reference_rotation_base: np.ndarray) -> Constraints:
        constraints = Constraints()
        constraints.name = "grasp_preview_tool_orientation_limit"
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = "tool0"
        orientation.orientation = self._quat_msg_from_matrix(reference_rotation_base)
        tolerance = self._tool_orientation_limit_rad()
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _moveit_error_message(self, code: int) -> str:
        names = {
            1: "SUCCESS",
            99999: "FAILURE",
            -1: "PLANNING_FAILED",
            -2: "INVALID_MOTION_PLAN",
            -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -4: "CONTROL_FAILED",
            -5: "UNABLE_TO_ACQUIRE_SENSOR_DATA",
            -6: "TIMED_OUT",
            -7: "PREEMPTED",
            -10: "START_STATE_IN_COLLISION",
            -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -26: "START_STATE_INVALID",
            -12: "GOAL_IN_COLLISION",
            -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED",
            -15: "INVALID_GROUP_NAME",
            -16: "INVALID_GOAL_CONSTRAINTS",
            -17: "INVALID_ROBOT_STATE",
            -18: "INVALID_LINK_NAME",
            -21: "FRAME_TRANSFORM_FAILURE",
            -22: "COLLISION_CHECKING_UNAVAILABLE",
            -23: "ROBOT_STATE_STALE",
            -27: "GOAL_STATE_INVALID",
            -28: "UNRECOGNIZED_GOAL_TYPE",
            -31: "NO_IK_SOLUTION",
        }
        return f"{names.get(code, 'UNKNOWN')}({code})"

    def _current_tool_rotation_base(self) -> np.ndarray:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.aubo_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
            base_from_aubo = self._matrix_from_transform(transform)
            return base_from_aubo[:3, :3] @ self.kinematics.fk(
                np.asarray(self.current_arm_joints, dtype=float)
            )[:3, :3]
        except TransformException:
            return self.kinematics.fk(np.asarray(self.current_arm_joints, dtype=float))[:3, :3]

    def _current_end_effector_rotation_base(self) -> np.ndarray:
        if float(self.args.grasp_tcp_offset_m) > 0.0:
            return self._current_tool_rotation_base()
        for frame_id in self._frame_candidates(END_EFFECTOR_FRAME):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.01),
                )
            except TransformException:
                continue
            return self._matrix_from_transform(transform)[:3, :3]
        return self._current_tool_rotation_base()

    def _make_local_arm_trajectory(
        self, preview: GraspPreview
    ) -> tuple[list[JointTrajectoryFrame], str]:
        targets = self._planning_target_samples(preview)
        if not targets:
            return [], "no planning key waypoints"
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return [], f"waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"

        aubo_from_base = self._matrix_from_transform(transform)
        base_from_aubo = self._invert_rigid(aubo_from_base)
        current_rotation_base = self._current_end_effector_rotation_base()
        q_seed = np.asarray(self.current_arm_joints, dtype=float)
        q_waypoints: list[np.ndarray] = [np.asarray(q_seed, dtype=float)]
        failures = 0
        collision_failures = 0
        max_position_error = 0.0
        max_orientation_error = 0.0
        orientation_candidates = 0
        local_deadline = 0.0
        local_timeout = max(float(self.args.local_planning_timeout_sec), 0.0)
        if local_timeout > 0.0:
            local_deadline = time.monotonic() + local_timeout
        for target_name, point, progress in targets:
            best_candidate = None
            for candidate_index, target_rotation_base in enumerate(
                self._target_orientation_candidates_base(
                    point,
                    progress,
                    current_rotation_base,
                    preview.pointcloud_shape,
                    target_name,
                )
            ):
                orientation_candidates += 1
                tool_target = self._tool0_target_from_grasp_target(point, target_rotation_base)
                if tool_target is None:
                    _matrix, message = self._tool_to_grasp_matrix()
                    return [], f"local ik: {message}"
                target_tool0_base, target_tool0_rotation_base = tool_target
                unreachable_note = self._moveit_unreachable_note(target_tool0_base)
                if unreachable_note:
                    x, y, z = point
                    return (
                        [],
                        f"target_out_of_reach at {target_name} xyz=({x:.3f},{y:.3f},{z:.3f}); "
                        f"{unreachable_note}",
                    )
                target_position = np.asarray(
                    self._transform_point(transform, target_tool0_base), dtype=float
                )
                target_transform = np.eye(4)
                target_transform[:3, :3] = aubo_from_base[:3, :3] @ target_tool0_rotation_base
                target_transform[:3, 3] = target_position
                ok, q_solution, position_error, orientation_error, _iterations = self.kinematics.solve_pose(
                    q_seed,
                    target_transform,
                    position_tolerance=max(float(self.args.local_position_tolerance), PREVIEW_IK_TOLERANCE_M),
                    orientation_tolerance=max(float(self.args.local_orientation_tolerance), 0.01),
                    damping=PREVIEW_IK_DAMPING,
                    max_iterations=max(int(self.args.local_ik_max_iterations), 10),
                    max_step=PREVIEW_IK_MAX_STEP,
                    orientation_weight=0.25,
                )
                q_unwrapped = q_waypoints[-1] + self._joint_delta(q_solution, q_waypoints[-1])
                endpoint_safe, endpoint_clearance, endpoint_hit_name = self._arm_collision_clearance_base(
                    q_unwrapped, base_from_aubo
                )
                segment_safe, segment_clearance, segment_hit_name = (
                    self._joint_segment_collision_clearance_base(
                        q_waypoints[-1], q_unwrapped, base_from_aubo
                    )
                )
                collision_free = bool(endpoint_safe and segment_safe)
                clearance = min(float(endpoint_clearance), float(segment_clearance))
                hit_name = endpoint_hit_name if not endpoint_safe else segment_hit_name
                joint_step = self._joint_motion_cost(target_name, q_unwrapped, q_waypoints[-1])
                release_config_penalty = self._release_configuration_penalty(
                    target_name, q_unwrapped, q_waypoints[-1]
                )
                collision_score = 0.0 if collision_free else COLLISION_PENALTY
                score = (
                    collision_score
                    + 150.0 * float(position_error)
                    + 8.0 * float(orientation_error)
                    + 1.2 * joint_step
                    + release_config_penalty
                    + 0.05 * float(candidate_index)
                    - 0.4 * float(clearance)
                )
                candidate = (
                    score,
                    collision_free,
                    ok,
                    q_solution,
                    q_unwrapped,
                    float(position_error),
                    float(orientation_error),
                    hit_name,
                    target_rotation_base,
                )
                if best_candidate is None or score < best_candidate[0]:
                    best_candidate = candidate
                if local_deadline > 0.0 and time.monotonic() >= local_deadline:
                    return [], f"local planning timeout after {local_timeout:.1f}s"

            if best_candidate is None:
                failures += 1
                continue

            (
                _score,
                collision_free,
                ok,
                q_solution,
                q_unwrapped,
                position_error,
                orientation_error,
                hit_name,
                target_rotation_base,
            ) = best_candidate
            if not collision_free:
                collision_failures += 1
                if not bool(self.args.allow_colliding_best_effort):
                    return [], f"collision blocked at {hit_name or 'vehicle'} progress={progress:.2f}"
            if not ok:
                failures += 1
            if bool(self.args.local_strict_ik):
                position_limit = max(float(self.args.local_position_tolerance), 0.001)
                orientation_limit = max(float(self.args.local_orientation_tolerance), 0.01)
                if (not ok) or position_error > position_limit or orientation_error > orientation_limit:
                    x, y, z = point
                    return (
                        [],
                        f"local ik rejected at {target_name} xyz=({x:.3f},{y:.3f},{z:.3f}) "
                        f"pos_err={position_error:.3f}m/{position_limit:.3f}m "
                        f"ori_err={orientation_error:.3f}rad/{orientation_limit:.3f}rad",
                    )
            max_position_error = max(max_position_error, float(position_error))
            max_orientation_error = max(max_orientation_error, float(orientation_error))
            if np.max(np.abs(q_unwrapped - q_waypoints[-1])) > 1e-5:
                q_waypoints.append(q_unwrapped)
            q_seed = np.asarray(q_solution, dtype=float)
            current_rotation_base = self._orthonormalize_rotation(
                np.asarray(target_rotation_base, dtype=float)
            )

        frames, limit_message = self._generate_limited_joint_frames(q_waypoints)
        status = "ok" if failures == 0 else f"best-effort ik_failures={failures}"
        if collision_failures:
            status += f" collision_failures={collision_failures}"
        return (
            frames,
            f"{status}, {limit_message}, "
            f"targets={','.join(name for name, _point, _progress in targets)} "
            f"orientation_candidates={orientation_candidates} "
            f"max_pos_error={max_position_error:.3f}m "
            f"max_ori_error={max_orientation_error:.3f}rad",
        )

    def _trajectory_reference_samples(
        self, segments: list[CartesianSegment]
    ) -> list[tuple[tuple[float, float, float], float]]:
        points = self._trajectory_reference_points(segments)
        if not points:
            return []
        if len(points) == 1:
            return [(points[0], 0.0)]
        return [(point, index / float(len(points) - 1)) for index, point in enumerate(points)]

    def _target_orientation_candidates_base(
        self,
        point_base: tuple[float, float, float],
        progress: float,
        current_rotation_base: np.ndarray,
        pointcloud_shape: PointCloudGraspShape | None = None,
        target_name: str = "",
    ) -> list[np.ndarray]:
        release_tilt = math.radians(float(self.args.release_tool_tilt_deg))
        tilted_down = np.asarray(
            [-math.sin(release_tilt), 0.0, -math.cos(release_tilt)], dtype=float
        )
        downward = self._rotation_from_tool_z_base(tilted_down, np.asarray([1.0, 0.0, 0.0]))

        grasp_hint = np.asarray(point_base, dtype=float) - np.asarray(
            [self.basket_release_base[0], self.basket_release_base[1], point_base[2]],
            dtype=float,
        )
        if np.linalg.norm(grasp_hint) < 1e-6:
            grasp_hint = np.asarray([1.0, 0.0, 0.0], dtype=float)
        grasp_down = self._rotation_from_tool_z_base(
            np.asarray([0.0, 0.0, -1.0], dtype=float), grasp_hint
        )
        shape_hints = self._pointcloud_shape_orientation_hints(pointcloud_shape)

        phase = self._target_orientation_phase(target_name, progress)
        if bool(self.args.lock_grasp_orientation) and phase == "grasp":
            roll, pitch, yaw = _float_values(
                str(self.args.topdown_flange_rpy_rad),
                3,
                "--topdown-flange-rpy-rad",
            )
            axis_yaw = self._pointcloud_major_axis_yaw(pointcloud_shape)
            candidates: list[np.ndarray] = []
            for yaw_offset in self._angle_offsets_rad(
                str(self.args.grasp_orientation_yaw_offsets_deg),
                "--grasp-orientation-yaw-offsets-deg",
            ):
                candidates.append(
                    self._orthonormalize_rotation(
                        self._rpy_matrix(
                            float(roll),
                            float(pitch),
                            float(yaw) + axis_yaw + float(yaw_offset),
                        )
                    )
                )
                if len(candidates) >= self._max_grasp_orientation_candidates():
                    break
            return candidates
        if phase == "current":
            nominal = current_rotation_base
        elif phase == "grasp":
            nominal = grasp_down
        elif phase in {"carry", "release"}:
            nominal = current_rotation_base
        else:
            nominal = downward

        limit_rad = self._tool_orientation_limit_rad()
        if phase in {"current", "carry", "release"}:
            yaw_offsets = (0.0,)
            tilt_offsets = ((0.0, 0.0),)
        elif phase == "grasp":
            yaw_offsets = self._angle_offsets_rad(
                str(self.args.grasp_orientation_yaw_offsets_deg),
                "--grasp-orientation-yaw-offsets-deg",
            )
            tilt_offsets = self._grasp_tilt_offsets_rad()
        else:
            yaw_offsets = self._angle_offsets_rad(
                str(self.args.transit_orientation_yaw_offsets_deg),
                "--transit-orientation-yaw-offsets-deg",
            )
            tilt_offsets = ((0.0, 0.0),)
        candidates: list[np.ndarray] = []
        if phase == "grasp":
            for hint in shape_hints:
                shape_nominal = self._rotation_from_tool_z_base(
                    np.asarray([0.0, 0.0, -1.0], dtype=float), hint
                )
                for yaw in (0.0,):
                    for roll, pitch in tilt_offsets[:3]:
                        candidate = self._orthonormalize_rotation(
                            shape_nominal @ self._rpy_matrix(roll, pitch, yaw)
                        )
                        candidate = self._limit_rotation_from_reference(
                            current_rotation_base, candidate, limit_rad
                        )
                        if not self._is_topdown_grasp_orientation(candidate):
                            continue
                        if not any(
                            self._rotation_distance(candidate, existing) < 1e-4
                            for existing in candidates
                        ):
                            candidates.append(candidate)
                            if len(candidates) >= self._max_grasp_orientation_candidates():
                                return candidates
        for yaw in yaw_offsets:
            for roll, pitch in tilt_offsets:
                candidate = nominal @ self._rpy_matrix(roll, pitch, yaw)
                if phase in {"current", "grasp", "carry", "release"}:
                    candidate = self._limit_rotation_from_reference(
                        current_rotation_base, candidate, limit_rad
                    )
                else:
                    candidate = self._orthonormalize_rotation(candidate)
                if phase in {"grasp", "carry", "release"} and not self._is_topdown_grasp_orientation(
                    candidate
                ):
                    continue
                if not any(
                    self._rotation_distance(candidate, existing) < 1e-4
                    for existing in candidates
                ):
                    candidates.append(candidate)
                    if (
                        phase == "grasp"
                        and len(candidates) >= self._max_grasp_orientation_candidates()
                    ):
                        return candidates
        if not candidates and phase in {"grasp", "carry", "release"}:
            candidates.append(self._orthonormalize_rotation(np.asarray(current_rotation_base, dtype=float)))
        return candidates

    def _target_orientation_phase(self, target_name: str, progress: float) -> str:
        name = str(target_name).strip()
        if name in {"start_ee", "observe_start"}:
            return "current"
        if name in {"approach", "grasp"}:
            return "grasp"
        if name in {"safe_mid", "lift"}:
            return "carry"
        if name in {"basket_over", "drop"}:
            return "release"
        if progress < 0.18:
            return "current"
        if progress < 0.60:
            return "grasp"
        return "release"

    def _max_grasp_orientation_candidates(self) -> int:
        return max(int(self.args.max_grasp_orientation_candidates), 1)

    def _pointcloud_shape_orientation_hints(
        self, pointcloud_shape: PointCloudGraspShape | None
    ) -> list[np.ndarray]:
        if pointcloud_shape is None:
            return []
        axes: list[np.ndarray] = []
        if (
            pointcloud_shape.visual_axis_base is not None
            and pointcloud_shape.visual_axis_confidence
            >= max(float(self.args.visual_axis_confidence_threshold), 0.0)
        ):
            visual_major = np.asarray(pointcloud_shape.visual_axis_base, dtype=float)
            visual_minor = np.asarray([-visual_major[1], visual_major[0], 0.0], dtype=float)
            axes.extend([visual_minor, -visual_minor, visual_major, -visual_major])
        if pointcloud_shape.axis_confidence >= max(
            float(self.args.pointcloud_axis_confidence_threshold), 0.0
        ):
            axes.extend(
                [
                    np.asarray(pointcloud_shape.minor_axis_base, dtype=float),
                    -np.asarray(pointcloud_shape.minor_axis_base, dtype=float),
                    np.asarray(pointcloud_shape.major_axis_base, dtype=float),
                    -np.asarray(pointcloud_shape.major_axis_base, dtype=float),
                ]
            )
        hints: list[np.ndarray] = []
        for axis in axes:
            axis[2] = 0.0
            norm = float(np.linalg.norm(axis))
            if norm < 1e-6:
                continue
            axis = axis / norm
            if not any(float(np.linalg.norm(axis - existing)) < 1e-3 for existing in hints):
                hints.append(axis)
        return hints

    def _pointcloud_major_axis_yaw(self, pointcloud_shape: PointCloudGraspShape | None) -> float:
        if pointcloud_shape is None:
            return 0.0
        axis = None
        if (
            pointcloud_shape.visual_axis_base is not None
            and pointcloud_shape.visual_axis_confidence
            >= max(float(self.args.visual_axis_confidence_threshold), 0.0)
        ):
            axis = np.asarray(pointcloud_shape.visual_axis_base, dtype=float)
        elif pointcloud_shape.axis_confidence >= max(
            float(self.args.pointcloud_axis_confidence_threshold), 0.0
        ):
            axis = np.asarray(pointcloud_shape.major_axis_base, dtype=float)
        if axis is None:
            return 0.0
        axis[2] = 0.0
        if float(np.linalg.norm(axis)) < 1e-6:
            return 0.0
        return float(math.atan2(float(axis[1]), float(axis[0])))

    def _is_topdown_grasp_orientation(self, rotation_base: np.ndarray) -> bool:
        tool_z_base = np.asarray(rotation_base, dtype=float)[:3, 2]
        if float(np.linalg.norm(tool_z_base)) < 1e-9:
            return False
        tool_z_base = tool_z_base / float(np.linalg.norm(tool_z_base))
        max_tilt = min(max(float(self.args.grasp_topdown_max_tilt_deg), 1.0), 89.0)
        return float(np.dot(tool_z_base, np.asarray([0.0, 0.0, -1.0]))) >= math.cos(
            math.radians(max_tilt)
        )

    def _angle_offsets_rad(self, value: str, label: str) -> tuple[float, ...]:
        offsets = [math.radians(item) for item in _float_list(value, label)]
        if not any(abs(item) < 1e-9 for item in offsets):
            offsets.insert(0, 0.0)
        return tuple(offsets)

    def _grasp_tilt_offsets_rad(self) -> tuple[tuple[float, float], ...]:
        values = self._angle_offsets_rad(
            str(self.args.grasp_orientation_tilt_offsets_deg),
            "--grasp-orientation-tilt-offsets-deg",
        )
        offsets: list[tuple[float, float]] = [(0.0, 0.0)]
        for value in values:
            if abs(value) < 1e-9:
                continue
            offsets.append((value, 0.0))
            offsets.append((0.0, value))
        return tuple(offsets)

    def _rotation_from_tool_z_base(self, tool_z: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
        z_axis = np.asarray(tool_z, dtype=float)
        z_axis /= max(float(np.linalg.norm(z_axis)), 1e-9)
        x_axis = np.asarray(x_hint, dtype=float)
        x_axis = x_axis - z_axis * float(np.dot(x_axis, z_axis))
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = np.asarray([0.0, 1.0, 0.0], dtype=float)
            x_axis = x_axis - z_axis * float(np.dot(x_axis, z_axis))
        x_axis /= max(float(np.linalg.norm(x_axis)), 1e-9)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= max(float(np.linalg.norm(y_axis)), 1e-9)
        x_axis = np.cross(y_axis, z_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def _blend_rotation(self, start: np.ndarray, end: np.ndarray, amount: float) -> np.ndarray:
        raw = (1.0 - amount) * np.asarray(start, dtype=float) + amount * np.asarray(end, dtype=float)
        u, _s, vh = np.linalg.svd(raw)
        rotation = u @ vh
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vh
        return rotation

    def _tool_orientation_limit_rad(self) -> float:
        return max(math.radians(float(self.args.tool_orientation_limit_deg)), math.radians(1.0))

    def _limit_rotation_from_reference(
        self, reference: np.ndarray, candidate: np.ndarray, limit_rad: float
    ) -> np.ndarray:
        delta = np.asarray(reference, dtype=float).T @ np.asarray(candidate, dtype=float)
        roll, pitch, yaw = self._matrix_to_rpy(delta)
        clipped_delta = self._rpy_matrix(
            float(np.clip(roll, -limit_rad, limit_rad)),
            float(np.clip(pitch, -limit_rad, limit_rad)),
            float(np.clip(yaw, -limit_rad, limit_rad)),
        )
        return self._orthonormalize_rotation(np.asarray(reference, dtype=float) @ clipped_delta)

    def _matrix_to_rpy(self, matrix: np.ndarray) -> tuple[float, float, float]:
        rotation = np.asarray(matrix, dtype=np.float64)
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
        cp = math.cos(pitch)
        if abs(cp) > 1e-8:
            roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
            yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        else:
            roll = 0.0
            yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
        return (self._wrap_angle(roll), self._wrap_angle(pitch), self._wrap_angle(yaw))

    def _wrap_angle(self, angle: float) -> float:
        return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi

    def _smoothstep(self, value: float) -> float:
        x = min(max(float(value), 0.0), 1.0)
        return x * x * (3.0 - 2.0 * x)

    def _rotation_distance(self, left: np.ndarray, right: np.ndarray) -> float:
        delta = np.asarray(left, dtype=float).T @ np.asarray(right, dtype=float)
        trace = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
        return math.acos(trace)

    def _orthonormalize_rotation(self, matrix: np.ndarray) -> np.ndarray:
        u, _s, vh = np.linalg.svd(np.asarray(matrix, dtype=float))
        rotation = u @ vh
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vh
        return rotation

    def _arm_collision_clearance_base(
        self, q: np.ndarray, base_from_aubo: np.ndarray
    ) -> tuple[bool, float, str | None]:
        link_points = [
            self._transform_matrix_point(base_from_aubo, transform[:3, 3])
            for _name, transform in self.kinematics.link_transforms(np.asarray(q, dtype=float))
        ]
        tool_to_grasp, _message = self._tool_to_grasp_matrix()
        if tool_to_grasp is not None:
            tool_in_aubo = self.kinematics.fk(np.asarray(q, dtype=float))
            grasp_in_base = np.asarray(base_from_aubo, dtype=float) @ tool_in_aubo @ tool_to_grasp
            link_points.append(tuple(float(value) for value in grasp_in_base[:3, 3]))
        samples_per_link = max(int(self.args.arm_collision_samples_per_link), 2)
        radius = max(float(self.args.arm_collision_radius), 0.0)
        ground_threshold = (
            float(self.args.ground_min_z_base)
            + max(float(self.args.ground_clearance), 0.0)
            + radius
        )
        min_clearance = float("inf")
        hit_name: str | None = None
        for start, end in zip(link_points[1:], link_points[2:]):
            a = np.asarray(start, dtype=float)
            b = np.asarray(end, dtype=float)
            for t in np.linspace(0.0, 1.0, samples_per_link):
                point = a * (1.0 - t) + b * t
                ground_clearance = float(point[2] - ground_threshold)
                if ground_clearance < min_clearance:
                    min_clearance = ground_clearance
                    hit_name = "ground/arm"
                if ground_clearance < 0.0:
                    return False, ground_clearance, "ground/arm"
                for box in self.collision_boxes:
                    clearance = self._point_box_clearance(point, box) - radius
                    if clearance < min_clearance:
                        min_clearance = float(clearance)
                        hit_name = box.name
                    if clearance < 0.0:
                        return False, float(clearance), box.name
        return True, float(min_clearance if np.isfinite(min_clearance) else 0.0), hit_name

    def _point_box_clearance(self, point: np.ndarray, box: CollisionBox) -> float:
        center = np.asarray(box.center, dtype=float)
        half = 0.5 * np.asarray(box.size, dtype=float)
        delta = np.abs(np.asarray(point, dtype=float) - center) - half
        outside = np.maximum(delta, 0.0)
        outside_distance = float(np.linalg.norm(outside))
        if outside_distance > 0.0:
            return outside_distance
        return float(np.max(delta))

    def _trajectory_reference_points(
        self, segments: list[CartesianSegment]
    ) -> list[tuple[float, float, float]]:
        step_m = max(float(self.args.trajectory_cartesian_step), 0.005)
        points: list[tuple[float, float, float]] = []
        for segment in segments:
            count = max(int(np.ceil(self._segment_length(segment) / step_m)) + 1, 2)
            for index, t in enumerate(np.linspace(0.0, 1.0, count)):
                if points and index == 0:
                    continue
                points.append(self._sample_segment(segment, float(t)))
        return points

    def _generate_limited_joint_frames(
        self, waypoints: list[np.ndarray]
    ) -> tuple[list[JointTrajectoryFrame], str]:
        if not waypoints:
            return [], "no joint waypoints"
        rate = max(float(self.args.playback_rate), 1.0)
        dt = 1.0 / rate
        max_speed = max(float(self.args.preview_max_joint_speed), 0.01)
        max_accel = max(float(self.args.preview_max_joint_accel), 0.05)
        max_jerk = max(float(self.args.preview_max_joint_jerk), 0.1)
        tau = max(float(self.args.preview_smoothing_tau), 0.02)
        tolerance = max(float(self.args.trajectory_joint_tolerance), 1e-4)
        max_frames = max(int(max(float(self.args.trajectory_max_duration), dt) * rate), 1)

        q = np.asarray(waypoints[0], dtype=float)
        velocity = np.zeros(6, dtype=float)
        accel = np.zeros(6, dtype=float)
        frames = [self._joint_frame(0.0, q, velocity, accel)]
        target_index = 1
        time_from_start = 0.0
        max_observed_speed = 0.0
        max_observed_accel = 0.0

        while target_index < len(waypoints) and len(frames) < max_frames:
            target = np.asarray(waypoints[target_index], dtype=float)
            delta = target - q
            if float(np.max(np.abs(delta))) <= tolerance:
                target_index += 1
                continue

            target_velocity = np.clip(delta / max(dt, 1e-9), -max_speed, max_speed)
            alpha = min(max(dt / tau, 0.0), 1.0)
            smoothed_target = velocity + alpha * (target_velocity - velocity)
            desired_accel = np.clip((smoothed_target - velocity) / dt, -max_accel, max_accel)
            accel_delta = np.clip(desired_accel - accel, -max_jerk * dt, max_jerk * dt)
            accel = accel + accel_delta
            next_velocity = np.clip(velocity + accel * dt, -max_speed, max_speed)
            step = next_velocity * dt

            for joint_index, (candidate_step, remaining) in enumerate(zip(step, delta)):
                if abs(candidate_step) >= abs(remaining) or candidate_step * remaining < 0.0:
                    step[joint_index] = remaining
                    next_velocity[joint_index] = 0.0
                    accel[joint_index] = 0.0

            q = q + step
            velocity = next_velocity
            time_from_start += dt
            max_observed_speed = max(max_observed_speed, float(np.max(np.abs(velocity))))
            max_observed_accel = max(max_observed_accel, float(np.max(np.abs(accel))))
            frames.append(self._joint_frame(time_from_start, q, velocity, accel))

        if frames:
            final = np.asarray(frames[-1].positions, dtype=float)
            frames.append(self._joint_frame(frames[-1].time_from_start + dt, final, np.zeros(6), np.zeros(6)))
        reached = target_index >= len(waypoints)
        status = "reached" if reached else "truncated"
        return (
            frames,
            f"frames={len(frames)} duration={frames[-1].time_from_start:.2f}s "
            f"{status} max_speed={max_observed_speed:.3f}rad/s "
            f"max_accel={max_observed_accel:.3f}rad/s2",
        )

    def _joint_frame(
        self, time_from_start: float, q: np.ndarray, velocity: np.ndarray, accel: np.ndarray
    ) -> JointTrajectoryFrame:
        return JointTrajectoryFrame(
            float(time_from_start),
            tuple(float(value) for value in q),  # type: ignore[arg-type]
            tuple(float(value) for value in velocity),  # type: ignore[arg-type]
            tuple(float(value) for value in accel),  # type: ignore[arg-type]
        )

    def _make_base_path(
        self,
        camera_path: list[tuple[float, float, float]],
        roi_points: list[tuple[float, float, float]] | None = None,
        visual_axis_camera: np.ndarray | None = None,
        visual_axis_confidence: float = 0.0,
    ) -> tuple[
        list[tuple[float, float, float]],
        list[CartesianSegment],
        list[tuple[str, tuple[float, float, float]]],
        tuple[float, float, float] | None,
        PointCloudGraspShape | None,
    ]:
        start = time.monotonic()
        if self.latest_depth is None:
            return [], [], [], None, None
        depth_header = self.latest_depth.header
        source_frame = depth_header.frame_id
        try:
            transform = self._base_from_depth_transform(depth_header, 0.05)
        except TransformException as exc:
            self._throttled_log(f"waiting for TF {self.base_frame} <- {source_frame}: {exc}")
            return [], [], [], None, None
        if transform is None:
            return [], [], [], None, None
        tf_done = time.monotonic()

        base_grasp_path = [
            self._apply_grasp_base_offset(self._transform_point(transform, point))
            for point in camera_path
        ]
        transform_done = time.monotonic()
        pointcloud_shape = self._estimate_pointcloud_grasp_shape_base(
            roi_points,
            transform,
            visual_axis_camera,
            visual_axis_confidence,
        )
        shape_done = time.monotonic()
        if pointcloud_shape is not None and pointcloud_shape.z_bias_m > 1e-6:
            base_grasp_path = self._apply_visible_upper_half_bias(
                base_grasp_path, pointcloud_shape.z_bias_m
            )
            pointcloud_shape = replace(
                pointcloud_shape,
                center_base=(
                    pointcloud_shape.center_base[0],
                    pointcloud_shape.center_base[1],
                    pointcloud_shape.center_base[2] - pointcloud_shape.z_bias_m,
                ),
            )
        start_base = self._current_grasp_origin_base()
        if start_base is None:
            start_base = self._frame_origin_in_base(END_EFFECTOR_FRAME)
        if start_base is None:
            start_base = self._transform_point(transform, (0.0, 0.0, 0.0))
        observe_base = start_base
        raw_pregrasp_base = base_grasp_path[0]
        grasp_base = base_grasp_path[1]
        raw_lift_base = base_grasp_path[2]
        grasp_base = (
            grasp_base[0],
            grasp_base[1],
            grasp_base[2] + float(self.args.grasp_final_z_offset_m),
        )
        if math.isfinite(float(self.args.fixed_grasp_z_base)):
            grasp_base = (grasp_base[0], grasp_base[1], float(self.args.fixed_grasp_z_base))
        if bool(self.args.vertical_approach):
            min_grasp_z = max(
                float(self.args.min_grasp_z_base),
                float(self.args.ground_min_z_base)
                + max(float(self.args.tool_ground_clearance), 0.0)
                + 0.015,
            )
            if grasp_base[2] < min_grasp_z:
                grasp_base = (grasp_base[0], grasp_base[1], min_grasp_z)
            approach_height = max(float(self.args.approach_distance), 0.08)
            lift_height = max(float(self.args.lift_distance), 0.08)
            approach_z = max(
                grasp_base[2] + approach_height,
                float(self.args.ground_min_z_base) + 0.18,
            )
            pregrasp_base = (grasp_base[0], grasp_base[1], approach_z)
            lift_base = (
                grasp_base[0],
                grasp_base[1],
                max(grasp_base[2] + lift_height, approach_z),
            )
        else:
            pregrasp_base = raw_pregrasp_base
            lift_base = raw_lift_base
        safe_mid_lift = max(float(self.args.safe_mid_lift_height), 0.0)
        transit_z = max(
            float(self.args.transit_height),
            grasp_base[2] + safe_mid_lift,
            lift_base[2],
            self.basket_approach_base[2],
        )
        safe_mid_base = (
            lift_base[0],
            lift_base[1],
            transit_z,
        )
        waypoints = [
            ("start_ee", start_base),
            ("approach", pregrasp_base),
            ("grasp", grasp_base),
            ("safe_mid", safe_mid_base),
            ("basket_over", self.basket_approach_base),
            ("drop", self.basket_release_base),
            ("observe_start", observe_base),
        ]
        waypoint_done = time.monotonic()
        raw_segments = [
            CartesianSegment(
                "quadratic",
                start_base,
                pregrasp_base,
                self._arc_control_point(start_base, pregrasp_base),
            ),
            CartesianSegment("line", pregrasp_base, grasp_base),
            CartesianSegment("line", grasp_base, lift_base),
            CartesianSegment(
                "quadratic",
                lift_base,
                safe_mid_base,
                self._arc_control_point(lift_base, safe_mid_base),
            ),
            CartesianSegment(
                "quadratic",
                safe_mid_base,
                self.basket_approach_base,
                self._arc_control_point(safe_mid_base, self.basket_approach_base),
            ),
            CartesianSegment("line", self.basket_approach_base, self.basket_release_base),
            CartesianSegment(
                "quadratic",
                self.basket_release_base,
                observe_base,
                self._arc_control_point(self.basket_release_base, observe_base),
            ),
        ]
        segments = self._time_parameterize_segments(raw_segments)
        segment_done = time.monotonic()
        path = self._sample_trajectory_for_display(segments)
        sample_done = time.monotonic()
        self.get_logger().info(
            "grasp_timing base_path "
            f"total={sample_done - start:.3f}s "
            f"tf={tf_done - start:.3f}s "
            f"camera_to_base={transform_done - tf_done:.3f}s "
            f"shape={shape_done - transform_done:.3f}s "
            f"waypoints={waypoint_done - shape_done:.3f}s "
            f"segments={segment_done - waypoint_done:.3f}s "
            f"sample={sample_done - segment_done:.3f}s "
            f"path_points={len(path)}"
        )
        return path, segments, waypoints, grasp_base, pointcloud_shape

    def _estimate_pointcloud_grasp_shape_base(
        self,
        roi_points: list[tuple[float, float, float]] | None,
        transform,
        visual_axis_camera: np.ndarray | None = None,
        visual_axis_confidence: float = 0.0,
    ) -> PointCloudGraspShape | None:
        if bool(self.args.disable_pointcloud_grasp_shape):
            return None
        visual_axis_base = self._visual_axis_base(transform, visual_axis_camera)
        if not roi_points:
            return self._visual_only_grasp_shape(visual_axis_base, visual_axis_confidence, 0)
        min_points = max(int(self.args.pointcloud_grasp_min_points), 6)
        if len(roi_points) < min_points:
            return self._visual_only_grasp_shape(
                visual_axis_base, visual_axis_confidence, len(roi_points)
            )
        base_points = np.asarray(
            [self._transform_point(transform, point) for point in roi_points],
            dtype=np.float64,
        )
        finite = np.all(np.isfinite(base_points), axis=1)
        base_points = base_points[finite]
        if base_points.shape[0] < min_points:
            return self._visual_only_grasp_shape(
                visual_axis_base, visual_axis_confidence, int(base_points.shape[0])
            )

        low, high = np.percentile(base_points, [5.0, 95.0], axis=0)
        trimmed = base_points[np.all((base_points >= low) & (base_points <= high), axis=1)]
        if trimmed.shape[0] >= min_points:
            base_points = trimmed

        center = np.median(base_points, axis=0)
        xy = base_points[:, :2]
        xy_center = np.median(xy, axis=0)
        centered_xy = xy - xy_center
        if centered_xy.shape[0] < 3:
            return None
        cov = np.cov(centered_xy.T)
        if not np.all(np.isfinite(cov)):
            return None
        eig_values, eig_vectors = np.linalg.eigh(cov)
        order = np.argsort(eig_values)[::-1]
        eig_values = eig_values[order]
        eig_vectors = eig_vectors[:, order]
        if float(eig_values[0]) <= 1e-10:
            return None

        major_xy = eig_vectors[:, 0].astype(np.float64)
        major_xy /= max(float(np.linalg.norm(major_xy)), 1e-9)
        if major_xy[0] < -1e-9 or (abs(major_xy[0]) < 1e-9 and major_xy[1] < 0.0):
            major_xy *= -1.0
        minor_xy = np.asarray([-major_xy[1], major_xy[0]], dtype=np.float64)

        major_projection = centered_xy @ major_xy
        minor_projection = centered_xy @ minor_xy
        extent_major = float(np.percentile(major_projection, 90.0) - np.percentile(major_projection, 10.0))
        extent_minor = float(np.percentile(minor_projection, 90.0) - np.percentile(minor_projection, 10.0))
        extent_z = float(np.percentile(base_points[:, 2], 90.0) - np.percentile(base_points[:, 2], 10.0))
        anisotropy = float((eig_values[0] - eig_values[1]) / max(eig_values[0] + eig_values[1], 1e-9))
        density_score = min(float(base_points.shape[0]) / float(max(min_points * 2, 1)), 1.0)
        axis_confidence = float(np.clip(anisotropy * density_score, 0.0, 1.0))
        shifted_center = self._apply_grasp_base_offset(
            (float(center[0]), float(center[1]), float(center[2]))
        )
        z_bias = min(
            max(float(self.args.pointcloud_visible_upper_half_z_bias_ratio), 0.0)
            * max(extent_z, 0.0),
            max(float(self.args.pointcloud_visible_upper_half_z_bias_max), 0.0),
        )
        ground_limit = (
            float(self.args.ground_min_z_base)
            + max(float(self.args.tool_ground_clearance), 0.0)
            + 0.01
        )
        z_bias = min(z_bias, max(float(shifted_center[2]) - ground_limit, 0.0))
        return PointCloudGraspShape(
            center_base=shifted_center,
            major_axis_base=(float(major_xy[0]), float(major_xy[1]), 0.0),
            minor_axis_base=(float(minor_xy[0]), float(minor_xy[1]), 0.0),
            visual_axis_base=visual_axis_base,
            extent_major_m=max(extent_major, 0.0),
            extent_minor_m=max(extent_minor, 0.0),
            extent_z_m=max(extent_z, 0.0),
            axis_confidence=axis_confidence,
            visual_axis_confidence=float(np.clip(visual_axis_confidence, 0.0, 1.0))
            if visual_axis_base is not None
            else 0.0,
            point_count=int(base_points.shape[0]),
            z_bias_m=max(float(z_bias), 0.0),
        )

    def _visual_only_grasp_shape(
        self,
        visual_axis_base: tuple[float, float, float] | None,
        visual_axis_confidence: float,
        point_count: int,
    ) -> PointCloudGraspShape | None:
        if visual_axis_base is None:
            return None
        return PointCloudGraspShape(
            center_base=(0.0, 0.0, 0.0),
            major_axis_base=visual_axis_base,
            minor_axis_base=(-visual_axis_base[1], visual_axis_base[0], 0.0),
            visual_axis_base=visual_axis_base,
            extent_major_m=0.0,
            extent_minor_m=0.0,
            extent_z_m=0.0,
            axis_confidence=0.0,
            visual_axis_confidence=float(np.clip(visual_axis_confidence, 0.0, 1.0)),
            point_count=max(int(point_count), 0),
            z_bias_m=0.0,
        )

    def _visual_axis_base(
        self, transform, visual_axis_camera: np.ndarray | None
    ) -> tuple[float, float, float] | None:
        if visual_axis_camera is None:
            return None
        matrix = self._matrix_from_transform(transform)
        axis = np.asarray(matrix[:3, :3], dtype=np.float64) @ np.asarray(
            visual_axis_camera, dtype=np.float64
        )
        axis[2] = 0.0
        norm = float(np.linalg.norm(axis))
        if norm < 1e-6:
            return None
        axis /= norm
        if axis[0] < -1e-9 or (abs(axis[0]) < 1e-9 and axis[1] < 0.0):
            axis *= -1.0
        return (float(axis[0]), float(axis[1]), 0.0)

    def _apply_visible_upper_half_bias(
        self,
        base_grasp_path: list[tuple[float, float, float]],
        z_bias: float,
    ) -> list[tuple[float, float, float]]:
        if z_bias <= 0.0 or len(base_grasp_path) < 2:
            return base_grasp_path
        ground_limit = (
            float(self.args.ground_min_z_base)
            + max(float(self.args.tool_ground_clearance), 0.0)
            + 0.01
        )
        shifted: list[tuple[float, float, float]] = []
        for index, point in enumerate(base_grasp_path):
            if index in {1, 2}:
                z = max(float(point[2]) - z_bias, ground_limit)
                shifted.append((float(point[0]), float(point[1]), z))
            else:
                shifted.append(point)
        return shifted

    def _apply_grasp_base_offset(
        self, point_base: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        offset = np.asarray(self.grasp_base_offset, dtype=np.float64)
        point = np.asarray(point_base, dtype=np.float64)
        shifted = point + offset
        return (float(shifted[0]), float(shifted[1]), float(shifted[2]))

    def _time_parameterize_segments(
        self, segments: list[CartesianSegment]
    ) -> list[CartesianSegment]:
        if not segments:
            return []
        lengths = [max(self._segment_length(segment), 1e-6) for segment in segments]
        total = max(sum(lengths), 1e-6)
        period = max(float(self.args.playback_period), 0.5)
        return [
            CartesianSegment(
                kind=segment.kind,
                start=segment.start,
                end=segment.end,
                control=segment.control,
                duration=period * length / total,
            )
            for segment, length in zip(segments, lengths)
        ]

    def _sample_trajectory_for_display(
        self, segments: list[CartesianSegment]
    ) -> list[tuple[float, float, float]]:
        path: list[tuple[float, float, float]] = []
        for segment in segments:
            if segment.kind == "line" and segment.start == self.basket_approach_base:
                samples = self.args.basket_descent_samples
            else:
                samples = self.args.line_samples if segment.kind == "line" else self.args.arc_samples
            points = [
                self._sample_segment(segment, t)
                for t in np.linspace(0.0, 1.0, max(int(samples), 2))
            ]
            self._extend_path(path, points)
        return path

    def _segment_length(self, segment: CartesianSegment) -> float:
        points = [self._sample_segment(segment, t) for t in np.linspace(0.0, 1.0, 25)]
        return float(
            sum(
                np.linalg.norm(np.asarray(b, dtype=np.float64) - np.asarray(a, dtype=np.float64))
                for a, b in zip(points, points[1:])
            )
        )

    def _sample_segment(
        self, segment: CartesianSegment, t: float
    ) -> tuple[float, float, float]:
        u = min(max(float(t), 0.0), 1.0)
        if segment.kind == "quadratic" and segment.control is not None:
            return self._quadratic_bezier_point(segment.start, segment.control, segment.end, u)
        a = np.asarray(segment.start, dtype=np.float64)
        b = np.asarray(segment.end, dtype=np.float64)
        point = a * (1.0 - u) + b * u
        return (float(point[0]), float(point[1]), float(point[2]))

    def _point_on_trajectory(
        self, segments: list[CartesianSegment], progress: float
    ) -> tuple[float, float, float]:
        if not segments:
            return (0.0, 0.0, 0.0)
        total = sum(max(float(segment.duration), 0.0) for segment in segments)
        if total <= 1e-9:
            return segments[0].start
        target_time = min(max(float(progress), 0.0), 1.0) * total
        elapsed = 0.0
        for segment in segments:
            duration = max(float(segment.duration), 1e-9)
            if elapsed + duration >= target_time:
                return self._sample_segment(segment, (target_time - elapsed) / duration)
            elapsed += duration
        return segments[-1].end

    def _base_point_to_aubo(
        self, point: tuple[float, float, float]
    ) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.aubo_base_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException as exc:
            self.preview_ik_message = f"waiting for TF {self.aubo_base_frame} <- {self.base_frame}: {exc}"
            return None
        return self._transform_point(transform, point)

    def _frame_origin_in_base(self, frame_id: str) -> tuple[float, float, float] | None:
        last_error = ""
        for candidate in self._frame_candidates(frame_id):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    candidate,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.03),
                )
            except TransformException as exc:
                last_error = str(exc)
                continue
            translation = transform.transform.translation
            return (float(translation.x), float(translation.y), float(translation.z))
        self._throttled_log(f"waiting for TF {self.base_frame} <- {frame_id}: {last_error}")
        return None

    def _extend_path(
        self, path: list[tuple[float, float, float]], segment: list[tuple[float, float, float]]
    ) -> None:
        for index, point in enumerate(segment):
            if path and index == 0:
                continue
            path.append(point)

    def _sample_line(
        self, start: tuple[float, float, float], end: tuple[float, float, float], samples: int
    ) -> list[tuple[float, float, float]]:
        count = max(int(samples), 2)
        a = np.asarray(start, dtype=np.float64)
        b = np.asarray(end, dtype=np.float64)
        return [
            tuple((a * (1.0 - t) + b * t).astype(float))
            for t in np.linspace(0.0, 1.0, count)
        ]

    def _sample_quadratic_bezier(
        self,
        start: tuple[float, float, float],
        control: tuple[float, float, float],
        end: tuple[float, float, float],
        samples: int,
    ) -> list[tuple[float, float, float]]:
        count = max(int(samples), 3)
        points = []
        for t in np.linspace(0.0, 1.0, count):
            points.append(self._quadratic_bezier_point(start, control, end, float(t)))
        return points

    def _quadratic_bezier_point(
        self,
        start: tuple[float, float, float],
        control: tuple[float, float, float],
        end: tuple[float, float, float],
        t: float,
    ) -> tuple[float, float, float]:
        a = np.asarray(start, dtype=np.float64)
        c = np.asarray(control, dtype=np.float64)
        b = np.asarray(end, dtype=np.float64)
        one = 1.0 - t
        point = one * one * a + 2.0 * one * t * c + t * t * b
        return (float(point[0]), float(point[1]), float(point[2]))

    def _arc_control_point(
        self, start: tuple[float, float, float], end: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        sx, sy, sz = start
        ex, ey, ez = end
        return (
            0.5 * (sx + ex),
            0.5 * (sy + ey),
            max(sz, ez, float(self.args.transit_height)) + max(float(self.args.transit_arc_height), 0.0),
        )

    def _transform_point(self, transform, xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = self._quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        vec = np.asarray(xyz, dtype=np.float64)
        out = matrix @ vec + np.asarray([translation.x, translation.y, translation.z])
        return (float(out[0]), float(out[1]), float(out[2]))

    def _matrix_from_transform(self, transform) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self._quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        matrix[:3, 3] = np.asarray([translation.x, translation.y, translation.z], dtype=np.float64)
        return matrix

    def _invert_rigid(self, transform: np.ndarray) -> np.ndarray:
        rotation = np.asarray(transform[:3, :3], dtype=np.float64)
        translation = np.asarray(transform[:3, 3], dtype=np.float64)
        inverse = np.eye(4, dtype=np.float64)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -(rotation.T @ translation)
        return inverse

    def _transform_matrix_point(self, transform: np.ndarray, xyz: np.ndarray) -> tuple[float, float, float]:
        vec = np.asarray([float(xyz[0]), float(xyz[1]), float(xyz[2]), 1.0], dtype=np.float64)
        out = np.asarray(transform, dtype=np.float64) @ vec
        return (float(out[0]), float(out[1]), float(out[2]))

    def _quat_to_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        norm = max((x * x + y * y + z * z + w * w) ** 0.5, 1e-9)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.asarray(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ],
            dtype=np.float64,
        )

    def _quat_msg_from_matrix(self, matrix: np.ndarray) -> Quaternion:
        rotation = np.asarray(matrix, dtype=np.float64)
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (rotation[2, 1] - rotation[1, 2]) / scale
            y = (rotation[0, 2] - rotation[2, 0]) / scale
            z = (rotation[1, 0] - rotation[0, 1]) / scale
        else:
            diagonal = np.diag(rotation)
            index = int(np.argmax(diagonal))
            if index == 0:
                scale = math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12)) * 2.0
                w = (rotation[2, 1] - rotation[1, 2]) / scale
                x = 0.25 * scale
                y = (rotation[0, 1] + rotation[1, 0]) / scale
                z = (rotation[0, 2] + rotation[2, 0]) / scale
            elif index == 1:
                scale = math.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12)) * 2.0
                w = (rotation[0, 2] - rotation[2, 0]) / scale
                x = (rotation[0, 1] + rotation[1, 0]) / scale
                y = 0.25 * scale
                z = (rotation[1, 2] + rotation[2, 1]) / scale
            else:
                scale = math.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12)) * 2.0
                w = (rotation[1, 0] - rotation[0, 1]) / scale
                x = (rotation[0, 2] + rotation[2, 0]) / scale
                y = (rotation[1, 2] + rotation[2, 1]) / scale
                z = 0.25 * scale
        norm = max(math.sqrt(x * x + y * y + z * z + w * w), 1e-9)
        return Quaternion(x=float(x / norm), y=float(y / norm), z=float(z / norm), w=float(w / norm))

    def _rpy_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        return rz @ ry @ rx

    def _basket_path_safe(self, points: list[tuple[float, float, float]]) -> bool:
        if not points:
            return False
        radius = max(float(self.args.gripper_radius), 0.0)
        mins = np.asarray(self.basket_keepout_min, dtype=np.float64) - radius
        maxs = np.asarray(self.basket_keepout_max, dtype=np.float64) + radius
        for start, end in zip(points, points[1:]):
            a = np.asarray(start, dtype=np.float64)
            b = np.asarray(end, dtype=np.float64)
            for t in np.linspace(0.0, 1.0, 25):
                p = a * (1.0 - t) + b * t
                if np.all(p >= mins) and np.all(p <= maxs):
                    return False
        return True

    def _markers(self, preview: GraspPreview, header: Header) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        bbox = self._marker(header, "bbox", 1, Marker.LINE_LIST, _color(1.0, 0.55, 0.0))
        bbox.scale.x = 0.008
        corners = preview.bbox_points
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            bbox.points.extend([_point(corners[a]), _point(corners[b])])
        markers.markers.append(bbox)

        path = self._marker(header, "camera_path", 2, Marker.LINE_STRIP, _color(0.0, 0.8, 1.0))
        path.scale.x = 0.012
        for xyz in [
            preview.pregrasp_xyz,
            preview.grasp_xyz,
            preview.lift_xyz,
            preview.retreat_xyz,
        ]:
            path.points.append(_point(xyz))
        markers.markers.append(path)

        arrow = self._marker(header, "approach", 3, Marker.ARROW, _color(0.0, 1.0, 0.25))
        arrow.scale.x = 0.018
        arrow.scale.y = 0.035
        arrow.scale.z = 0.035
        arrow.points.extend([_point(preview.pregrasp_xyz), _point(preview.grasp_xyz)])
        markers.markers.append(arrow)

        markers.markers.append(
            self._sphere(header, "pregrasp", 4, preview.pregrasp_xyz, _color(1.0, 0.85, 0.0))
        )
        markers.markers.append(
            self._sphere(header, "grasp", 5, preview.grasp_xyz, _color(0.0, 1.0, 0.2))
        )
        markers.markers.append(
            self._sphere(header, "lift", 6, preview.lift_xyz, _color(0.35, 0.45, 1.0))
        )

        text = self._marker(header, "label", 7, Marker.TEXT_VIEW_FACING, _color(1.0, 1.0, 1.0))
        text.pose.position = _point(preview.lift_xyz)
        text.pose.position.y -= 0.04
        text.scale.z = 0.055
        gx, gy, gz = preview.grasp_xyz
        text.text = (
            f"{preview.detection.label} {preview.detection.confidence:.2f}\n"
            f"depth {preview.depth_m:.3f} m\n"
            f"grasp [{gx:.3f}, {gy:.3f}, {gz:.3f}]\n"
            f"3D {preview.snapshot_reason}\n"
            f"IK {preview.ik_message}\n"
            f"basket {'clear' if preview.basket_safe else 'risk'}"
        )
        markers.markers.append(text)
        if preview.base_path_xyz:
            markers.markers.extend(self._basket_markers(preview))
        return markers

    def _basket_markers(self, preview: GraspPreview) -> list[Marker]:
        if not preview.base_path_xyz:
            return []
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame
        safe_color = _color(0.0, 0.95, 0.25) if preview.basket_safe else _color(1.0, 0.0, 0.0)
        markers: list[Marker] = []

        basket_path = self._marker(header, "planned_grasp_path", 20, Marker.LINE_STRIP, safe_color)
        basket_path.scale.x = 0.018
        for xyz in preview.base_path_xyz:
            basket_path.points.append(_point(xyz))
        markers.append(basket_path)
        markers.extend(self._base_waypoint_markers(header, preview))
        markers.extend(self._path_playback_markers(header, preview))

        for idx, (name, xyz, color) in enumerate(
            [
                ("basket_approach", self.basket_approach_base, _color(0.0, 0.65, 1.0)),
                ("basket_release", self.basket_release_base, _color(1.0, 0.35, 0.0)),
            ],
            start=21,
        ):
            markers.append(self._sphere(header, name, idx, xyz, color))

        keepout = self._marker(header, "basket_keepout", 30, Marker.CUBE, _color(1.0, 0.0, 0.0, 0.20))
        min_x, min_y, min_z = self.basket_keepout_min
        max_x, max_y, max_z = self.basket_keepout_max
        keepout.pose.position = _point(
            ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)
        )
        keepout.scale.x = max_x - min_x
        keepout.scale.y = max_y - min_y
        keepout.scale.z = max_z - min_z
        markers.append(keepout)

        rear_keepout = self._marker(
            header, "rear_rack_keepout", 31, Marker.CUBE, _color(1.0, 0.35, 0.0, 0.14)
        )
        min_x, min_y, min_z = self.rear_rack_keepout_min
        max_x, max_y, max_z = self.rear_rack_keepout_max
        rear_keepout.pose.position = _point(
            ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)
        )
        rear_keepout.scale.x = max_x - min_x
        rear_keepout.scale.y = max_y - min_y
        rear_keepout.scale.z = max_z - min_z
        markers.append(rear_keepout)

        label = self._marker(header, "basket_label", 32, Marker.TEXT_VIEW_FACING, _color(1.0, 1.0, 1.0))
        label.pose.position = _point((self.basket_release_base[0], self.basket_release_base[1], self.basket_release_base[2] + 0.08))
        label.scale.z = 0.055
        label.text = "release over basket\nkeepout clear" if preview.basket_safe else "basket collision risk"
        markers.append(label)
        return markers

    def _base_waypoint_markers(self, header: Header, preview: GraspPreview) -> list[Marker]:
        colors = {
            "start_ee": _color(1.0, 1.0, 1.0),
            "approach": _color(1.0, 0.85, 0.0),
            "grasp": _color(0.0, 1.0, 0.2),
            "safe_mid": _color(0.0, 0.65, 1.0),
            "basket_over": _color(0.0, 0.85, 1.0),
            "drop": _color(1.0, 0.35, 0.0),
            "observe_start": _color(0.75, 0.4, 1.0),
        }
        markers: list[Marker] = []
        for index, (name, xyz) in enumerate(preview.base_waypoints):
            sphere = self._sphere(
                header,
                "task_waypoints",
                40 + index,
                xyz,
                colors.get(name, _color(1.0, 1.0, 1.0)),
            )
            sphere.scale.x = 0.045
            sphere.scale.y = 0.045
            sphere.scale.z = 0.045
            markers.append(sphere)

            text = self._marker(
                header,
                "task_waypoint_labels",
                50 + index,
                Marker.TEXT_VIEW_FACING,
                _color(1.0, 1.0, 1.0),
            )
            text.pose.position = _point((xyz[0], xyz[1], xyz[2] + 0.055))
            text.scale.z = 0.04
            text.text = name.replace("_", " ")
            markers.append(text)
        return markers

    def _path_playback_markers(self, header: Header, preview: GraspPreview) -> list[Marker]:
        if not preview.arm_trajectory_frames:
            return []
        cursor_xyz = self._current_grasp_position_base(preview)
        if cursor_xyz is None:
            return []
        progress = self._playback_progress(preview)

        cursor = self._sphere(
            header, "path_playback", 70, cursor_xyz, _color(1.0, 0.0, 1.0)
        )
        cursor.scale.x = 0.06
        cursor.scale.y = 0.06
        cursor.scale.z = 0.06

        label = self._marker(
            header, "path_playback", 71, Marker.TEXT_VIEW_FACING, _color(1.0, 1.0, 1.0)
        )
        label.pose.position = _point((cursor_xyz[0], cursor_xyz[1], cursor_xyz[2] + 0.07))
        label.scale.z = 0.045
        label.text = f"path playback {progress * 100.0:04.1f}%"
        return [cursor, label]

    def _playback_progress(self, preview: GraspPreview) -> float:
        duration = (
            max(float(preview.arm_trajectory_frames[-1].time_from_start), 1e-6)
            if preview.arm_trajectory_frames
            else max(float(self.args.playback_period), 0.5)
        )
        period_override = float(self.args.playback_period)
        period = max(period_override, duration) if period_override > 0.0 else duration
        start_time = self.plan_lock_time or self.snapshot_time or time.monotonic()
        elapsed = max(time.monotonic() - start_time, 0.0)
        t = elapsed % max(period, 1e-6) if bool(self.args.loop_playback) else min(elapsed, duration)
        return min(max(t / duration, 0.0), 1.0)

    def _marker(
        self, header: Header, namespace: str, marker_id: int, marker_type: int, color: ColorRGBA
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.color = color
        marker.pose.orientation.w = 1.0
        return marker

    def _sphere(
        self, header: Header, namespace: str, marker_id: int, xyz: tuple[float, float, float], color
    ) -> Marker:
        marker = self._marker(header, namespace, marker_id, Marker.SPHERE, color)
        marker.pose.position = _point(xyz)
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        return marker

    def _path(self, preview: GraspPreview, header: Header) -> PathMsg:
        msg = PathMsg()
        path_header = Header()
        path_header.stamp = self.get_clock().now().to_msg()
        path_header.frame_id = self.base_frame
        msg.header = path_header
        if preview.base_path_xyz:
            path_points = preview.base_path_xyz
        else:
            path_points = []
        for xyz in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = _point(xyz)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish_annotated(self, image: np.ndarray, preview: GraspPreview, header: Header) -> None:
        gx, gy, gz = preview.grasp_xyz
        state = "PLAN_LOCKED / YOLO paused" if self.inference_paused else "SEARCH_2D"
        cv2.putText(
            image,
            f"{state} depth={preview.depth_m:.3f}m {preview.snapshot_reason} "
            f"grasp=({gx:.2f},{gy:.2f},{gz:.2f}) "
            f"traj_pts={len(preview.base_path_xyz)} basket={'clear' if preview.basket_safe else 'risk'}",
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        self.image_pub.publish(_image_from_bgr(image, header))

    def _pointcloud_shape_payload(
        self, shape: PointCloudGraspShape | None
    ) -> dict[str, object] | None:
        if shape is None:
            return None
        return {
            "center_base": shape.center_base,
            "major_axis_base": shape.major_axis_base,
            "minor_axis_base": shape.minor_axis_base,
            "visual_axis_base": shape.visual_axis_base,
            "extent_major_m": shape.extent_major_m,
            "extent_minor_m": shape.extent_minor_m,
            "extent_z_m": shape.extent_z_m,
            "axis_confidence": shape.axis_confidence,
            "visual_axis_confidence": shape.visual_axis_confidence,
            "point_count": shape.point_count,
            "visible_upper_half_z_bias_m": shape.z_bias_m,
        }

    def _save_latest(
        self, annotated: np.ndarray, raw: np.ndarray, preview: GraspPreview
    ) -> None:
        cv2.imwrite(str(self.save_dir / "latest_raw.jpg"), raw)
        cv2.imwrite(str(self.save_dir / "latest_annotated.jpg"), annotated)
        if preview.snapshot_reason != "cached":
            key = (int(self.snapshot_count), str(preview.snapshot_reason))
            if key != self.last_saved_detection_key:
                self.last_saved_detection_key = key
                label = re.sub(r"[^A-Za-z0-9_.-]+", "_", preview.detection.label)[:40] or "target"
                reason = re.sub(r"[^A-Za-z0-9_.-]+", "_", preview.snapshot_reason)[:40] or "snapshot"
                stem = f"{self.snapshot_count:04d}_{reason}_{label}_{preview.detection.confidence:.2f}"
                out_dir = self.save_dir / "detections"
                out_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out_dir / f"{stem}_raw.jpg"), raw)
                cv2.imwrite(str(out_dir / f"{stem}_annotated.jpg"), annotated)
        payload = {
            "label": preview.detection.label,
            "class_id": preview.detection.class_id,
            "confidence": preview.detection.confidence,
            "bbox_xyxy": preview.detection.xyxy,
            "has_mask": preview.detection.mask_xy is not None,
            "mask_area_px": preview.detection.mask_area_px,
            "roi_source": preview.roi_source,
            "depth_m": preview.depth_m,
            "grasp_camera_xyz": preview.grasp_xyz,
            "path_camera_xyz": [
                preview.pregrasp_xyz,
                preview.grasp_xyz,
                preview.lift_xyz,
                preview.retreat_xyz,
            ],
            "depth_projection": {
                "flip_x": bool(self.args.depth_projection_flip_x),
                "flip_y": bool(self.args.depth_projection_flip_y),
            },
            "grasp_base_offset_xyz": self.grasp_base_offset,
            "pointcloud_grasp_shape": self._pointcloud_shape_payload(preview.pointcloud_shape),
            "planned_grasp_path_base_xyz": preview.base_path_xyz,
            "planned_grasp_path_source": (
                "fk_from_arm_trajectory_frames" if preview.arm_trajectory_frames else "pending"
                if not preview.base_path_xyz
                else "cartesian_reference"
            ),
            "trajectory_segments_base": [
                {
                    "kind": segment.kind,
                    "start": segment.start,
                    "control": segment.control,
                    "end": segment.end,
                    "duration": segment.duration,
                }
                for segment in preview.base_trajectory_segments
            ],
            "waypoints_base": [
                {"name": name, "xyz": xyz} for name, xyz in preview.base_waypoints
            ],
            "planning_key_waypoints": [
                {"name": name, "xyz": xyz}
                for name, xyz, _progress in self._planning_target_samples(preview)
            ],
            "trajectory_mode": "constraint_generated_joint_trajectory",
            "end_effector_frame": END_EFFECTOR_FRAME,
            "planning_tcp": {
                "source": (
                    "tool0_virtual_z_offset"
                    if float(self.args.grasp_tcp_offset_m) > 0.0
                    else "urdf_grasp_frame"
                ),
                "tool0_z_offset_m": max(float(self.args.grasp_tcp_offset_m), 0.0),
                "tool_to_grasp_frames": self.tool_to_grasp_frames,
            },
            "path_playback": True,
            "path_playback_loop": bool(self.args.loop_playback),
            "path_playback_period": (
                max(float(self.args.playback_period), 0.5)
                if float(self.args.playback_period) > 0.0
                else (
                    float(preview.arm_trajectory_frames[-1].time_from_start)
                    if preview.arm_trajectory_frames
                    else 0.0
                )
            ),
            "path_playback_rate_hz": max(float(self.args.playback_rate), 1.0),
            "planner_backend": str(self.args.planner_backend),
            "grasp_strategy": {
                "vertical_approach": bool(self.args.vertical_approach),
                "lock_grasp_orientation": bool(self.args.lock_grasp_orientation),
                "tool_orientation_limit_deg": float(self.args.tool_orientation_limit_deg),
                "max_grasp_orientation_candidates": int(self.args.max_grasp_orientation_candidates),
                "local_strict_ik": bool(self.args.local_strict_ik),
                "real_sdk_semantic_targets_only": bool(self.args.real_sdk_semantic_targets_only),
                "safe_mid_lift_height_m": float(self.args.safe_mid_lift_height),
            },
            "moveit_plan_service": str(self.args.moveit_plan_service),
            "moveit_planners": [
                token.strip()
                for token in str(self.args.moveit_planners).split(",")
                if token.strip()
            ],
            "ik_sampling": "moveit_goal_waypoints" if str(self.args.planner_backend) == "moveit" else "offline_reference_points_before_constraint_generation",
            "ik_mode": "moveit_ompl_motion_plan" if str(self.args.planner_backend) == "moveit" else "local_pose_ik_then_constrained_joint_generation",
            "joint_stream_limits": {
                "max_speed_rad_sec": max(float(self.args.preview_max_joint_speed), 0.01),
                "max_accel_rad_sec2": max(float(self.args.preview_max_joint_accel), 0.05),
                "max_jerk_rad_sec3": max(float(self.args.preview_max_joint_jerk), 0.1),
                "smoothing_tau_sec": max(float(self.args.preview_smoothing_tau), 0.02),
            },
            "ground_safety": {
                "ground_min_z_base": float(self.args.ground_min_z_base),
                "min_grasp_z_base": float(self.args.min_grasp_z_base),
                "ground_clearance": float(self.args.ground_clearance),
                "tool_ground_clearance": float(self.args.tool_ground_clearance),
                "arm_collision_radius": float(self.args.arm_collision_radius),
                "collision_margin": float(self.args.collision_margin),
                "rear_rack_collision_margin": float(self.args.rear_rack_collision_margin),
                "grasp_topdown_max_tilt_deg": float(self.args.grasp_topdown_max_tilt_deg),
            },
            "gripper_preview": {
                "gripper_type": str(self.args.gripper_type),
                "close_waypoint": str(self.args.gripper_close_waypoint),
                "open_waypoint": str(self.args.gripper_open_waypoint),
                "transition_progress": float(self.args.gripper_transition_progress),
                "close_progress": [
                    float(self.args.gripper_close_progress_start),
                    float(self.args.gripper_close_progress_end),
                ],
                "open_progress": [
                    float(self.args.gripper_open_progress_start),
                    float(self.args.gripper_open_progress_end),
                ],
            },
            "real_execution": {
                "enabled": bool(self.args.execute_real),
                "confirmed": str(self.args.execute_real_confirm).strip().upper() == "YES",
                "backend": str(self.args.real_execute_backend),
                "action": str(self.args.real_follow_action),
                "sdk_rpc": f"{self.args.real_sdk_ip}:{int(self.args.real_sdk_rpc_port)}",
                "sdk_prewarm": bool(self.args.real_sdk_prewarm),
                "sdk_prewarm_wait_sec": float(self.args.real_sdk_prewarm_wait_sec),
                "sdk_move_speed_rad_sec": float(self.args.real_sdk_move_speed),
                "sdk_move_accel_rad_sec2": float(self.args.real_sdk_move_accel),
                "joint_states_topic": str(self.args.real_joint_states_topic),
                "arm_joint_names": self._real_arm_joint_names()
                if bool(self.args.execute_real)
                else list(REAL_ARM_JOINT_NAMES),
                "allow_partial": bool(self.args.real_allow_partial),
                "trajectory_point_stride": max(int(self.args.real_trajectory_point_stride), 1),
                "execute_gripper": bool(self.args.real_execute_gripper),
                "gripper_command_topic": str(self.args.real_gripper_command_topic),
            },
            "arm_trajectory_frame_count": len(preview.arm_trajectory_frames),
            "arm_trajectory_joint_angle_convention": "continuous_unwrapped",
            "arm_trajectory_duration_sec": (
                float(preview.arm_trajectory_frames[-1].time_from_start)
                if preview.arm_trajectory_frames
                else 0.0
            ),
            "arm_trajectory_frames": [
                {
                    "time_from_start": frame.time_from_start,
                    "positions": frame.positions,
                    "velocities": frame.velocities,
                    "accelerations": frame.accelerations,
                }
                for frame in preview.arm_trajectory_frames
            ],
            "depth_snapshot_policy": "snapshot_once_then_lock_until_restart",
            "snapshot_reason": preview.snapshot_reason,
            "snapshot_count": self.snapshot_count,
            "state": "PLAN_LOCKED" if self.inference_paused else "SEARCH_2D",
            "inference_paused": self.inference_paused,
            "restart_search_topic": self.args.restart_search_topic,
            "base_grasp_xyz": preview.base_grasp_xyz,
            "arm_joint_names": list(ARM_JOINT_NAMES),
            "arm_joint_path": [],
            "arm_joint_path_precomputed": False,
            "ik_message": self.preview_ik_message,
            "yolo_inference_count": self.inference_count,
            "basket_release_base": self.basket_release_base,
            "basket_keepout_min_base": self.basket_keepout_min,
            "basket_keepout_max_base": self.basket_keepout_max,
            "rear_rack_keepout_min_base": self.rear_rack_keepout_min,
            "rear_rack_keepout_max_base": self.rear_rack_keepout_max,
            "basket_safe": preview.basket_safe,
            "roi_points": len(preview.roi_points),
        }
        (self.save_dir / "latest_grasp_preview.json").write_text(
            json.dumps(payload, indent=2), encoding="utf-8"
        )

    def _clear_markers(self, header: Header) -> None:
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        self.markers_pub.publish(MarkerArray(markers=[clear]))

    def _throttled_log(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_log_time > 1.5:
            self.last_log_time = now
            self.get_logger().info(message)


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = GraspPreviewNode(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
