from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _fourcc(code: str) -> int:
    padded = (code + "    ")[:4]
    return cv2.VideoWriter_fourcc(*padded)


def _bool_param(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _camera_info(width: int, height: int, frame_id: str, fx: float, fy: float) -> CameraInfo:
    msg = CameraInfo()
    msg.width = int(width)
    msg.height = int(height)
    msg.header.frame_id = frame_id
    cx = (float(width) - 1.0) * 0.5
    cy = (float(height) - 1.0) * 0.5
    fx = float(fx) if fx > 0.0 else float(width) * 0.9
    fy = float(fy) if fy > 0.0 else fx
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


# factory_calibration.json schema (exported by scripts/vision/orbbec_factory_intrinsics.sh):
# {
#   "format": "arachne_gemini335_factory_calibration_v1",
#   "sensor": "Gemini 335",
#   "exported_at": "...",
#   "sdk_version": "v1.10.27",
#   "streams": {
#     "color": {
#       "image_size": [640, 480],
#       "intrinsic": {"fx": ..., "fy": ..., "cx": ..., "cy": ...},
#       "distortion": [k1, k2, k3, k4, k5, k6, p1, p2],   # SDK OBCameraDistortion field order
#       "distortion_model": "modified_brown_conrady",
#       "is_mirrored": false
#     },
#     "depth": { ... }
#   },
#   "transform_depth_to_color": {"rot": [...], "trans": [...]}
# }
def _load_factory_intrinsics(path: str) -> dict | None:
    """Load factory calibration JSON; return {"color": entry, "depth": entry} or None on any failure.

    A single bad stream entry invalidates only that stream: the dict may contain only one of
    "color"/"depth". Never raises.
    """
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return None
    streams = data.get("streams") if isinstance(data, dict) else None
    if not isinstance(streams, dict):
        return None
    result: dict[str, dict] = {}
    for key in ("color", "depth"):
        entry = streams.get(key)
        if not isinstance(entry, dict):
            continue
        intrinsic = entry.get("intrinsic")
        distortion = entry.get("distortion")
        image_size = entry.get("image_size")
        ok = (
            isinstance(intrinsic, dict)
            and all(isinstance(intrinsic.get(f), (int, float)) for f in ("fx", "fy", "cx", "cy"))
            and isinstance(distortion, list)
            and len(distortion) >= 8
            and all(isinstance(v, (int, float)) for v in distortion)
            and isinstance(image_size, list)
            and len(image_size) == 2
            and all(isinstance(v, (int, float)) for v in image_size)
        )
        if ok:
            result[key] = {
                "image_size": [int(image_size[0]), int(image_size[1])],
                "intrinsic": {
                    "fx": float(intrinsic["fx"]),
                    "fy": float(intrinsic["fy"]),
                    "cx": float(intrinsic["cx"]),
                    "cy": float(intrinsic["cy"]),
                },
                "distortion": [float(v) for v in distortion],
                "distortion_model": str(entry.get("distortion_model", "")),
                "is_mirrored": bool(entry.get("is_mirrored", False)),
            }
    return result or None


def _apply_factory_intrinsics(msg: CameraInfo, entry: dict) -> CameraInfo:
    """Fill CameraInfo K/D from a factory calibration stream entry.

    Orbbec 8-coeff distortion [k1..k6, p1, p2] maps to ROS plumb_bob
    [k1, k2, p1, p2, k3] (Brown-Conrady order; k4-k6 dropped).
    """
    fx = entry["intrinsic"]["fx"]
    fy = entry["intrinsic"]["fy"]
    cx = entry["intrinsic"]["cx"]
    cy = entry["intrinsic"]["cy"]
    k1, k2, k3, _k4, _k5, _k6, p1, p2 = entry["distortion"][:8]
    msg.distortion_model = "plumb_bob"
    msg.d = [k1, k2, p1, p2, k3]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def _image_from_array(array: np.ndarray, header: Header, encoding: str) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    contiguous = np.ascontiguousarray(array)
    channels = 1 if contiguous.ndim == 2 else int(contiguous.shape[2])
    msg.step = int(contiguous.shape[1] * contiguous.dtype.itemsize * channels)
    msg.data = contiguous.tobytes()
    return msg


@dataclass
class DepthFrame:
    stamp: rclpy.time.Time
    data: np.ndarray


class Gemini335V4L2Node(Node):
    def __init__(self) -> None:
        super().__init__("gemini335_v4l2_node")

        self.declare_parameter("color_device", "/dev/video6")
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("color_fps", 30.0)
        self.declare_parameter("color_fourcc", "YUYV")
        self.declare_parameter("color_yuv_layout", "YUYV")
        self.declare_parameter("color_v4l2_controls", "")
        self.declare_parameter("color_capture_mode", "v4l2-ctl")
        self.declare_parameter("color_batch_frames", 30)
        self.declare_parameter("color_capture_timeout_sec", 4.0)
        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_color", True)

        self.declare_parameter("depth_device", "/dev/video0")
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 5.0)
        self.declare_parameter("depth_fourcc", "Z16 ")
        self.declare_parameter("depth_batch_frames", 3)
        self.declare_parameter("depth_capture_timeout_sec", 4.0)
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("pointcloud_min_depth_m", 0.05)
        self.declare_parameter("pointcloud_max_depth_m", 2.0)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("publish_depth_color", True)
        self.declare_parameter("publish_pointcloud", True)
        self.declare_parameter("pointcloud_decimation", 6)
        self.declare_parameter("pointcloud_rate", 5.0)
        self.declare_parameter("camera_fx", 0.0)
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("factory_intrinsics_file", "")
        self.declare_parameter("projection_flip_x", True)
        self.declare_parameter("projection_flip_y", True)

        self.publish_color = _bool_param(self.get_parameter("publish_color").value)
        self.publish_depth = _bool_param(self.get_parameter("publish_depth").value)
        self.publish_depth_color = _bool_param(self.get_parameter("publish_depth_color").value)
        self.publish_pointcloud = _bool_param(self.get_parameter("publish_pointcloud").value)

        self.color_device = str(self.get_parameter("color_device").value)
        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.color_fps = float(self.get_parameter("color_fps").value)
        self.color_v4l2_controls = str(self.get_parameter("color_v4l2_controls").value)
        self.color_capture_mode = str(self.get_parameter("color_capture_mode").value).strip().lower()
        self.color_batch_frames = max(int(self.get_parameter("color_batch_frames").value), 1)
        self.color_capture_timeout_sec = max(
            float(self.get_parameter("color_capture_timeout_sec").value), 0.5
        )
        self.color_frame_id = str(self.get_parameter("color_frame_id").value)

        self.depth_device = str(self.get_parameter("depth_device").value)
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.depth_fps = float(self.get_parameter("depth_fps").value)
        self.depth_batch_frames = max(int(self.get_parameter("depth_batch_frames").value), 1)
        self.depth_capture_timeout_sec = max(
            float(self.get_parameter("depth_capture_timeout_sec").value), 0.5
        )
        self.depth_frame_id = str(self.get_parameter("depth_frame_id").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.pointcloud_min_depth_m = max(
            float(self.get_parameter("pointcloud_min_depth_m").value), 0.0
        )
        self.pointcloud_max_depth_m = max(
            float(self.get_parameter("pointcloud_max_depth_m").value),
            self.pointcloud_min_depth_m,
        )
        self.pointcloud_decimation = max(int(self.get_parameter("pointcloud_decimation").value), 1)
        self.pointcloud_period = 1.0 / max(float(self.get_parameter("pointcloud_rate").value), 0.1)
        self.camera_fx = float(self.get_parameter("camera_fx").value)
        self.camera_fy = float(self.get_parameter("camera_fy").value)
        self.projection_flip_x = _bool_param(self.get_parameter("projection_flip_x").value)
        self.projection_flip_y = _bool_param(self.get_parameter("projection_flip_y").value)

        factory_path = str(self.get_parameter("factory_intrinsics_file").value).strip()
        self.factory_intrinsics = _load_factory_intrinsics(factory_path) if factory_path else None
        if factory_path and self.factory_intrinsics is None:
            self.get_logger().warning(
                f"factory_intrinsics_file {factory_path!r} missing or invalid; "
                "falling back to camera_fx/camera_fy parameters or legacy width*0.9 guess"
            )

        self.color_info = _camera_info(
            self.color_width, self.color_height, self.color_frame_id, self.camera_fx, self.camera_fy
        )
        self.depth_info = _camera_info(
            self.depth_width, self.depth_height, self.depth_frame_id, self.camera_fx, self.camera_fy
        )
        color_entry = self.factory_intrinsics.get("color") if self.factory_intrinsics else None
        depth_entry = self.factory_intrinsics.get("depth") if self.factory_intrinsics else None
        if color_entry is not None and (
            color_entry["image_size"][0] != self.color_width or color_entry["image_size"][1] != self.color_height
        ):
            self.get_logger().warning(
                f"color factory intrinsics image_size {color_entry['image_size']} does not match "
                f"configured stream {self.color_width}x{self.color_height}; ignoring factory intrinsics"
            )
            color_entry = None
        if depth_entry is not None and (
            depth_entry["image_size"][0] != self.depth_width or depth_entry["image_size"][1] != self.depth_height
        ):
            self.get_logger().warning(
                f"depth factory intrinsics image_size {depth_entry['image_size']} does not match "
                f"configured stream {self.depth_width}x{self.depth_height}; ignoring factory intrinsics"
            )
            depth_entry = None
        if color_entry is not None:
            if color_entry["is_mirrored"]:
                self.get_logger().warning(
                    "color factory intrinsics is_mirrored=true; verify cx/cy and distortion match the raw (unmirrored) stream"
                )
            self.color_info = _apply_factory_intrinsics(self.color_info, color_entry)
            self.get_logger().info(
                f"color camera_info uses factory intrinsics fx={color_entry['intrinsic']['fx']:.4f} "
                f"fy={color_entry['intrinsic']['fy']:.4f} cx={color_entry['intrinsic']['cx']:.4f} "
                f"cy={color_entry['intrinsic']['cy']:.4f}"
            )
        else:
            self.get_logger().info(
                f"color camera_info uses fx={self.color_info.k[0]:.4f} fy={self.color_info.k[4]:.4f} "
                f"cx={self.color_info.k[2]:.4f} cy={self.color_info.k[5]:.4f} "
                "(factory intrinsics unavailable)"
            )
        if depth_entry is not None:
            if depth_entry["is_mirrored"]:
                self.get_logger().warning(
                    "depth factory intrinsics is_mirrored=true; verify cx/cy and distortion match the raw (unmirrored) stream"
                )
            self.depth_info = _apply_factory_intrinsics(self.depth_info, depth_entry)
            self.get_logger().info(
                f"depth camera_info uses factory intrinsics fx={depth_entry['intrinsic']['fx']:.4f} "
                f"fy={depth_entry['intrinsic']['fy']:.4f} cx={depth_entry['intrinsic']['cx']:.4f} "
                f"cy={depth_entry['intrinsic']['cy']:.4f}"
            )
        else:
            self.get_logger().info(
                f"depth camera_info uses fx={self.depth_info.k[0]:.4f} fy={self.depth_info.k[4]:.4f} "
                f"cx={self.depth_info.k[2]:.4f} cy={self.depth_info.k[5]:.4f} "
                "(factory intrinsics unavailable)"
            )

        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.color_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.depth_color_pub = self.create_publisher(Image, "/camera/depth/image_color", 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        self.points_pub = (
            self.create_publisher(PointCloud2, "/camera/points", 10)
            if self.publish_pointcloud
            else None
        )

        self.color_capture: cv2.VideoCapture | None = None
        self.color_process: subprocess.Popen[bytes] | None = None
        self.color_process_lock = threading.Lock()
        self.color_tmp_dir: str | None = None
        self.color_raw_path: str | None = None
        self.color_thread: threading.Thread | None = None
        self.depth_process: subprocess.Popen[bytes] | None = None
        self.depth_process_lock = threading.Lock()
        self.depth_tmp_dir: str | None = None
        self.depth_raw_path: str | None = None
        self.depth_thread: threading.Thread | None = None
        self.depth_lock = threading.Lock()
        self.latest_depth: DepthFrame | None = None
        self.stop_event = threading.Event()
        self.last_pointcloud_stamp = 0.0
        self.last_color_warning = 0.0
        self.last_depth_warning = 0.0
        self.color_frame_count = 0

        if self.publish_color:
            self._start_color_stream()
        if self.publish_depth or self.publish_depth_color or self.publish_pointcloud:
            self._start_depth_stream()

        self.get_logger().info(
            "Gemini335 V4L2 node ready: "
            f"color={self.color_device} depth={self.depth_device} "
            f"publish_color={self.publish_color} publish_depth={self.publish_depth} "
            f"publish_depth_color={self.publish_depth_color} "
            f"publish_pointcloud={self.publish_pointcloud} "
            f"topics=/camera/color/image_raw,/camera/depth/image_raw,/camera/points"
        )

    def destroy_node(self) -> bool:
        self.stop_event.set()
        with self.color_process_lock:
            color_process = self.color_process
        if color_process is not None:
            color_process.terminate()
            try:
                color_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                color_process.kill()
        with self.depth_process_lock:
            depth_process = self.depth_process
        if depth_process is not None:
            depth_process.terminate()
            try:
                depth_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                depth_process.kill()
        if self.color_thread is not None and self.color_thread.is_alive():
            self.color_thread.join(timeout=2.0)
        if self.depth_thread is not None and self.depth_thread.is_alive():
            self.depth_thread.join(timeout=2.0)
        if self.color_raw_path is not None:
            try:
                os.unlink(self.color_raw_path)
            except OSError:
                pass
        if self.color_tmp_dir is not None:
            try:
                os.rmdir(self.color_tmp_dir)
            except OSError:
                pass
        if self.depth_raw_path is not None:
            try:
                os.unlink(self.depth_raw_path)
            except OSError:
                pass
        if self.depth_tmp_dir is not None:
            try:
                os.rmdir(self.depth_tmp_dir)
            except OSError:
                pass
        if self.color_capture is not None:
            self.color_capture.release()
        return super().destroy_node()

    def _open_color(self) -> None:
        cap = cv2.VideoCapture(self.color_device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open color device {self.color_device}")
        fourcc = str(self.get_parameter("color_fourcc").value)
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, _fourcc(fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.color_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.color_height)
        cap.set(cv2.CAP_PROP_FPS, self.color_fps)
        self._apply_color_v4l2_controls()
        self.color_capture = cap

    def _apply_color_v4l2_controls(self) -> None:
        controls = [
            item.strip()
            for item in self.color_v4l2_controls.replace(";", ",").split(",")
            if item.strip()
        ]
        for control in controls:
            result = subprocess.run(
                [
                    "v4l2-ctl",
                    "--silent",
                    "-d",
                    self.color_device,
                    f"--set-ctrl={control}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            if result.returncode != 0:
                detail = result.stderr.strip()
                suffix = f": {detail}" if detail else ""
                self._warn_color(f"Gemini335 color control ignored {control!r}{suffix}")

    def _start_depth_stream(self) -> None:
        if not os.path.exists(self.depth_device):
            raise RuntimeError(f"depth device does not exist: {self.depth_device}")
        self.depth_tmp_dir = tempfile.mkdtemp(prefix="arachne_gemini335_")
        self.depth_raw_path = os.path.join(self.depth_tmp_dir, "depth_z16.raw")
        self.depth_thread = threading.Thread(
            target=self._depth_capture_worker,
            daemon=True,
        )
        self.depth_thread.start()

    def _start_color_stream(self) -> None:
        if not os.path.exists(self.color_device):
            raise RuntimeError(f"color device does not exist: {self.color_device}")
        self.get_logger().info(
            f"starting Gemini335 color stream: {self.color_device} "
            f"{self.color_width}x{self.color_height}@{self.color_fps:g} "
            f"mode={self.color_capture_mode}"
        )
        target = (
            self._color_capture_worker_opencv
            if self.color_capture_mode in ("opencv", "cv2")
            else self._color_capture_worker
        )
        self.color_thread = threading.Thread(
            target=target,
            daemon=True,
        )
        self.color_thread.start()

    def _color_capture_worker_opencv(self) -> None:
        """Capture color frames via OpenCV cv2.VideoCapture (blocking read loop)."""
        self.get_logger().info("Gemini335 color OpenCV capture worker started")
        while not self.stop_event.is_set():
            try:
                if self.color_capture is None or not self.color_capture.isOpened():
                    self._open_color()
                    if self.color_capture is None or not self.color_capture.isOpened():
                        self._warn_color(
                            "Gemini335 color OpenCV failed to open device, retrying..."
                        )
                        self.stop_event.wait(1.0)
                        continue
                while not self.stop_event.is_set():
                    ok, frame = self.color_capture.read()
                    if not ok or frame is None:
                        self._warn_color(
                            "Gemini335 color OpenCV read failed, reopening device..."
                        )
                        break
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    self._publish_color_array(frame)
            except Exception as exc:
                self._warn_color(f"Gemini335 color OpenCV error: {exc}")
            finally:
                if self.color_capture is not None:
                    try:
                        self.color_capture.release()
                    except Exception:
                        pass
                    self.color_capture = None
            if not self.stop_event.is_set():
                self.stop_event.wait(0.5)
        self.get_logger().info("Gemini335 color OpenCV capture worker stopped")

    def _color_capture_worker(self) -> None:
        pixelformat = (str(self.get_parameter("color_fourcc").value) + "    ")[:4]
        bytes_per_pixel = 2
        if pixelformat not in {"YUYV", "YUY2", "UYVY"}:
            self._warn_color(
                f"unsupported color pixelformat for v4l2 streaming: {pixelformat}; use YUYV"
            )
            return
        frame_bytes = self.color_width * self.color_height * bytes_per_pixel
        period = 1.0 / max(self.color_fps, 0.1)
        configure_command = [
            "v4l2-ctl",
            "--silent",
            "-d",
            self.color_device,
            f"--set-fmt-video=width={self.color_width},height={self.color_height},pixelformat={pixelformat}",
            f"--set-parm={max(self.color_fps, 1.0):g}",
        ]
        command = [
            "v4l2-ctl",
            "--silent",
            "-d",
            self.color_device,
            "--stream-mmap=8",
            "--stream-count=0",
            "--stream-to=-",
        ]
        backoff = 0.0
        while not self.stop_event.is_set():
            start = time.monotonic()
            process: subprocess.Popen[bytes] | None = None
            try:
                subprocess.run(
                    configure_command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=True,
                )
                self._apply_color_v4l2_controls()
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                )
                with self.color_process_lock:
                    if self.stop_event.is_set():
                        return
                    self.color_process = process
                self.get_logger().info(f"Gemini335 color v4l2 stream pid={process.pid}")
                if process.stdout is None:
                    self._warn_color("Gemini335 color capture stdout is unavailable")
                    process.terminate()
                    self._sleep_color_period(start, period)
                    continue
                layout = str(self.get_parameter("color_yuv_layout").value).strip().upper()
                code = {
                    "UYVY": cv2.COLOR_YUV2BGR_UYVY,
                    "YUYV": cv2.COLOR_YUV2BGR_YUY2,
                    "YUY2": cv2.COLOR_YUV2BGR_YUY2,
                    "YVYU": cv2.COLOR_YUV2BGR_YVYU,
                }.get(layout)
                if code is None:
                    self._warn_color(f"unknown color_yuv_layout={layout}; falling back to YVYU")
                    code = cv2.COLOR_YUV2BGR_YVYU
                while not self.stop_event.is_set():
                    chunk = process.stdout.read(frame_bytes)
                    if len(chunk) != frame_bytes:
                        if not self.stop_event.is_set():
                            self._warn_color(
                                "Gemini335 color capture stream ended before a complete frame"
                            )
                        break
                    raw = np.frombuffer(chunk, dtype=np.uint8).reshape(
                        self.color_height, self.color_width, bytes_per_pixel
                    )
                    frame = cv2.cvtColor(raw, code)
                    self._publish_color_array(frame)
            except OSError as exc:
                self._warn_color(f"Gemini335 color capture IO error: {exc}")
            finally:
                with self.color_process_lock:
                    if self.color_process is process:
                        self.color_process = None
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                if process is not None and process.returncode not in (None, 0) and not self.stop_event.is_set():
                    self._warn_color(f"Gemini335 color capture exited with {process.returncode}")
            self._sleep_color_period(start, period)
            if process is not None and process.returncode not in (None, 0, -15):
                self.stop_event.wait(backoff)
                backoff = min(backoff * 2, 3.0)
            else:
                backoff = 0.2 if backoff == 0.0 else max(backoff * 0.5, 0.1)
        self.get_logger().info("Gemini335 color v4l2 capture worker stopped")

    def _sleep_color_period(self, start: float, period: float) -> None:
        remaining = period - (time.monotonic() - start)
        if remaining > 0.0:
            self.stop_event.wait(remaining)

    def _depth_capture_worker(self) -> None:
        if self.depth_raw_path is None:
            return
        frame_pixels = self.depth_width * self.depth_height
        pixelformat = (str(self.get_parameter("depth_fourcc").value) + "    ")[:4]
        period = 1.0 / max(self.depth_fps, 0.1)
        command = [
            "v4l2-ctl",
            "-d",
            self.depth_device,
            f"--set-fmt-video=width={self.depth_width},height={self.depth_height},pixelformat={pixelformat}",
            f"--set-parm={max(self.depth_fps, 1.0):g}",
            "--stream-mmap=4",
            f"--stream-count={self.depth_batch_frames}",
            f"--stream-to={self.depth_raw_path}",
        ]
        while not self.stop_event.is_set():
            start = time.monotonic()
            try:
                try:
                    os.unlink(self.depth_raw_path)
                except OSError:
                    pass
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
                with self.depth_process_lock:
                    if self.stop_event.is_set():
                        return
                    self.depth_process = process
                try:
                    _, stderr = process.communicate(timeout=self.depth_capture_timeout_sec)
                except subprocess.TimeoutExpired:
                    process.kill()
                    _, stderr = process.communicate()
                    if self.stop_event.is_set():
                        return
                    self._warn_depth(
                        "Gemini335 depth capture timeout; retrying short batch capture"
                    )
                    continue
                finally:
                    with self.depth_process_lock:
                        if self.depth_process is process:
                            self.depth_process = None
                if self.stop_event.is_set():
                    return
                data = np.fromfile(self.depth_raw_path, dtype=np.uint16)
                if process.returncode != 0 and data.size < frame_pixels:
                    detail = stderr.decode(errors="ignore").strip()
                    suffix = f": {detail}" if detail else ""
                    self._warn_depth(f"Gemini335 depth capture failed{suffix}")
                    self._sleep_depth_period(start, period)
                    continue
                if process.returncode != 0:
                    detail = stderr.decode(errors="ignore").strip()
                    suffix = f": {detail}" if detail else ""
                    self._warn_depth(
                        "Gemini335 depth capture returned non-zero but produced frames; "
                        f"publishing latest frame{suffix}"
                    )
            except OSError as exc:
                self._warn_depth(f"Gemini335 depth capture IO error: {exc}")
                self._sleep_depth_period(start, period)
                continue

            frames = data.size // frame_pixels
            if frames < 1:
                self._warn_depth("Gemini335 depth capture produced no complete Z16 frame")
                self._sleep_depth_period(start, period)
                continue
            offset = (frames - 1) * frame_pixels
            depth = data[offset : offset + frame_pixels].reshape(self.depth_height, self.depth_width)
            stamp = self.get_clock().now()
            depth_copy = depth.copy()
            with self.depth_lock:
                self.latest_depth = DepthFrame(stamp=stamp, data=depth_copy)
            if self.publish_depth:
                self._publish_depth(depth_copy, stamp)
            if self.publish_depth_color:
                self._publish_depth_colormap(depth_copy, stamp)
            if self.publish_pointcloud:
                now = time.monotonic()
                if now - self.last_pointcloud_stamp >= self.pointcloud_period:
                    self.last_pointcloud_stamp = now
                    self._publish_pointcloud(depth_copy, stamp)
            self._sleep_depth_period(start, period)

    def _sleep_depth_period(self, start: float, period: float) -> None:
        remaining = period - (time.monotonic() - start)
        if remaining > 0.0:
            self.stop_event.wait(remaining)

    def _warn_depth(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_depth_warning >= 2.0:
            self.last_depth_warning = now
            self.get_logger().warning(message)

    def _warn_color(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_color_warning >= 2.0:
            self.last_color_warning = now
            self.get_logger().warning(message)

    def _header(self, stamp: rclpy.time.Time, frame_id: str) -> Header:
        header = Header()
        header.stamp = stamp.to_msg()
        header.frame_id = frame_id
        return header

    def _can_publish(self) -> bool:
        return not self.stop_event.is_set() and rclpy.ok()

    def _publish_safe(self, publisher, message) -> bool:
        if not self._can_publish():
            return False
        try:
            publisher.publish(message)
        except Exception:
            if self.stop_event.is_set() or not rclpy.ok():
                return False
            raise
        return True

    def _publish_color_frame(self) -> None:
        if self.color_capture is None or not self._can_publish():
            return
        try:
            ok, frame = self.color_capture.read()
        except cv2.error as exc:
            now = time.monotonic()
            if now - self.last_color_warning >= 2.0:
                self.last_color_warning = now
                self.get_logger().warning(f"Gemini335 color frame read error: {exc}")
            return
        if not ok or frame is None:
            now = time.monotonic()
            if now - self.last_color_warning >= 2.0:
                self.last_color_warning = now
                self.get_logger().warning("Gemini335 color frame read failed")
            return
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        self._publish_color_array(frame)

    def _publish_color_array(self, frame: np.ndarray) -> None:
        if not self._can_publish():
            return
        stamp = self.get_clock().now()
        header = self._header(stamp, self.color_frame_id)
        if not self._publish_safe(self.color_pub, _image_from_array(frame, header, "bgr8")):
            return
        self.color_info.header = header
        self._publish_safe(self.color_info_pub, self.color_info)
        self.color_frame_count += 1
        if self.color_frame_count == 1:
            self.get_logger().info(
                f"first color frame published: {frame.shape[1]}x{frame.shape[0]}"
            )

    def _publish_depth(self, depth: np.ndarray, stamp: rclpy.time.Time) -> None:
        header = self._header(stamp, self.depth_frame_id)
        if not self._publish_safe(self.depth_pub, _image_from_array(depth, header, "16UC1")):
            return
        self.depth_info.header = header
        self._publish_safe(self.depth_info_pub, self.depth_info)

    def _publish_depth_colormap(self, depth: np.ndarray, stamp: rclpy.time.Time) -> None:
        valid = depth > 0
        if np.any(valid):
            values = depth[valid]
            lo = float(np.percentile(values, 2.0))
            hi = float(np.percentile(values, 98.0))
            if hi <= lo:
                hi = lo + 1.0
            normalized = np.clip(
                (depth.astype(np.float32) - lo) * 255.0 / (hi - lo), 0.0, 255.0
            ).astype(np.uint8)
            normalized[~valid] = 0
        else:
            normalized = np.zeros_like(depth, dtype=np.uint8)
        color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        header = self._header(stamp, self.depth_frame_id)
        self._publish_safe(self.depth_color_pub, _image_from_array(color, header, "bgr8"))

    def _publish_pointcloud(self, depth: np.ndarray, stamp: rclpy.time.Time) -> None:
        if self.points_pub is None:
            return
        points = self._depth_to_points(depth)
        header = self._header(stamp, self.depth_frame_id)
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self._publish_safe(self.points_pub, cloud)

    def _depth_to_points(self, depth: np.ndarray) -> Iterable[tuple[float, float, float]]:
        h, w = depth.shape[:2]
        step = self.pointcloud_decimation
        fx = self.depth_info.k[0]
        fy = self.depth_info.k[4]
        cx = self.depth_info.k[2]
        cy = self.depth_info.k[5]
        sampled = depth[0:h:step, 0:w:step].astype(np.float32)
        vv, uu = np.mgrid[0:h:step, 0:w:step]
        z_all = sampled * self.depth_scale
        mask = (z_all >= self.pointcloud_min_depth_m) & (z_all <= self.pointcloud_max_depth_m)
        z = z_all[mask]
        u = uu[mask].astype(np.float32)
        v = vv[mask].astype(np.float32)
        pixel_x = cx - u if self.projection_flip_x else u - cx
        pixel_y = cy - v if self.projection_flip_y else v - cy
        x = pixel_x * z / fx
        y = pixel_y * z / fy
        return zip(x.astype(float), y.astype(float), z.astype(float))


def _run_dry_run_check() -> None:
    """Headless check of factory-intrinsics load/apply logic. Never touches hardware."""
    import tempfile

    sample = {
        "format": "arachne_gemini335_factory_calibration_v1",
        "streams": {
            "color": {
                "image_size": [640, 480],
                "intrinsic": {"fx": 601.5, "fy": 602.5, "cx": 320.25, "cy": 240.75},
                "distortion": [0.01, -0.02, 0.03, 0.0, 0.0, 0.0, 0.001, -0.002],
                "distortion_model": "modified_brown_conrady",
                "is_mirrored": False,
            },
            "depth": {
                "image_size": [640, 480],
                "intrinsic": {"fx": 611.0, "fy": 612.0, "cx": 319.0, "cy": 239.0},
                "distortion": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.01, 0.02],
                "distortion_model": "modified_brown_conrady",
                "is_mirrored": False,
            },
        },
    }
    with tempfile.TemporaryDirectory() as td:
        valid = os.path.join(td, "factory_calibration.json")
        with open(valid, "w", encoding="utf-8") as fh:
            json.dump(sample, fh)
        loaded = _load_factory_intrinsics(valid)
        assert loaded is not None and "color" in loaded and "depth" in loaded, "valid json must load"
        msg = _apply_factory_intrinsics(
            _camera_info(640, 480, "camera_color_optical_frame", 0.0, 0.0), loaded["color"]
        )
        assert abs(msg.k[0] - 601.5) < 1e-9 and abs(msg.k[4] - 602.5) < 1e-9, "K fx/fy from factory"
        assert abs(msg.k[2] - 320.25) < 1e-9 and abs(msg.k[5] - 240.75) < 1e-9, "K cx/cy from factory"
        assert list(msg.d) == [0.01, -0.02, 0.001, -0.002, 0.03], "plumb_bob [k1,k2,p1,p2,k3] mapping"
        assert loaded["color"]["is_mirrored"] is False and loaded["depth"]["is_mirrored"] is False, (
            "is_mirrored parsed from json"
        )
        # image_size mismatch must be rejected by the node-level guard (mirrors __init__ logic):
        assert (loaded["color"]["image_size"][0], loaded["color"]["image_size"][1]) == (640, 480), (
            "image_size parsed from json"
        )
        assert _load_factory_intrinsics(os.path.join(td, "missing.json")) is None, "missing -> None"
        bad = os.path.join(td, "bad.json")
        with open(bad, "w", encoding="utf-8") as fh:
            fh.write("{not json")
        assert _load_factory_intrinsics(bad) is None, "invalid json -> None"
        partial = os.path.join(td, "partial.json")
        with open(partial, "w", encoding="utf-8") as fh:
            json.dump(
                {
                    "streams": {
                        "color": sample["streams"]["color"],
                        "depth": {"image_size": [640, 480], "intrinsic": {"fx": 1.0}},
                    }
                },
                fh,
            )
        partial_loaded = _load_factory_intrinsics(partial)
        assert partial_loaded is not None and "color" in partial_loaded and "depth" not in partial_loaded, (
            "bad depth entry drops only depth"
        )
        assert _load_factory_intrinsics("") is None, "empty path -> None"
    print("gemini335_v4l2_node dry-run check ok")


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Gemini335 V4L2 driver node")
    parser.add_argument("--dry-run-check", action="store_true", help="headless check, no hardware")
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.dry_run_check:
        _run_dry_run_check()
        return
    rclpy.init(args=ros_args)
    node = Gemini335V4L2Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
