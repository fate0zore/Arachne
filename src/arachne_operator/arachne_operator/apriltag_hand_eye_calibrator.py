from __future__ import annotations  # 启用 PEP 585/604 后向兼容的类型注解语法

import json  # 用于将标定结果序列化为 JSON 文件
import math  # 用于三角函数、平方根等数学运算
from dataclasses import dataclass  # 数据类装饰器，简化纯数据容器
from datetime import datetime  # 生成时间戳，用于输出文件命名和采样记录
from pathlib import Path  # 跨平台路径处理，替代 os.path
from typing import Any  # 类型注解：当类型确实无法确定时使用

import cv2  # OpenCV，负责图像处理、PnP 求解、手眼标定核心算法
import numpy as np  # 数值计算库，所有变换矩阵和点云数据均以 ndarray 表示
import rclpy  # ROS2 Python 客户端库
from ament_index_python.packages import get_package_share_directory  # 获取 ROS 包共享目录路径
from rclpy.duration import Duration  # ROS2 时间跨度类型，用于 TF 查询超时
from rclpy.node import Node  # ROS2 节点基类
from rclpy.time import Time  # ROS2 时间类型，用于 TF 查找
from sensor_msgs.msg import CameraInfo, Image  # 相机内参和图像消息类型
from std_srvs.srv import Trigger  # 标准无参数触发服务（capture/solve/reset）
import tf2_ros  # ROS2 TF 树监听与缓冲区，负责获取机器人运动链变换

try:
    # pupil-apriltags 是高性能 AprilTag 检测器（tagStandard41h12 等新家族必需）
    from pupil_apriltags import Detector as PupilAprilTagDetector
except Exception:  # pragma: no cover - optional runtime dependency
    # 如果未安装，置 None，运行时选择 tag_family 时会主动报错提示安装
    PupilAprilTagDetector = None


def _rotation_matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """将 3x3 旋转矩阵转为单位四元数 (qx, qy, qz, qw)。"""
    m = matrix
    trace = float(np.trace(m))  # 矩阵迹（对角线之和），决定选用哪个分支避免数值不稳定
    if trace > 0.0:
        # 迹 >0：所有对角元之和为正，最稳定的分支
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        # m[0,0] 最大：从 x 分量计算，避免除以接近零的值
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        # m[1,1] 最大：从 y 分量计算
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        # m[2,2] 最大：从 z 分量计算
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return (qx, qy, qz, qw)


def _quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:
    """将单位四元数转为 3x3 旋转矩阵。先归一化防止累积误差。"""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float64)  # 零四元数退化情况，返回单位阵
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm  # 归一化
    return np.array(
        [
            # 第 0 行：旋转矩阵的 (0,0) (0,1) (0,2)
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            # 第 1 行
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            # 第 2 行
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_to_rpy(matrix: np.ndarray) -> tuple[float, float, float]:
    """将 3x3 旋转矩阵转为 RPY (roll, pitch, yaw) 欧拉角。"""
    # 计算 pitch 角的正弦值，用于判断是否接近万向锁
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    if sy > 1e-9:
        # 一般情况：非万向锁
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        # 万向锁情况：pitch 接近 ±90°，roll 和 yaw 合并为一个自由度
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """构建 4x4 齐次变换矩阵 [R | t; 0 0 0 1]。"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation  # 左上 3x3 为旋转部分
    transform[:3, 3] = translation.reshape(3)  # 右上 3x1 为平移部分
    return transform


def _invert_transform(transform: np.ndarray) -> np.ndarray:
    """求 4x4 齐次变换矩阵的逆：R⁻¹ = Rᵀ, t' = -Rᵀ·t。绕过了通用 4x4 求逆的昂贵计算。"""
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T  # 旋转部分转置即逆
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]  # 平移部分取反后旋转
    return inverse


def _transform_to_dict(transform: np.ndarray) -> dict[str, Any]:
    """将 4x4 变换矩阵转为可 JSON 序列化的字典，同时导出 xyz/rpy/四元数/矩阵。"""
    xyz = transform[:3, 3]  # 平移向量
    rpy = _rotation_matrix_to_rpy(transform[:3, :3])  # 欧拉角
    quat = _rotation_matrix_to_quaternion(transform[:3, :3])  # 四元数
    return {
        "xyz": [float(v) for v in xyz],
        "rpy": [float(v) for v in rpy],
        "quaternion_xyzw": [float(v) for v in quat],
        "matrix": transform.tolist(),  # 完整 4x4 矩阵（最精确的表示）
    }


def _ros_transform_to_matrix(msg: Any) -> np.ndarray:
    """将 ROS TransformStamped 消息转为 4x4 齐次变换矩阵。"""
    translation = np.array(
        [
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ],
        dtype=np.float64,
    )
    rotation = _quaternion_to_rotation_matrix(
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w,
    )
    return _make_transform(rotation, translation)


@dataclass
class Detection:
    """一次 AprilTag 检测结果：目标坐标系相对于相机的 4x4 变换。"""
    tag_id: int  # 主标签 ID（仅检测单个标签时使用）
    tag_ids: list[int]  # 检测到的所有标签 ID 列表（板级检测时使用）
    target_to_camera: np.ndarray  # 4x4 齐次变换：目标坐标系 → 相机坐标系
    reprojection_error_px: float  # 重投影误差（像素），衡量 PnP 解算质量


@dataclass
class Sample:
    """一个手眼标定的采样数据对：机械臂位姿 + 视觉检测位姿。"""
    stamp: str  # ISO 时间戳，用于结果追溯
    tag_id: int  # 主标签 ID
    tag_ids: list[int]  # 所有可见标签 ID
    base_to_gripper: np.ndarray  # 4x4 变换：机器人基座 → 夹爪/工具（来自 TF）
    target_to_camera: np.ndarray  # 4x4 变换：标定目标 → 相机（来自标签检测 PnP）
    reprojection_error_px: float  # 该采样的重投影误差


class AprilTagHandEyeCalibrator(Node):
    """ROS2 节点：通过 AprilTag 标定板进行相机-夹爪手眼标定。
    
    订阅相机图像和内参，提供三个 Trigger 服务：
    - ~/capture：采集当前检测 + 机械臂位姿为一个样本
    - ~/solve：基于已采集样本运行手眼标定（尝试多种 OpenCV 方法）
    - ~/reset：清空样本
    """

    def __init__(self) -> None:
        super().__init__("arachne_apriltag_hand_eye_calibrator")
        # ---- 声明所有 ROS2 参数（带默认值），允许在 launch 中覆盖 ----
        self.declare_parameter("image_topic", "/camera/color/image_raw")  # 彩色图像话题
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")  # 相机内参话题
        self.declare_parameter("base_frame", "base_link")  # 机器人基座坐标系
        self.declare_parameter("gripper_frame", "tool0")  # 夹爪（工具）坐标系
        self.declare_parameter(
            "board_image_path", "/home/jetson/zhaoyang/arachne_floor_apriltag_board_a3.png"
        )  # 标定板参考图像路径（用于模板匹配 fallback）
        self.declare_parameter("board_width_m", 0.420)  # 标定板物理宽度（米）
        self.declare_parameter("board_height_m", 0.297)  # 标定板物理高度（米）
        self.declare_parameter("min_board_matches", 24)  # 模板匹配最少内点数
        self.declare_parameter("tag_family", "tagStandard41h12")  # AprilTag 家族（空字符串则用 OpenCV）
        self.declare_parameter("tag_size_m", 0.070)  # 单个标签边长（米）
        self.declare_parameter("tag_pitch_m", 0.100)  # 标签中心间距（米）
        self.declare_parameter("tag_id", -1)  # 指定标签 ID（-1 表示使用所有标签/板级检测）
        self.declare_parameter("dictionary", "DICT_APRILTAG_36h11")  # OpenCV 字典名称（仅 tag_family 为空时生效）
        self.declare_parameter("enable_board_template_fallback", False)  # 是否启用 ORB 模板匹配 fallback
        self.declare_parameter("min_target_distance_m", 0.05)  # 有效检测最小距离
        self.declare_parameter("max_target_distance_m", 3.0)  # 有效检测最大距离
        self.declare_parameter("max_reprojection_error_px", 8.0)  # 有效检测最大重投影误差
        self.declare_parameter("output_dir", "log/calibration/hand_eye")  # 结果输出目录

        # ---- 读取参数到实例变量 ----
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.gripper_frame = str(self.get_parameter("gripper_frame").value)
        self.board_image_path = str(self.get_parameter("board_image_path").value)
        self.board_width_m = float(self.get_parameter("board_width_m").value)
        self.board_height_m = float(self.get_parameter("board_height_m").value)
        self.min_board_matches = int(self.get_parameter("min_board_matches").value)
        self.tag_family = str(self.get_parameter("tag_family").value).strip()
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_pitch_m = float(self.get_parameter("tag_pitch_m").value)
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.enable_board_template_fallback = bool(
            self.get_parameter("enable_board_template_fallback").value
        )
        self.min_target_distance_m = float(self.get_parameter("min_target_distance_m").value)
        self.max_target_distance_m = float(self.get_parameter("max_target_distance_m").value)
        self.max_reprojection_error_px = float(
            self.get_parameter("max_reprojection_error_px").value
        )
        self.output_dir = self._resolve_output_dir(str(self.get_parameter("output_dir").value))

        # ---- 运行时状态变量 ----
        self.camera_matrix: np.ndarray | None = None  # 相机内参矩阵 3x3
        self.dist_coeffs: np.ndarray | None = None  # 畸变系数
        self.latest_image: Image | None = None  # 最新一帧图像消息
        self.latest_detection: Detection | None = None  # 最新检测结果
        self.samples: list[Sample] = []  # 已采集的标定样本列表

        # ---- 初始化 AprilTag 或 OpenCV AprilTag 检测器 ----
        self.apriltag_detector: Any = None
        if self.tag_family:
            # 使用 pupil-apriltags 库（支持 tagStandard41h12 等新家族）
            if PupilAprilTagDetector is None:
                raise RuntimeError(
                    "tagStandard41h12 requires pupil-apriltags. "
                    "Install it with: python3 -m pip install --user pupil-apriltags"
                )
            self.apriltag_detector = PupilAprilTagDetector(
                families=self.tag_family,
                nthreads=4,  # 用 4 个线程加速检测
                quad_decimate=1.0,  # 不下采样（保持原始分辨率）
                quad_sigma=0.0,  # 不做高斯模糊预处理
                refine_edges=True,  # 边缘细化和亚像素定位
                decode_sharpening=0.25,  # 解码锐化强度
            )
            self.dictionary = None
            self.detector = None
        else:
            # 回退到 OpenCV 内置的 AprilTag 字典（如 DICT_APRILTAG_36h11）
            self.dictionary = self._make_dictionary(str(self.get_parameter("dictionary").value))
            self.detector = self._make_detector(self.dictionary)

        # ---- 初始化模板匹配（ORB 特征） ----
        self.orb = cv2.ORB_create(nfeatures=2500)  # ORB 特征提取器，最多 2500 个特征点
        self.board_gray: np.ndarray | None = None  # 标定板参考灰度图
        self.board_keypoints: Any = None  # 参考图的关键点
        self.board_descriptors: np.ndarray | None = None  # 参考图的描述子
        self.board_tag_object_corners: dict[int, np.ndarray] = {}  # 板中各标签的物理 3D 角点
        self._load_board_template()  # 加载标定板参考图像

        # ---- TF 监听 ----
        self.tf_buffer = tf2_ros.Buffer()  # TF 缓冲区
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # 自动接收 TF

        # ---- 订阅与服务的创建 ----
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.create_service(Trigger, "~/capture", self._on_capture)  # 采集样本
        self.create_service(Trigger, "~/solve", self._on_solve)  # 求解标定
        self.create_service(Trigger, "~/reset", self._on_reset)  # 重置样本

        # ---- 启动日志 ----
        self.get_logger().info(
            "AprilTag hand-eye calibrator ready: "
            f"image={self.image_topic} camera_info={self.camera_info_topic} "
            f"base={self.base_frame} gripper={self.gripper_frame} "
            f"tag_family={self.tag_family or 'opencv'} "
            f"tag_size={self.tag_size_m:.3f}m tag_pitch={self.tag_pitch_m:.3f}m "
            f"tag_id={self.tag_id} "
            f"dictionary={self.get_parameter('dictionary').value} "
            f"template_fallback={self.enable_board_template_fallback} "
            f"board={self.board_image_path}"
        )

    def _load_board_template(self) -> None:
        """加载标定板参考图像，提取 ORB 特征和 AprilTag 参考角点。"""
        path = Path(self.board_image_path).expanduser()
        if not path.exists():
            self.get_logger().warn(f"board image not found, template fallback disabled: {path}")
            return
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().warn(f"failed to read board image, template fallback disabled: {path}")
            return
        self.board_gray = image  # 保存参考灰度图
        self._load_apriltag_board_reference(image)  # 从参考图中提取板标签物理坐标
        keypoints, descriptors = self.orb.detectAndCompute(image, None)  # 提取 ORB 特征
        self.board_keypoints = keypoints
        self.board_descriptors = descriptors
        count = 0 if keypoints is None else len(keypoints)
        self.get_logger().info(
            f"loaded board template {path} size={image.shape[1]}x{image.shape[0]} "
            f"features={count} apriltag_ids={sorted(self.board_tag_object_corners)} "
            f"physical={self.board_width_m:.3f}x{self.board_height_m:.3f}m"
        )

    def _load_apriltag_board_reference(self, image: np.ndarray) -> None:
        """从标定板参考图像中检测 AprilTag，计算各标签的物理 3D 角点（板中心为原点）。

        物理角点直接由参考图像素按 board_width_m/board_height_m 实测映射，
        不依赖 tag_size_m/tag_pitch_m 假设，避免打印尺寸与参数不一致导致的整板 PnP 系统性错位。
        """
        if self.apriltag_detector is None:
            return  # 只有 pupil-apriltags 检测器才支持板级参考
        detections = list(self.apriltag_detector.detect(image, estimate_tag_pose=False))
        if not detections:
            self.get_logger().warn(
                f"no {self.tag_family} tags detected in board reference; board PnP disabled"
            )
            return
        height, width = image.shape[:2]
        scale_x = self.board_width_m / width
        scale_y = self.board_height_m / height
        loaded = 0
        for det in detections:
            corners_px = np.asarray(det.corners, dtype=np.float64)  # pupil 顺序（逆时针，图像坐标）
            physical = np.zeros((4, 3), dtype=np.float64)
            physical[:, 0] = (corners_px[:, 0] - width * 0.5) * scale_x
            physical[:, 1] = (height * 0.5 - corners_px[:, 1]) * scale_y
            self.board_tag_object_corners[int(det.tag_id)] = physical
            loaded += 1
        self.get_logger().info(
            f"loaded {loaded} {self.tag_family} board tags from reference "
            f"(physical corners measured from pixels)"
        )

    def _resolve_output_dir(self, value: str) -> Path:
        """解析输出目录路径：相对路径基于工作区根目录（ROS 包上级三级）生成。"""
        path = Path(value).expanduser()
        if not path.is_absolute():
            try:
                # 尝试从 ROS 包路径推算工作区根目录
                root = Path(get_package_share_directory("arachne_operator")).parents[3]
            except Exception:
                root = Path.cwd()  # 回退到当前工作目录
            path = root / path
        path.mkdir(parents=True, exist_ok=True)  # 自动创建目录
        return path

    def _make_dictionary(self, name: str) -> Any:
        """根据字符串名称获取 OpenCV ArUco/AprilTag 字典。"""
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV was built without cv2.aruco; install opencv-contrib-python.")
        if not hasattr(cv2.aruco, name):
            raise RuntimeError(f"Unknown ArUco/AprilTag dictionary: {name}")
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _make_detector(self, dictionary: Any) -> Any:
        """兼容 OpenCV 4.x 和旧版 API 创建 ArUco 检测器。"""
        if hasattr(cv2.aruco, "DetectorParameters"):
            parameters = cv2.aruco.DetectorParameters()  # 新版 API
        else:
            parameters = cv2.aruco.DetectorParameters_create()  # 旧版 API
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(dictionary, parameters)  # 新版
        return (dictionary, parameters)  # 旧版：返回元组供 detectMarkers 使用

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """接收相机内参消息，提取内参矩阵和畸变系数。"""
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)  # 内参矩阵 K
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def _on_image(self, msg: Image) -> None:
        """接收图像消息，如果有内参则进行 AprilTag 检测。"""
        self.latest_image = msg
        if self.camera_matrix is None:
            return  # 还没有内参，无法做 PnP
        try:
            image = self._image_to_bgr(msg)
            detection = self._detect_tag(image)
        except Exception as exc:
            self.latest_detection = None
            self.get_logger().warn(f"AprilTag detection skipped: {exc}", throttle_duration_sec=2.0)
            return
        self.latest_detection = detection  # 更新最新检测结果

    def _image_to_bgr(self, msg: Image) -> np.ndarray:
        """将 ROS Image 消息转为 OpenCV BGR 格式（三通道 uint8）。"""
        height = int(msg.height)
        width = int(msg.width)
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        step = int(msg.step)
        if encoding in ("bgr8", "rgb8"):
            # BGR/RGB 三通道格式：从 step 中提取有效宽度
            image = data.reshape(height, step)[:, : width * 3].reshape(height, width, 3)
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image
        if encoding in ("bgra8", "rgba8"):
            # BGRA/RGBA 四通道格式：去掉 alpha 通道
            image = data.reshape(height, step)[:, : width * 4].reshape(height, width, 4)
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(image, code)
        if encoding in ("mono8", "8uc1"):
            # 单通道灰度：转为三通道 BGR
            image = data.reshape(height, step)[:, :width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    def _detect_tag(self, image: np.ndarray) -> Detection | None:
        """对一帧图像进行 AprilTag 检测，返回 Detection 或 None。
        
        检测优先级：
        1. pupil-apriltags 板级检测（多个标签联合 PnP）
        2. 单标签检测（指定 tag_id 或单标签模式）
        3. ORB 模板匹配 fallback（当 enable_board_template_fallback 启用时）
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.apriltag_detector is not None:
            # 先尝试 pupil-apriltags 板级检测
            detection, diag = self._detect_apriltag_board(gray)
            if detection is not None:
                return detection
            if diag:
                self.get_logger().info(diag, throttle_duration_sec=2.0)
            if self.tag_id >= 0 or not self.enable_board_template_fallback:
                return None  # 指定了 tag_id 但没检测到，不继续尝试 fallback

        # 使用 OpenCV ArUco 检测单个标签
        if hasattr(cv2.aruco, "ArucoDetector"):
            corners, ids, _ = self.detector.detectMarkers(gray)  # 新版 API
        else:
            dictionary, parameters = self.detector
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)  # 旧版 API
        if ids is None or len(ids) == 0:
            if self.tag_id >= 0 or not self.enable_board_template_fallback:
                return None  # 未启用 fallback 或无指定标签
            return self._detect_board_template(gray)  # 尝试 ORB 模板匹配
        flat_ids = ids.reshape(-1)
        selected = 0
        if self.tag_id >= 0:
            # 从检测到的标签中过滤出指定 ID
            matches = np.where(flat_ids == self.tag_id)[0]
            if len(matches) == 0:
                return None
            selected = int(matches[0])  # 取第一个匹配
        tag_id = int(flat_ids[selected])
        image_points = corners[selected].reshape(4, 2).astype(np.float64)  # 图像上的四个角点
        object_points = self._tag_object_points()  # 标签物理 3D 角点（以标签中心为原点）
        # 使用 IPPE 算法（针对平面物体的高效 PnP，仅需 4 个点）
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)  # 旋转向量 → 旋转矩阵
        # 计算重投影误差
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - image_points, axis=1)))
        distance = float(np.linalg.norm(tvec.reshape(3)))  # 相机到目标的距离
        if not self._valid_detection_geometry(distance, error):
            return None  # 超出有效范围
        return Detection(
            tag_id=tag_id,
            tag_ids=[tag_id],
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        )

    def _detect_apriltag_board(self, gray: np.ndarray) -> tuple[Detection | None, str | None]:
        """使用 pupil-apriltags 进行板级检测：联合多个标签解算位姿，精度更高。

        返回 (检测结果, 失败诊断)：成功时诊断为 None，失败时诊断说明具体原因。
        """
        if self.apriltag_detector is None or not self.board_tag_object_corners:
            return None, "board PnP disabled: no tags loaded from board reference"
        detections = list(self.apriltag_detector.detect(gray, estimate_tag_pose=False))
        object_points: list[np.ndarray] = []  # 所有匹配标签的物理 3D 点（板坐标系）
        image_points: list[np.ndarray] = []  # 对应的图像 2D 点
        used_ids = []
        for det in detections:
            tag_id = int(det.tag_id)
            if self.tag_id >= 0 and tag_id != self.tag_id:
                continue  # 过滤指定标签 ID
            corners = self.board_tag_object_corners.get(tag_id)
            if corners is None:
                continue  # 该标签不在参考板中（可能是背景标签）
            object_points.extend(corners)  # 4 个物理角点
            # 物理角点与图像角点均为 pupil-apriltags 同一 corners 顺序（逆时针从图像左下起），
            # 直接按序对应即可，无需重排
            image_points.extend(np.asarray(det.corners, dtype=np.float64))  # 4 个图像角点
            used_ids.append(tag_id)
        if len(object_points) < 4:
            if not detections:
                return None, "detected 0 AprilTags in frame"
            return None, (
                f"detected {len(detections)} tag(s) but only {len(used_ids)} in board "
                f"(board ids {sorted(self.board_tag_object_corners)})"
            )
        object_array = np.asarray(object_points, dtype=np.float64)
        image_array = np.asarray(image_points, dtype=np.float64)
        if len(object_array) == 4:
            # 只有一个标签，使用 IPPE 快速求解
            ok, rvec, tvec = cv2.solvePnP(
                object_array,
                image_array,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            inlier_object = object_array
            inlier_image = image_array
        else:
            # 多个标签，使用 RANSAC 剔除误匹配
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_array,
                image_array,
                self.camera_matrix,
                self.dist_coeffs,
                iterationsCount=100,
                reprojectionError=4.0,
                confidence=0.995,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok or inliers is None or len(inliers) < 4:
                return None, f"PnP RANSAC failed with {len(used_ids)} board tags"
            inlier_object = object_array[inliers.reshape(-1)]
            inlier_image = image_array[inliers.reshape(-1)]
            # 用内点重新精细求解
            ok, rvec, tvec = cv2.solvePnP(
                inlier_object,
                inlier_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                useExtrinsicGuess=True,  # 用 RANSAC 结果作为初值加速收敛
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if not ok:
            return None, f"PnP refinement failed with {len(used_ids)} board tags"
        rotation, _ = cv2.Rodrigues(rvec)
        # 重投影误差计算
        projected, _ = cv2.projectPoints(
            inlier_object, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - inlier_image, axis=1)))
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if not self._valid_detection_geometry(distance, error):
            return None, (
                f"geometry rejected: {len(used_ids)} tags dist={distance:.2f}m "
                f"reproj={error:.2f}px (limits {self.min_target_distance_m}-"
                f"{self.max_target_distance_m}m / {self.max_reprojection_error_px}px)"
            )
        tag_label = used_ids[0] if len(used_ids) == 1 else -1  # 多标签时主 ID 设为 -1
        return Detection(
            tag_id=tag_label,
            tag_ids=sorted(set(used_ids)),  # 去重并排序的标签 ID 列表
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        ), None

    def _detect_board_template(self, gray: np.ndarray) -> Detection | None:
        """ORB 特征点模板匹配 fallback：当 AprilTag 检测完全失败时使用。
        
        通过 ORB 特征匹配参考图和当前图，再用 PnP 解算位姿。
        精度低于 AprilTag，仅作为应急手段。
        """
        if self.board_gray is None or self.board_descriptors is None:
            return None
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        if descriptors is None or keypoints is None or len(keypoints) < self.min_board_matches:
            return None  # 特征点太少
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)  # 暴力匹配器，汉明距离
        pairs = matcher.knnMatch(self.board_descriptors, descriptors, k=2)  # 每个参考点找两个最近邻
        good = []
        for pair in pairs:
            if len(pair) != 2:
                continue
            first, second = pair
            # Lowe's ratio test：最优距离 < 0.72 * 次优距离 时认为是好匹配
            if first.distance < 0.72 * second.distance:
                good.append(first)
        if len(good) < self.min_board_matches:
            return None  # 足够的好匹配数
        object_points = []  # 物理 3D 点（板坐标系）
        image_points = []  # 对应图像 2D 点
        board_h, board_w = self.board_gray.shape[:2]
        for match in good:
            u, v = self.board_keypoints[match.queryIdx].pt  # 参考图中的像素坐标
            # 将参考图像素坐标映射到物理板坐标（板中心为原点）
            x = (u - board_w * 0.5) * self.board_width_m / board_w
            y = (v - board_h * 0.5) * self.board_height_m / board_h
            object_points.append((x, y, 0.0))
            image_points.append(keypoints[match.trainIdx].pt)  # 当前图中对应点
        object_array = np.asarray(object_points, dtype=np.float64)
        image_array = np.asarray(image_points, dtype=np.float64)
        # 使用 RANSAC PnP 从 2D-3D 匹配中解算位姿
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_array,
            image_array,
            self.camera_matrix,
            self.dist_coeffs,
            iterationsCount=100,
            reprojectionError=5.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or inliers is None or len(inliers) < self.min_board_matches:
            return None
        inlier_object = object_array[inliers.reshape(-1)]
        inlier_image = image_array[inliers.reshape(-1)]
        # 用内点精细求解
        ok, rvec, tvec = cv2.solvePnP(
            inlier_object,
            inlier_image,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        # 重投影误差
        projected, _ = cv2.projectPoints(
            inlier_object, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - inlier_image, axis=1)))
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if not self._valid_detection_geometry(distance, error):
            return None
        return Detection(
            tag_id=-1,  # 模板匹配没有标签 ID
            tag_ids=[],
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        )

    def _valid_detection_geometry(self, distance_m: float, reprojection_error_px: float) -> bool:
        """检查检测结果的几何有效性：距离范围和重投影误差是否在允许范围内。"""
        if not np.isfinite(distance_m) or not np.isfinite(reprojection_error_px):
            return False  # 数值不合法（无穷大或 NaN）
        if distance_m < self.min_target_distance_m or distance_m > self.max_target_distance_m:
            return False  # 超出有效距离范围
        if reprojection_error_px > self.max_reprojection_error_px:
            return False  # 重投影误差过大
        return True

    def _tag_object_points(self) -> np.ndarray:
        """返回单标签的四个物理角点（标签中心为原点，Z=0）。"""
        half = self.tag_size_m * 0.5
        return np.array(
            [
                [-half, half, 0.0],   # 左上
                [half, half, 0.0],    # 右上
                [half, -half, 0.0],   # 右下
                [-half, -half, 0.0],  # 左下
            ],
            dtype=np.float64,
        )

    def _lookup_base_to_gripper(self) -> np.ndarray:
        """从 TF 树获取机器人基座到夹爪的 4x4 变换。"""
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.gripper_frame,
            Time(),  # 当前时间
            timeout=Duration(seconds=0.5),  # 最多等待 0.5 秒
        )
        return _ros_transform_to_matrix(transform)

    def _on_capture(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """采集一个标定样本：当前 AprilTag 检测 + 当前机械臂位姿。"""
        detection = self.latest_detection
        if detection is None:
            response.success = False
            response.message = "no AprilTag detection yet"
            return response
        try:
            base_to_gripper = self._lookup_base_to_gripper()  # 获取当前机器人位姿
        except Exception as exc:
            response.success = False
            response.message = f"missing TF {self.base_frame}->{self.gripper_frame}: {exc}"
            return response
        stamp = datetime.now().isoformat(timespec="milliseconds")  # 精确到毫秒的时间戳
        self.samples.append(
            Sample(
                stamp=stamp,
                tag_id=detection.tag_id,
                tag_ids=detection.tag_ids,
                base_to_gripper=base_to_gripper,
                target_to_camera=detection.target_to_camera,
                reprojection_error_px=detection.reprojection_error_px,
            )
        )
        response.success = True
        if detection.tag_ids:
            tag_text = f"board_tags={detection.tag_ids}"
        else:
            tag_text = f"tag_id={detection.tag_id}"
        response.message = (
            f"captured sample {len(self.samples)} {tag_text} "
            f"reproj={detection.reprojection_error_px:.2f}px"
        )
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """清空所有已采集的样本。"""
        self.samples.clear()
        response.success = True
        response.message = "cleared hand-eye samples"
        return response

    def _on_solve(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """基于所有已采集样本运行手眼标定，保存结果到 JSON 文件。"""
        if len(self.samples) < 5:
            # 最少需要 5 个样本，否则标定结果不可靠
            response.success = False
            response.message = f"need at least 5 samples, have {len(self.samples)}"
            return response
        if not hasattr(cv2, "calibrateHandEye"):
            response.success = False
            response.message = "OpenCV calibrateHandEye is unavailable"
            return response

        # 将样本数据拆分为旋转和平移列表，供 OpenCV 手眼标定使用
        r_gripper_to_base = []  # 夹爪→基座的旋转矩阵列表
        t_gripper_to_base = []  # 夹爪→基座的平移向量列表
        r_target_to_camera = []  # 目标→相机的旋转矩阵列表
        t_target_to_camera = []  # 目标→相机的平移向量列表
        for sample in self.samples:
            r_gripper_to_base.append(sample.base_to_gripper[:3, :3])
            t_gripper_to_base.append(sample.base_to_gripper[:3, 3])
            r_target_to_camera.append(sample.target_to_camera[:3, :3])
            t_target_to_camera.append(sample.target_to_camera[:3, 3])

        # 使用多种手眼标定方法求解，选出最优
        candidates = self._solve_hand_eye_candidates(
            r_gripper_to_base,
            t_gripper_to_base,
            r_target_to_camera,
            t_target_to_camera,
        )
        valid_candidates = [candidate for candidate in candidates if candidate.get("valid")]
        if not valid_candidates:
            response.success = False
            response.message = "hand-eye solve failed: no valid candidate"
            return response
        # 选择一致性得分最低（最好）的候选结果
        selected = min(valid_candidates, key=lambda item: float(item["consistency_score"]))
        camera_to_gripper = np.asarray(selected["camera_to_gripper_matrix"], dtype=np.float64)
        gripper_to_camera = np.asarray(selected["gripper_to_camera_matrix"], dtype=np.float64)
        # 构建完整的标定结果字典
        result = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "base_frame": self.base_frame,
            "gripper_frame": self.gripper_frame,
            "camera_frame": self.latest_image.header.frame_id if self.latest_image else "",
            "board_image_path": self.board_image_path,
            "board_width_m": self.board_width_m,
            "board_height_m": self.board_height_m,
            "tag_size_m": self.tag_size_m,
            "tag_pitch_m": self.tag_pitch_m,
            "tag_id": self.tag_id,
            "tag_family": self.tag_family,
            "sample_count": len(self.samples),
            "mean_reprojection_error_px": float(
                np.mean([sample.reprojection_error_px for sample in self.samples])
            ),
            "selected_method": selected["method"],
            "hand_eye_candidates": candidates,  # 所有方法的结果供分析
            "camera_to_gripper": _transform_to_dict(camera_to_gripper),
            "gripper_to_camera": _transform_to_dict(gripper_to_camera),  # 最终关心的结果
            "samples": [
                {
                    "stamp": sample.stamp,
                    "tag_id": sample.tag_id,
                    "tag_ids": sample.tag_ids,
                    "reprojection_error_px": sample.reprojection_error_px,
                    "base_to_gripper": _transform_to_dict(sample.base_to_gripper),
                    "target_to_camera": _transform_to_dict(sample.target_to_camera),
                }
                for sample in self.samples
            ],
        }
        # 写 JSON 文件
        output_path = self.output_dir / f"hand_eye_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

        # 在响应中返回关键结果
        xyz = result["gripper_to_camera"]["xyz"]
        rpy = result["gripper_to_camera"]["rpy"]
        response.success = True
        response.message = (
            f"saved {output_path}; gripper_to_camera xyz="
            f"({xyz[0]:.6f},{xyz[1]:.6f},{xyz[2]:.6f}) rpy="
            f"({rpy[0]:.6f},{rpy[1]:.6f},{rpy[2]:.6f})"
        )
        self.get_logger().info(response.message)
        return response

    def _solve_hand_eye_candidates(
        self,
        r_gripper_to_base: list[np.ndarray],
        t_gripper_to_base: list[np.ndarray],
        r_target_to_camera: list[np.ndarray],
        t_target_to_camera: list[np.ndarray],
    ) -> list[dict[str, Any]]:
        """尝试所有 OpenCV 手眼标定方法，返回一致的候选结果列表。
        
        数学背景：求解 AX = XB，其中 A = base_to_gripper⁻¹, B = target_to_camera。
        OpenCV 的 calibrateHandEye 输入是 R_gripper2base 和 R_target2cam。
        """
        # OpenCV 支持的 5 种手眼标定算法
        methods = [
            ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
            ("PARK", cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
            ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]
        candidates: list[dict[str, Any]] = []
        # 预先组合完整 4x4 变换，供一致性评估使用
        base_to_gripper = [
            _make_transform(rotation, translation)
            for rotation, translation in zip(r_gripper_to_base, t_gripper_to_base)
        ]
        target_to_camera = [
            _make_transform(rotation, translation)
            for rotation, translation in zip(r_target_to_camera, t_target_to_camera)
        ]
        for name, method in methods:
            try:
                # OpenCV 返回 camera_to_gripper（即 X⁻¹：相机在夹爪坐标系中的位姿）
                r_camera_to_gripper, t_camera_to_gripper = cv2.calibrateHandEye(
                    r_gripper_to_base,
                    t_gripper_to_base,
                    r_target_to_camera,
                    t_target_to_camera,
                    method=method,
                )
                camera_to_gripper = _make_transform(
                    np.asarray(r_camera_to_gripper, dtype=np.float64),
                    np.asarray(t_camera_to_gripper, dtype=np.float64).reshape(3),
                )
                gripper_to_camera = _invert_transform(camera_to_gripper)  # 取逆得夹爪在相机中的位姿
                translation_norm = float(np.linalg.norm(gripper_to_camera[:3, 3]))  # 平移量
                det = float(np.linalg.det(camera_to_gripper[:3, :3]))  # 旋转矩阵行列式应接近 1
                consistency = self._hand_eye_consistency(
                    base_to_gripper, target_to_camera, gripper_to_camera
                )
                # 有效性验证：数值有限、旋转矩阵合法（det≈1）、平移量在合理范围
                valid = (
                    np.all(np.isfinite(camera_to_gripper))
                    and 0.8 <= det <= 1.2
                    and 0.02 <= translation_norm <= 2.0
                    and np.isfinite(consistency["score"])
                )
                candidates.append(
                    {
                        "method": name,
                        "valid": bool(valid),
                        "determinant": det,
                        "translation_norm_m": translation_norm,
                        "consistency_score": float(consistency["score"]),
                        "consistency_translation_mean_m": float(
                            consistency["translation_mean_m"]
                        ),
                        "consistency_rotation_mean_rad": float(
                            consistency["rotation_mean_rad"]
                        ),
                        "camera_to_gripper": _transform_to_dict(camera_to_gripper),
                        "gripper_to_camera": _transform_to_dict(gripper_to_camera),
                        "camera_to_gripper_matrix": camera_to_gripper.tolist(),
                        "gripper_to_camera_matrix": gripper_to_camera.tolist(),
                    }
                )
            except Exception as exc:
                # 若某种方法求解失败，记录但不中断整体流程
                candidates.append(
                    {
                        "method": name,
                        "valid": False,
                        "error": str(exc),
                        "consistency_score": float("inf"),
                    }
                )
        return candidates

    def _hand_eye_consistency(
        self,
        base_to_gripper: list[np.ndarray],
        target_to_camera: list[np.ndarray],
        gripper_to_camera: np.ndarray,
    ) -> dict[str, float]:
        """评估手眼标定结果的一致性：用求解出的 gripper_to_camera 将各样本投影到基座坐标系，
        再计算投影后基底→目标的变换的离散程度。
        
        得分越低表示一致性越好（样本间的差异小）。
        """
        # 利用 X = gripper_to_camera 计算 base_to_target：
        # base_to_target = base_to_gripper * X * target_to_camera⁻¹
        base_to_target = [
            base_to_gripper_sample @ gripper_to_camera @ _invert_transform(target_to_camera_sample)
            for base_to_gripper_sample, target_to_camera_sample in zip(
                base_to_gripper, target_to_camera
            )
        ]
        if len(base_to_target) < 2:
            # 少于 2 个采样本无法评估一致性
            return {
                "score": float("inf"),
                "translation_mean_m": float("inf"),
                "rotation_mean_rad": float("inf"),
            }
        # 以第一个样本为参考，计算其他样本与它的偏差
        reference = base_to_target[0]
        translation_errors = []
        rotation_errors = []
        for transform in base_to_target[1:]:
            delta = _invert_transform(reference) @ transform  # 相对偏差
            translation_errors.append(float(np.linalg.norm(delta[:3, 3])))  # 平移误差
            rotation_errors.append(self._rotation_angle(delta[:3, :3]))  # 旋转误差（弧度）
        translation_mean = float(np.mean(translation_errors))
        rotation_mean = float(np.mean(rotation_errors))
        return {
            "score": translation_mean + 0.25 * rotation_mean,  # 加权综合得分，旋转权重 0.25
            "translation_mean_m": translation_mean,
            "rotation_mean_rad": rotation_mean,
        }

    def _rotation_angle(self, rotation: np.ndarray) -> float:
        """从旋转矩阵计算旋转角度（弧度）。公式：θ = arccos((tr(R)-1)/2)。"""
        value = (float(np.trace(rotation)) - 1.0) * 0.5
        # 限制在 [-1, 1] 范围内防止数值舍入误差导致 acos 越界
        return float(math.acos(min(1.0, max(-1.0, value))))


def main() -> None:
    """节点入口：初始化 ROS2，创建手眼标定节点，进入事件循环。"""
    rclpy.init()
    node = AprilTagHandEyeCalibrator()
    try:
        rclpy.spin(node)  # 阻塞等待回调
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
