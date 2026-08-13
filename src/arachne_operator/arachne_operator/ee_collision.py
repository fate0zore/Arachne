"""末端相机/支撑板偏心碰撞包络（tool0 系，URDF 推导，纯 numpy，无 ROS 依赖）。

背景：2026-08-13 视点执行事故——末端相机（侧向悬臂件）挂到后货架/篮筐沿造成
结构性损坏。根因之一：碰撞模型只把末端建模为一个抓取点，相机/支撑板无体积。
本模块把相机+支撑板相对 tool0 的包络（旋转后 AABB + 采样点）做成共享几何，
teach_panel 的 keepout 检查与 grasp_preview 的碰撞模型共用，避免两端漂移。

几何来源（URDF，改动时保持同步）：
  src/arachne_description/urdf/sensors/ee_camera.xacro
    - ee_camera_support_link 碰撞 box：support 系 center (0, 0.025, 0.0025)，
      size (0.063, 0.113, 0.005)（L17-22）
    - ee_camera_link（Gemini）碰撞 box：camera 系 center (0.0051, 0, 0.0125)，
      size (0.030, 0.089, 0.025)（L44-49）
  src/arachne_description/urdf/arachne.urdf.xacro
    - support 相对 aubo_wrist3_Link（tool0 轴系）：Rz(135°)（ee_support_rpy, L6）
    - camera 相对 support：T(0, -0.0741, 0.005) Rz(90°)（ee_camera_xyz/rpy, L20-21）

推导（tool0 系 AABB，未含 margin）：
    support: center (-0.0177, -0.0177, 0.0025)  half (0.0622, 0.0622, 0.0025)
    camera : center ( 0.0488,  0.0488, 0.0175)  half (0.0421, 0.0421, 0.0125)
相机为末端唯一大体积侧向偏心件：中心离 tool0 轴线约 6.9 cm，加半尺寸后 AABB
最外缘离轴线约 9.1 cm（实际旋转 box 角点更远）；wrist 转动 + 下降时扫掠半径最大。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 默认 margin（m）：在 URDF 几何外再外扩一层，吸收标定/安装误差。
DEFAULT_EE_CAMERA_MARGIN_M = 0.010

# URDF 参数（支持板安装旋转、相机偏移，改动 URDF 时同步更新此处）
_EE_SUPPORT_RPY_Z_DEG = 135.0
_EE_CAMERA_XYZ_SUPPORT = (0.0, -0.0741, 0.005)
_EE_CAMERA_RPY_Z_DEG = 90.0

# 各件碰撞 box（各自 link 系：center + 半尺寸）
_EE_SUPPORT_BOX = ("ee_camera_support", (0.0, 0.025, 0.0025), (0.0315, 0.0565, 0.0025))
_EE_CAMERA_BOX = ("ee_camera_link", (0.0051, 0.0, 0.0125), (0.015, 0.0445, 0.0125))


@dataclass(frozen=True)
class EeBoxTool0:
    """末端偏心件相对 tool0 的 AABB 包络（含 margin，tool0 系）。"""

    name: str
    center: tuple[float, float, float]
    half: tuple[float, float, float]


def _rz_matrix(z_deg: float) -> np.ndarray:
    angle = np.deg2rad(z_deg)
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _rotated_aabb(center: tuple[float, float, float], half: tuple[float, float, float],
                  rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把轴对齐 box（center/half）经 rotation 旋转后，取其轴对齐包络（AABB）。

    旋转后 AABB 比物理 box 保守（旋转 45° 时边长投影放大 √2），方向是安全的：
    宁可在包络层面误报，也不漏掉侧向悬出的相机。
    """
    rotated_center = rotation @ np.asarray(center, dtype=np.float64)
    rotated_half = np.abs(rotation) @ np.abs(np.asarray(half, dtype=np.float64))
    return rotated_center, rotated_half


def ee_camera_boxes_tool0(
    margin_m: float = DEFAULT_EE_CAMERA_MARGIN_M,
) -> list[EeBoxTool0]:
    """返回相机+支撑板相对 tool0 的 AABB 包络（含 margin）。

    顺序：支撑板在前、相机在后（相机是主要碰撞件）。margin 默认
    DEFAULT_EE_CAMERA_MARGIN_M；传入 0.0 可得到纯 URDF 几何。
    """
    pad = max(float(margin_m), 0.0)
    support_rot = _rz_matrix(_EE_SUPPORT_RPY_Z_DEG)
    camera_rot = support_rot @ _rz_matrix(_EE_CAMERA_RPY_Z_DEG)

    boxes: list[EeBoxTool0] = []
    for name, center, half in (_EE_SUPPORT_BOX, _EE_CAMERA_BOX):
        if name == _EE_CAMERA_BOX[0]:
            # camera 相对 support 先平移再旋转：camera 局部点 → support 系 → tool0 系
            local_rotated = _rz_matrix(_EE_CAMERA_RPY_Z_DEG) @ np.asarray(center, dtype=np.float64)
            center_support = local_rotated + np.asarray(_EE_CAMERA_XYZ_SUPPORT, dtype=np.float64)
            box_center, box_half = _rotated_aabb(
                tuple(float(value) for value in center_support), half, support_rot
            )
        else:
            box_center, box_half = _rotated_aabb(center, half, support_rot)
        boxes.append(
            EeBoxTool0(
                name=name,
                center=tuple(float(value) for value in box_center),
                half=tuple(float(value + pad) for value in box_half),
            )
        )
    return boxes


def ee_envelope_points_tool0(
    margin_m: float = DEFAULT_EE_CAMERA_MARGIN_M,
) -> list[tuple[str, np.ndarray]]:
    """返回末端包络的采样点（tool0 系），供点-盒碰撞检查使用。

    每个 box 采样 8 角点 + 6 面心（14 点）：角点覆盖"包络进入 keepout"的常规
    情形；面心覆盖角点全部在 keepout 内/外的边界情形。AABB 本身已保守，
    角点/面心采样对"小包络 vs 大 keepout box"不会漏检。
    """
    points: list[tuple[str, np.ndarray]] = []
    for box in ee_camera_boxes_tool0(margin_m=margin_m):
        center = np.asarray(box.center, dtype=np.float64)
        half = np.asarray(box.half, dtype=np.float64)
        corners = np.array(
            [
                [sx * half[0], sy * half[1], sz * half[2]]
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ],
            dtype=np.float64,
        )
        for index, corner in enumerate(corners):
            points.append((f"{box.name}_corner_{index}", center + corner))
        face_centers = np.array(
            [
                [half[0], 0.0, 0.0],
                [-half[0], 0.0, 0.0],
                [0.0, half[1], 0.0],
                [0.0, -half[1], 0.0],
                [0.0, 0.0, half[2]],
                [0.0, 0.0, -half[2]],
            ],
            dtype=np.float64,
        )
        for index, face in enumerate(face_centers):
            points.append((f"{box.name}_face_{index}", center + face))
    return points
