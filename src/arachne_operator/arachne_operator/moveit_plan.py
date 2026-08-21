"""MoveIt joint-goal 规划请求构造与调用的共享辅助（供 grasp_preview 与 teach_panel 共用以防漂移）。

背景：2026-08-21 视点 #7 事故——teach_panel 视点执行走裸速度关节插值、无自碰撞检查，
途中两节手臂相碰。修复方案 A：视点执行改走 MoveIt(OMPL) 碰撞检查通道。
本模块把"构造 joint-goal 规划请求 + 发起调用/取结果"抽成纯函数，避免 grip/teach 两端
复制同一套 MoveIt 请求逻辑后漂移。

调用方职责：
  - 建好 GetMotionPlan 服务客户端并保证服务可用；
  - 提供 aubo_* 顺序的 start/goal 关节值与关节名（teach_panel 用 kinematics.JOINTS）；
  - 对 MoveIt 返回的 aubo_* 轨迹按各自 controller 期望的关节名（无前缀）做映射后执行。

纯 RCL/ROS 类型依赖：moveit_msgs.srv.GetMotionPlan、moveit_msgs.msg.*，以及 rclpy（仅
call_moveit_joint_plan 轮询用）。不含机械臂运动学，避免与 kinematics.py 耦合。
"""

from __future__ import annotations

import time
from typing import Any

import rclpy
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan

DEFAULT_GROUP_NAME = "aubo_arm"
DEFAULT_PIPELINE_ID = "ompl"

# 与 grasp_preview_pipeline.py L4980-4992 的 workspace 一致，保持两端同界。
DEF_WORKSPACE_MIN = (-1.2, -1.2, -0.3)
DEF_WORKSPACE_MAX = (1.6, 1.2, 1.2)


def build_joint_goal_constraint(
    joint_names: list[str],
    positions: list[float],
    *,
    tolerance: float,
    name: str = "moveit_joint_goal",
) -> Constraints:
    """构造 joint-goal 的 JointConstraint 列表（镜像 grasp `_moveit_joint_goal_constraint`）。"""
    constraints = Constraints()
    constraints.name = name
    tol = max(float(tolerance), 1e-4)
    for joint_name, position in zip(joint_names, positions):
        joint = JointConstraint()
        joint.joint_name = joint_name
        joint.position = float(position)
        joint.tolerance_above = tol
        joint.tolerance_below = tol
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    return constraints


def build_joint_goal_motion_request(
    request: GetMotionPlan.Request,
    *,
    start_names: list[str],
    start_positions: list[float],
    goal_joint_names: list[str],
    goal_positions: list[float],
    goal_tolerance: float,
    planner_id: str = "RRTConnect",
    group_name: str = DEFAULT_GROUP_NAME,
    pipeline_id: str = DEFAULT_PIPELINE_ID,
    planning_attempts: int = 3,
    planning_time: float = 2.0,
    velocity_scale: float = 0.5,
    acceleration_scale: float = 0.5,
    frame_id: str = "base_link",
    workspace_min: tuple[float, float, float] = DEF_WORKSPACE_MIN,
    workspace_max: tuple[float, float, float] = DEF_WORKSPACE_MAX,
    path_constraints: Constraints | None = None,
) -> None:
    """把 GetMotionPlan.Request 填成 joint-goal 规划请求（原地填充，复用调用方已建对象）。"""
    motion = request.motion_plan_request
    motion.group_name = group_name
    motion.pipeline_id = pipeline_id
    motion.planner_id = planner_id
    motion.num_planning_attempts = max(int(planning_attempts), 1)
    motion.allowed_planning_time = max(float(planning_time), 0.2)
    motion.max_velocity_scaling_factor = min(max(float(velocity_scale), 0.01), 1.0)
    motion.max_acceleration_scaling_factor = min(max(float(acceleration_scale), 0.01), 1.0)
    motion.start_state.joint_state.name = list(start_names)
    motion.start_state.joint_state.position = [float(value) for value in start_positions]
    motion.start_state.is_diff = False
    motion.workspace_parameters.header.frame_id = frame_id
    motion.workspace_parameters.min_corner.x = float(workspace_min[0])
    motion.workspace_parameters.min_corner.y = float(workspace_min[1])
    motion.workspace_parameters.min_corner.z = float(workspace_min[2])
    motion.workspace_parameters.max_corner.x = float(workspace_max[0])
    motion.workspace_parameters.max_corner.y = float(workspace_max[1])
    motion.workspace_parameters.max_corner.z = float(workspace_max[2])
    motion.goal_constraints = [
        build_joint_goal_constraint(goal_joint_names, goal_positions, tolerance=goal_tolerance)
    ]
    if path_constraints is not None:
        motion.path_constraints = path_constraints


def call_moveit_joint_plan(
    client: Any, request: GetMotionPlan.Request, timeout_sec: float
) -> tuple[bool, Any | None, str]:
    """发起规划并等待结果；成功(ok,response,msg)。timeout/空结果/error_code!=1 均视为失败并给原因。

    采用与 grasp_preview_pipeline 一致的简单轮询：调用方节点必须已有某线程在 spin
    （teach_panel 为 MultiThreadedExecutor spin 线程），否则 future 不会完成。
    """
    future = client.call_async(request)
    deadline = time.monotonic() + max(float(timeout_sec), 0.3)
    while not future.done() and time.monotonic() < deadline and rclpy.ok():
        time.sleep(0.02)
    if not future.done():
        return False, None, f"moveit plan service timeout after {timeout_sec:.2f}s"
    result = future.result()
    if result is None:
        return False, None, "moveit plan empty service result"
    response = result.motion_plan_response
    if int(response.error_code.val) != 1:
        return False, response, f"moveit plan error code={int(response.error_code.val)}"
    return True, response, "moveit plan success"
