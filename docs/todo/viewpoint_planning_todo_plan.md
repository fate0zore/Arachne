# 视点规划模块（viewpoint_planner）任务规划（viewpoint_planning_todo_plan）

> 状态：**已实现（2026-08-06 按 §4 五阶段全部完成，验收见 §5）**——规划文档（2026-08-05 创建，2026-08-06 修订）；同日集成 teach_panel 入口（§4.1）；2026-08-10 完成详细视点结果落盘（§4.2）与 RViz Marker 可视化（§4.3）；2026-08-12 完成 Viewpoint tab 页面与 JSON 列表执行（§4.4）及真机验证（§5 全部勾选）
> 创建：2026-08-05
> 更新：2026-08-06（修订：修正"无带参 service 先例"错误、外参 frame 表述、IK 种子来源、`src/CMakeLists.txt` 不存在等问题；补充关节限位/采样回显/半球采样说明）；同日按 §4 实施完成并回写本文档（§4 实现记录 / §5 验收勾选），并集成 teach_panel 入口（§4.1）；2026-08-10 完成详细视点结果落盘（§4.2）；2026-08-12 新增 Viewpoint tab 页面（§4.4：服务控制/参数规划/JSON 列表执行），修复实机执行发现的 `ttk.Treeview` state bug，真机验证完成（§5 最后一项勾选）
> 来源：用户需求（新增 `viewpoint_planner` ROS 包）+ 2026-08-05 只读代码审查结论 + 用户确认的 4 项设计决策
> 目标读者：后续接手实现 `viewpoint_planner` 包的开发者；文档应可独立执行或续接

---

## 0. 任务总结（2026-08-05）

### 0.1 需求原文

1. 新增一个 ROS 包，命名为 `viewpoint_planner`。
2. 本项下的视点规划**不是先验的**：首先需要确定目标的位置（这部分已有现成代码）。
3. 视点规划的逻辑要支持不同策略：当下只做**斐波那契球面采样**，但需要留出拓展余地，后续可能接入更多更高级的算法。

### 0.2 已确认设计决策（2026-08-05 用户确认）

| 决策项 | 结论 |
|---|---|
| 接口形态 | ROS 2 **service**（请求=目标位置，响应=视点列表） |
| 执行范围 | **只规划，不运动**（不调用 `/arachne/aubo/move_joint`） |
| 目标输入 | **请求参数直接传入**（base_link 系目标 xyz；调用方从现成定位链路取 `base_grasp_xyz`） |
| 采样默认值 | 半径 **0.5 m** / **32 点**，默认 **IK 可达过滤**（均可用参数覆盖） |
| **service 选型**（2026-08-06 补充） | **纯 `ament_python` 包 + 独立接口包生成自定义 `PlanViewpoints.srv`**（详见 §2.1 修订） |
| **IK 种子来源**（2026-08-06 补充） | **节点订阅机械臂当前关节状态**作为 `solve_pose` 的 `q_start`（详见 §2.4 修订） |

### 0.3 合理性评估（用户要求"不符合常识则告知并确认"）

- ✅ **需求本身符合常识**：目标定位已有现成代码（`grasp_preview_pipeline.py`，§1.1），在定位之后"环绕目标采样视点"是主动感知/多视角观察的标准做法；斐波那契球面采样是球面均匀采样的经典起点。
- ⚠️ **风险项 1（精度）**：相机外参（`ee_camera_link→camera_color_optical_frame`）当前为**经验试调值**（非可信手眼标定结果，§1.2）。视点规划的"期望相机位姿 → tool0 位姿"变换依赖完整 `tool0→camera` 变换（含该外参），外参误差会传导为 tool0 位姿误差。**不阻塞模块开发**，视点精度验收留待外参可信之后（与 `hand_eye_calibration_next_steps.md` 阶段 B/C 衔接，见附录 A）。
- ⚠️ **风险项 2（碰撞）**：仓库**无现成碰撞检测设施**（MoveIt 配置存在但主执行链路用自研 IK，不用 move_group）。斐波那契球面采样是**先验均匀采样**，不感知遮挡/碰撞。第一版只做 **IK 可达性过滤**（复用自研 `AuboI5Kinematics`，§1.3），**碰撞/遮挡检查留扩展位**（§3.3）。
- 两项风险已在本节记录，用户已知晓并接受（决策确认时未提出异议）。

---

## 1. 现状调研（2026-08-05 只读代码审查结论，带 file:line）

### 1.1 目标定位现成代码（"确定目标位置"的来源）

- 唯一 3D 目标定位节点是 **`grasp_preview_pipeline.py`**（`src/arachne_operator/`）：订阅 color/depth 图，内部跑 YOLO 检测（`--model yolo26n`），做深度反投影 3D 化 + 点云抓取形状估计。
- 定位结果以 **String JSON** 发布到 **`/arachne/perception/taco_instances`**（`_publish_preview_detection_event`，L3393-3433），字段含：
  - `base_grasp_xyz`：目标抓取点在 **base_link 系**的 xyz（L3406-3410）；
  - `grasp_camera_xyz`：同一目标点在 camera 系坐标（L3405）；
  - `confidence`、`bbox_xyxy`、`waypoints_base`、`planning_key_waypoints`、`stamp` 等。
- 3D 化链路：YOLO bbox/掩膜 + 深度图 → `_pixels_to_xyz`（L3764-3776）反投影到 camera 系 → TF 变换到 `base_link`（`_base_from_depth_transform` L1504、`_make_base_path` L5813-5843）。
- 消费方先例：`step_cleanup_demo.py`（L59 默认 `detection_topic=/arachne/perception/taco_instances`，L406 `_detection_cb`，L457 读 `base_grasp_xyz`）、`road_cleanup_task_server.py`（L321 `_detection_cb`，L1256 `_candidate_reachable` 读 `base_grasp_xyz`）。
- **结论**：视点规划的目标位置直接复用该链路输出的 `base_grasp_xyz`（base_link 系）。按已确认决策，由调用方读取后作为 service 请求参数传入，`viewpoint_planner` **不依赖任何感知话题**，与感知解耦。

### 1.2 相机 TF 链与外参现状

- 相机为 **eye-in-hand**（装在机械臂末端）：
  - URDF `src/arachne_description/urdf/sensors/ee_camera.xacro`（L33-62）：`ee_camera_link` 通过 `ee_camera_joint` 挂在末端支撑上（xyz=(0.0,-0.0741,0.005)、rpy=(0,0,1.5708)）。
  - `gemini335.launch.py` L195-201：`camera_parent_frame=ee_camera_link`，color/depth optical frame 的 static TF 使用同一组 `camera_optical_*` 默认值。
- **当前生效外参**（`config/hand_eye_camera_extrinsics/current_effective.json`）：parent=`ee_camera_link`、child=`camera_color_optical_frame`，xyz=(0.0201, 0, 0.05)、rpy=(0, 0, -1.570796327)，`source_json="gemini335.launch.py default"` —— 即**经验试调值，非可信手眼标定结果**（可信标定在 `hand_eye_calibration_next_steps.md` 阶段 B/C，未完成）。
- **frame 澄清（2026-08-06 修订）**：该外参是 **`ee_camera_link → camera_color_optical_frame`**，**不是 `tool0 → camera`**。从 `tool0` 到相机光学系需叠加 URDF 固定链 `tool0 → ee_camera_support_link → ee_camera_link`（`ee_camera.xacro` 的 `arachne_ee_camera` 宏，L59-62）。**实现时不应手写该外参做矩阵换算，而应直接用 TF `lookup_transform(target_frame="tool0", source_frame="camera_color_optical_frame")` 解析完整 `tool0→camera` 变换**（TF 已含 URDF 固定关节 + 该外参 static TF），避免漏掉中间链。
- **影响**：视点规划的 `tool0 = camera_pose × (tool0→camera)⁻¹` 依赖完整 `tool0→camera` 变换（含上述 URDF 链 + 该外参）；外参误差直接传导为 tool0 位姿误差。设计上把外参做成**可配置参数**（默认取 current_effective.json 值，经 TF 生效），外参可信后仅需更新参数，无需改代码。

### 1.3 IK 与运动执行链

- **IK**：自研 `AuboI5Kinematics`（`src/arachne_operator/arachne_operator/real_hardware_acceptance_test.py` L45，FK/IK 链 `aubo_base_link → tool0`），数值 IK 入口 `solve_pose`（L152）；`grasp_preview_pipeline.py` L857 实例化使用（`self.kinematics = AuboI5Kinematics()`，IK 容差/阻尼常量在 L116-121）。**注意：`solve_pose` 是数值迭代 IK，不强制关节限位**，可达性判断需补充关节限位检查（见 §2.4 修订）。
- **执行**：`/arachne/aubo/move_joint` action（`src/arachne_hardware/action/AuboMoveJoint.action`，goal=`target_joints/speed_rad_sec/accel_rad_sec2/blend_radius/...`），客户端 `aubo_move_joint_client.py`。**本模块按决策只规划不运动，不调用执行接口。**
- **坐标系注意**：`base_link`（目标/视点所在 frame）与 `aubo_base_link`（IK 链基座）是**两个 frame**；`grasp_preview_pipeline.py` 用 `_base_point_to_aubo`（L6206-6219）做 base_link → aubo_base_link 变换。视点 IK 过滤前必须把 tool0 目标位姿变换到 aubo_base_link 系（TF lookup 或参数指定）。

### 1.4 包构建与接口生成先例

- 纯 Python 包：`ament_python`（package.xml `<build_type>ament_python</build_type>` + `setup.py` 的 `console_scripts`），先例：`src/arachne_operator/setup.py`（L7-46）、`src/arachne_sensors/package.xml`。
- 自定义 ROS 接口：`ament_cmake` + `ament_cmake_python` + `rosidl_generate_interfaces`，先例：`src/arachne_hardware/CMakeLists.txt`（L4-12，生成 `AuboMoveJoint.action`；Python 源码 install 到 `PYTHON_INSTALL_DIR`）。
- 服务惯例：仓库内**节点对外暴露的**服务多为 `std_srvs/Trigger` 无参触发（如 `demo_orchestrator.py` L149-163）；但**调用方**已有携带复杂参数的 service 先例 —— `grasp_preview_pipeline.py` 的 `_call_moveit_pose_plan`（L4944+）构造 `GetMotionPlan.Request()` 调用 MoveIt `moveit_msgs/GetMotionPlan`。**即"无带参 service 先例"不成立**：带参 service 在仓库已有使用，且纯 `ament_python` 节点完全可 `create_service` 带参接口（只要接口由独立接口包生成）。本模块 service 需携带目标位置与参数，推荐自定义 `PlanViewpoints.srv`（接口由独立 `ament_cmake` 接口包生成，`viewpoint_planner` 自身保持纯 `ament_python`，见 §2.1 修订）。

---

## 2. 接口设计

### 2.1 service 选型（2026-08-06 定稿）

> **修订说明**：初版以"仓库无带参 service 先例 → 方案 A 需混合 build 代价高"为论据，该论据**有误**（见 §1.4 修订）。经 2026-08-06 复审，重新定稿如下。

**结论：纯 `ament_python` 的 `viewpoint_planner` 包 + 独立接口包（`ament_cmake`）生成自定义 `PlanViewpoints.srv`。**

- `viewpoint_planner` 自身**保持纯 `ament_python`**（`console_scripts` 入口，`create_service(PlanViewpoints, ...)`），与 `arachne_operator`/`arachne_sensors` 一致，构建简单。
- 自定义接口 `PlanViewpoints.srv` / `Viewpoint.msg` 由**独立的 `ament_cmake` 接口包**（如 `viewpoint_planner_msgs`）经 `rosidl_generate_interfaces` 生成 —— 仿 `arachne_hardware` 的接口生成方式（L9-12），但接口与节点分离，避免把节点包变成混合 build。
- **理由**：类型安全、字段自文档化（优于 String JSON）；同时规避了初版方案 A"节点包混合 build"的代价（节点包仍为纯 Python）。接口包只含 `.srv/.msg` + 精简 `CMakeLists.txt`，构建开销小。

### 2.2 service 草案（接口由独立接口包生成，见 §2.1）

- 服务名：`/arachne/viewpoint/plan`（节点内参数 `viewpoint_plan_service` 可改，默认值如上）。

```text
# PlanViewpoints.srv —— 请求
float64[3] target_xyz_base     # 目标位置，base_link 系（来源：taco_instances.base_grasp_xyz 或调用方指定）
string strategy                # 策略名，默认 "fibonacci_sphere"
float64 radius_m               # 采样半径（球心=目标），默认 0.5
uint32 num_viewpoints          # 采样点数，默认 32
bool filter_ik_reachable       # 是否 IK 可达过滤，默认 true
float64[3] up_vector_base      # 球面参考轴，默认 [0, 0, 1]（可空）
---
# 响应
bool success
string message
Viewpoint[] viewpoints         # 视点列表（嵌套消息）
```

```text
# Viewpoint.msg —— 每个视点
builtin_interfaces/Time stamp  # 采样/规划时刻（2026-08-06 补充：便于多次调用对齐）
float64[3] camera_xyz_base     # 期望相机位置，base_link 系
float64[4] camera_quat_xyzw    # 期望相机姿态，base_link 系（光轴指向目标）
float64[3] tool0_xyz_base      # 换算后的 tool0 位置，base_link 系
float64[4] tool0_quat_xyzw     # 换算后的 tool0 姿态，base_link 系
bool ik_reachable              # IK 可达标志（过滤关闭时仍逐点标记）
float64[6] ik_joints           # 可达时的关节解（aubo 关节角；不可达时为 0）
```

### 2.3 坐标系与变换约定

1. **采样在 base_link 系**：球心 = `target_xyz_base`，半径 = `radius_m`，参考轴 = `up_vector_base`（默认世界 z 轴）。
2. **相机姿态**：相机光轴（`camera_color_optical_frame` 的 +z）指向球心（目标），即 `camera_pose` 的 z 轴从采样点指向目标。
3. **tool0 换算**：`tool0 = camera_pose × (tool0→camera)⁻¹`。其中 `tool0→camera` 变换**用 TF `lookup_transform(target_frame="tool0", source_frame="camera_color_optical_frame")` 实时解析**（含 URDF 固定链 + 相机外参，见 §1.2 frame 澄清），而非手写单一外参矩阵。外参经 TF 生效、保持可配置（默认 `current_effective.json` 值；外参可信后仅更新参数）。
4. **IK 过滤**：将 tool0 位姿经 base_link → aubo_base_link 变换后（TF `lookup_transform`，仿 `_base_point_to_aubo` L6206-6219），调用 `AuboI5Kinematics.solve_pose` 求解；仅保留可达视点（可同时输出关节解）。

### 2.4 IK 过滤实现要点（2026-08-06 修订补充）

> 初版未说明 IK 种子来源与关节限位，实现时绕不开，补充如下。

- **IK 种子（`q_start`）**：`solve_pose` 是数值迭代 IK，需初值。现有代码用**当前机械臂关节角**（`grasp_preview_pipeline.py` L5217 `q_seed = self.current_arm_joints`）。`viewpoint_planner` 为独立节点，**采用"订阅关节状态"方案（2026-08-06 用户确认）**：节点订阅机械臂关节状态话题 **`/joint_states`（实现已确认，节点参数 `joint_states_topic` 可改）**，以最新关节角作为 `q_start`，使 IK 解贴近当前实际位形、避免奇异/不收敛。
- **关节限位**：`AuboI5Kinematics.solve_pose` 是数值 IK、**不强制关节限位**（§1.3）。`ik_reachable` 判定需在 IK 收敛后**额外校验关节解是否落在 Aubo 关节限位内**（限位取 URDF `aubo_i5.urdf`/`arachne_description` 或参数），超限则标记不可达。
- **多视点多解**：同一球面上多个视点，可串行用前一视点解作下一视点种子（`grasp_preview_pipeline.py` L5347 `q_seed = q_solution` 已有此模式），提高相邻视点解的连续性。

---

## 3. 策略抽象设计

### 3.1 统一接口 + 注册表（扩展点）

```python
# viewpoint_planner/strategies/base.py
class ViewpointStrategy(ABC):
    name: str  # 注册名，service 的 strategy 字段按此选择

    @abstractmethod
    def plan(
        self,
        target_xyz_base: tuple[float, float, float],
        radius_m: float,
        num_viewpoints: int,
        up_vector_base: tuple[float, float, float] = (0.0, 0.0, 1.0),
        **kwargs,
    ) -> list[ViewpointSample]:  # ViewpointSample: camera_pose_base（xyz + quat）
        ...

STRATEGIES: dict[str, type[ViewpointStrategy]] = {}  # 注册表：装饰器/显式注册

def register_strategy(cls: type[ViewpointStrategy]) -> type[ViewpointStrategy]: ...
def get_strategy(name: str) -> ViewpointStrategy: ...  # 未知名报错并列出可用策略
```

### 3.2 `FibonacciSphereStrategy`（第一个实现）

- 算法：golden-angle 斐波那契球面均匀采样（`z_i = 1 - 2(i+0.5)/N`，`φ_i = i·golden_angle`），球心=目标、半径=`radius_m`、参考轴=`up_vector_base`。
- **全球面 vs 半球（2026-08-06 补充）**：上述公式是**全球面**均匀采样。实际视点规划常只需**上半球**（目标上方环绕观察）。设计上支持参数切换：默认全球面；半球模式取 `z_i ∈ [0, 1]`（即 `z_i = (i+0.5)/N`）后仍以 `up_vector_base` 为极轴。是否默认上半球由 launch 参数决定（默认全球面，与"采样在完整球面"语义一致）。
- 每个采样点构造相机位姿：位置=采样点；姿态=相机 +z 轴指向球心（目标），+y 轴与参考轴约束解算（消歧/退化处理：采样点与参考轴平行时的兜底）。
- **纯函数、无 ROS 依赖**，可离线单测：单位半径、朝向指向目标、点间距均匀性（最小弦长/覆盖度）。

### 3.3 扩展位（后续高级策略，本次不做）

- 信息增益/主动观测（NBV）、遮挡感知（利用深度点云评估可见性）、碰撞/workspace 过滤、非球面约束（如地面机器人视角）。
- 接入方式：实现 `ViewpointStrategy` 接口 + 注册即可，service 的 `strategy` 字段选择，**无需改节点/接口**。
- 过滤环节设计为策略之后的**独立管道环节**（IK 过滤可插拔、可组合多级过滤），避免把过滤逻辑绑死在某个策略内。

---

## 4. 实现阶段（2026-08-06 已按此全部实施）

### 4.0 实现记录（2026-08-06 回写）

| 阶段 | 交付 | 验证 |
|---|---|---|
| Phase 1 包脚手架 | `src/viewpoint_planner_msgs/`（`PlanViewpoints.srv` + `Viewpoint.msg`，ament_cmake 仿 arachne_hardware L9-12）+ `src/viewpoint_planner/`（ament_python：package.xml/setup.py/setup.cfg/resource/launch/config）；`scripts/build/build_workspace.sh` 的 `PACKAGES` 追加两包（该脚本用 `--packages-select` 显式列包，新包必须手动加入，否则全量构建不含新包） | 全量构建 17 包通过 |
| Phase 2 策略核心 | `strategies/base.py`（`ViewpointStrategy` 抽象 + `STRATEGIES` 注册表 + `register_strategy`/`get_strategy`，未知名报错并列出可用项）+ `strategies/fibonacci_sphere.py`（golden-angle，全球面 `z_i=1-2(i+0.5)/N`、半球 `z_i=(i+0.5)/N` 参数切换；相机 +z 指向球心、+y 贴近参考轴、极点退化兜底；纯函数无 ROS 依赖） | 14 项离线单测（球面半径/数量、光轴指向目标、无重复点、退化、注册表） |
| Phase 3 IK 可达过滤 | `AuboI5Kinematics` 抽离至 `src/arachne_operator/arachne_operator/kinematics.py`（`real_hardware_acceptance_test.py` 改 import 保持兼容），新增 `AUBO_I5_JOINT_LIMITS`（URDF 仅 ±2π 不可用，取官方名义值，节点参数 `joint_limits` 12 值可覆盖）；`ik_filter.py`（solve_pose 参数化参照 grasp_preview L116-121 + 限位校验 + 串行种子）；`tf_utils.py`（tool0←camera 用 TF `lookup_transform(target_frame="tool0", source_frame="camera_color_optical_frame")` 实时解析、base_link→aubo_base_link 完整 4×4 含姿态） | py_compile + FK/IK 自收敛冒烟 + 11 项单测（mock TF/关节状态） |
| Phase 4 service 节点 | `viewpoint_planner_node.py`：`create_service(PlanViewpoints, "/arachne/viewpoint/plan")`（参数 `viewpoint_plan_service` 可改）；默认 radius 0.5 / 32 点 / fibonacci_sphere / IK 过滤开（注意：ROS2 service request bool 默认 False，无法区分"未设置"，实现用 `filter = request 值 or 节点参数` 保持"默认开"语义，关闭需设节点参数）；订阅 `/joint_states` 作 IK 种子；响应含 camera/tool0 位姿 + ik_reachable/ik_joints；异常兜底必应答；launch `viewpoint_planner.launch.py`（参数可覆盖）；`--dry-run-check` 离线入口 | service 端到端冒烟（无 TF 时按设计返回明确错误 message）+ dry-run ok |
| Phase 5 测试与验收 | 见 §5 | 全量构建 17 包通过、25 单测全绿、全部 dry-run ok（含 arachne_operator 三 server，证明 kinematics 抽离无回归）、`git status --short` 已检查 |

**未执行项**：真机 TF 环境完整 service 冒烟（真实 TF + IK 过滤输出 32 点并逐点标记可达性）——按 §5 最后一项与用户约定，待用户确认机器人就绪后执行。

### 4.1 teach_panel 入口集成（2026-08-06 追加）

为 teach_panel 增加视点规划入口：Home 页 quick 区 `Runtime Services` 按钮组新增「Viewpoint Plan」按钮（`src/arachne_operator/arachne_operator/teach_panel.py`，`_build_home_tab` 的 `_build_button_group` 元组末尾）。

- **目标输入（两者结合，用户确认）**：点击按钮弹出 `_prompt_viewpoint_plan()` 模态对话框（Toplevel），X/Y/Z 三个 Entry 手动输入（base_link 系，无检测值时默认 0.3/0.0/0.2）；同时 teach_panel 节点订阅 `/arachne/perception/taco_instances`（参数 `taco_instances_topic` 可改），缓存最新 `base_grasp_xyz` 到 `latest_detection_xyz`（lock 保护、解析失败静默），弹窗打开时若存在则自动预填并标注来源。球心语义照附录 C（`base_grasp_xyz` 为抓取点非物体中心，调用方负责修正）。
- **调用链路**：`_prompt_viewpoint_plan` → `node.call_viewpoint_plan((x,y,z))` → `_call_viewpoint_plan_worker`：`wait_for_service(4s)`；**节点未启动则自动拉起**（复用 managed service 框架：`viewpoint_planner` 已注册进 `MANAGED_SERVICE_COMMAND_PARAMS`/`MANAGED_SERVICE_READY_SERVICES`，启动命令取参数 `viewpoint_planner_command`，日志写 `<session>/viewpoint_planner.log`）→ 轮询 `wait_for_service` 最多 30s（节点进程提前退出则中止）→ 仍不可用才 warn 并提示手动启动命令 → 构造 `PlanViewpoints.Request`（`radius_m`/`num_viewpoints`/`filter_ik_reachable` 取 teach_panel 参数 `viewpoint_plan_radius_m`(0.5)/`viewpoint_plan_num_viewpoints`(32)/`viewpoint_plan_filter_ik`(true)，`strategy="fibonacci_sphere"`）→ `call_async` + `_wait_service_future(20s)` → 状态栏摘要（视点数/IK 可达数/target）+ `_write_event("viewpoint_plan_result", ...)`（注意 `_write_event(kind, message, **fields)` 的 `message` 只能出现一次，附加内容用 `server_message=` 等字段名）；超时/失败/异常均回显不阻塞 UI。服务端 `filter_ik` 语义为**逐点标记**（保留全部采样点，`Viewpoint.ik_reachable` 标记；不删除采样点），如 `planned 32 viewpoints ... ik_reachable=13/32`。
- **依赖与参数**：`src/arachne_operator/package.xml` 新增 `<exec_depend>viewpoint_planner_msgs</exec_depend>`；teach_panel 参数 `viewpoint_plan_service`（默认 `/arachne/viewpoint/plan`）、`viewpoint_plan_radius_m`、`viewpoint_plan_num_viewpoints`、`viewpoint_plan_filter_ik`、`taco_instances_topic`、`viewpoint_planner_command`（默认 `ros2 launch viewpoint_planner viewpoint_planner.launch.py`）。**关键坑（2026-08-05 实测）**：本环境 `get_service_names_and_types()`（graph 缓存）与真实服务可用性（`wait_for_service`/rmw）可能不一致——节点存活时 graph 查询反而可能为空、节点死亡后缓存可能仍含旧名。因此 auto-start 调用 `_start_managed_process_worker("viewpoint_planner", skip_external_check=True)` **跳过「外部已就绪」判定**：`wait_for_service(4s)` 失败即无条件拉起进程，再轮询 client 可用性（最多 30s，进程提前退出则中止），避免误判「already available」导致不拉起、30s 空等。
- **验证（2026-08-06）**：py_compile、`teach_panel --headless-check` 通过；service 冒烟：节点拉起后 `/arachne/viewpoint/plan` 就绪、调用返回设计预期的 TF 错误（当前无 TF 环境）；自动启动链路（`wait_for_service` 失败 → managed 拉起 → 轮询就绪）代码路径与既有 managed service 机制一致；GUI 点击验证留待用户重启 teach_panel 后确认。
- **详细视点结果落盘（2026-08-10 已实现，§4.2）**：见 §4.2 记录。可选后续接 RViz Marker 可视化。
- **约束**：未修改 `teach_panel.launch.py`；入口只规划不运动（调用 `/arachne/viewpoint/plan`，不触碰 move_joint）。

### 4.2 详细视点结果落盘（2026-08-10 实现）

规划成功后，teach_panel 将请求参数 + 全部 `Viewpoint` 详细数据写入 `<session>/viewpoint_plan_<YYYYmmdd_HHMMSS_fff>.json`（`<session>` 即 `log/teach_panel/<YYYYmmdd_HHMMSS>`；文件名毫秒级，避免同秒两次规划覆盖）。

- **序列化**：模块级纯函数 `_serialize_viewpoint_plan(request, response)`（`teach_panel.py`，无 ROS 状态、可离线单测）：顶层 `planning_stamp`（ISO 毫秒）/`request`（六字段）/`server_message`/`num_viewpoints`/`ik_reachable_count`/`viewpoints`；每个 `Viewpoint` 全字段（`stamp`→`{sec,nanosec}`、`camera_xyz_base`/`camera_quat_xyzw`/`tool0_xyz_base`/`tool0_quat_xyzw`/`ik_reachable`/`ik_joints`），浮点数组转 `list[float]`。
- **写入**：`_write_viewpoint_plan_result(request, response)` 在 `_call_viewpoint_plan_worker` 的 `response.success` 分支调用，`json.dumps(..., ensure_ascii=False, indent=2)` 写 `_viewpoint_plan_result_path(self.runtime_log_dir)`；OSError 仅 `get_logger().warning`，不阻塞 UI（与 `_write_event` 容错一致）。
- **事件**：`viewpoint_plan_result` 事件追加 `result_file`（写盘成功，绝对路径便于直接打开）或 `result_file_error`（失败）。
- **验证（2026-08-10）**：py_compile + 全量构建 17 包 + 离线冒烟（结构/字段/Time/空 viewpoints/写读往返）+ `teach_panel --headless-check` 全绿；真机 GUI 点击链路与落盘文件留待用户重启 teach_panel 后确认。

### 4.3 RViz Marker 可视化（2026-08-10 实现）

规划成功后，`viewpoint_planner` 服务端在 `/arachne/viewpoint/markers`（参数 `markers_topic` 可改）发布 `visualization_msgs/MarkerArray`（frame=`markers_frame`，默认 `base_link`），把规划结果叠加到 RViz 机械臂模型上直观检查——真机验收「视点相对目标/工作空间是否合理」的直观手段，也能暴露外参误差。

- **内容**：首条固定 `DELETEALL`（每次规划先清空旧结果；失败/异常也只发 DELETEALL，不残留过期显示）；目标点 `SPHERE`（id=0，黄，直径 0.08 m，默认视角可见）；每个视点一个 `ARROW`（id=1..N，points 模式：起点=`camera_xyz_base`、终点=起点+光轴方向×`radius×0.35`，光轴由 `camera_quat_xyzw` 经 `quat_to_matrix_xyzw` 第 3 列解出；`ik_reachable=true` 绿 / false 红；杆径 0.025/头径 0.06/头长 0.10 m）。
- **实现**：`viewpoint_planner/markers.py` 纯函数 `build_viewpoint_marker_array(request, response, frame_id)`（无节点依赖、可离线单测）；节点 `_handle_plan` 统一在 return 前 `_publish_markers`（成功/失败/异常全路径），**QoS `TRANSIENT_LOCAL`（RViz2 MarkerArray display 默认 Durability 即 Transient Local；曾误改 VOLATILE 导致 RViz 报 DURABILITY 不兼容、看不到 marker，已改回）**；参数 `publish_markers`（默认 true）/`markers_topic`/`markers_frame`，launch 可覆盖。
- **IK 耗时控制（2026-08-10 补充）**：不可达目标的数值 IK 每点迭代满预算（180 次实测 861ms/点，32 点全不可达 ≈27.6s，击穿 teach_panel 20s 超时）。修复：`IkParams.max_iterations` 180→100（可达点仍收敛）+ `filter_poses(time_budget_sec)` 总预算（节点参数 `ik_time_budget_sec` 默认 12s，超预算剩余点标记不可达不求解）+ teach_panel `viewpoint_plan_service_timeout_sec` 20s→45s 兑底。
- **⚠️ RViz display 未添加**（2026-08-10 用户决定）：**不改任何 `.rviz` 配置文件**；用户需在 RViz 手动添加——Add → By topic → `/arachne/viewpoint/markers`（MarkerArray），或手动加 `MarkerArray` display 填该 topic。
- **验证（2026-08-10）**：py_compile + 全量构建 17 包 + 41 单测全绿（`test_markers.py` 11 例 + `FilterPosesBudgetTest` 2 例：预算内求解/超预算跳过）；真机视觉确认留待用户 RViz 手动加载后检查。

### 4.4 Viewpoint tab 页面与 JSON 列表执行（2026-08-12 实现）

应需求重构 teach_panel 页面：新增独立 **Viewpoint** label 页（`src/arachne_operator/arachne_operator/teach_panel.py`，`_build_viewpoint_tab`，注册于 Move tab 之后；Home 页「Viewpoint Plan」按钮保留为双入口），自上而下三个分区：

1. **服务控制区**：`Start Planner` / `Stop Planner`（复用 managed service 框架 `start_/stop_managed_process("viewpoint_planner")`）+ 状态标签实时显示进程状态与 `/arachne/viewpoint/plan` 服务就绪度（`snapshot()` 新增 `viewpoint_planner` 键，`_refresh_viewpoint` 每 100ms 刷新，非阻塞 `wait_for_service(0)` 探测）。
2. **规划参数区**：X/Y/Z（base_link 系，预填优先级沿用 `latest_detection_xyz` > 会话缓存 > 默认 0.3/0.0/0.2）、Radius(m)、Count、`Filter IK` 勾选（默认节点参数 `viewpoint_plan_filter_ik`）、strategy 只读（当前仅 `fibonacci_sphere`）；`Plan` 按钮 → `_plan_viewpoint_from_tab`（校验 Radius>0、Count∈[1,`MAX_VIEWPOINT_COUNT`]，写回会话缓存与 Home 弹窗互通）→ `node.call_viewpoint_plan`（新增 `filter_ik`/`on_success` 参数，`filter_ik=None` 回落节点参数，语义仍为逐点标记不删点）；成功后摘要显示 `N viewpoints, M ik-reachable | result_file 绝对路径`。
3. **结果列表区（JSON 加载 + 手动执行）**：`Load JSON...`（`filedialog` 默认 `runtime_log_dir`）或规划成功后**自动加载**最新 `viewpoint_plan_*.json`（`on_success` 回调经 `root.after` 回 UI 线程）；模块级纯函数 `_parse_viewpoint_plan_file`（无 ROS/Tk 依赖、可离线单测：校验 `viewpoints`/字段、`ik_joints` 弧度→度）；`ttk.Treeview` 五列 `# / reach / camera_xyz / tool0_xyz / joints(deg)`，不可达行灰显（tag）且拒绝执行；选中行 + `Execute`（或双击）弹确认框（视点号 + tool0 位置 + 6 关节角）→ 确认后 `node.move_arm_to_joints_degrees(ik_joints_deg, on_done=...)`（与 Move tab 同链路：keepout 检查 + 速度流式 + control owner/teach 门禁）；执行期间 busy 保护（按钮/列表禁用，`on_done` 恢复），`viewpoint_execute` 事件落盘（视点号/tool0/关节角/来源文件）。

**node 层小扩展（向后兼容）**：`call_viewpoint_plan(target_xyz, radius_m, num_viewpoints, filter_ik=None, on_success=None)`；`move_arm_to_joints_degrees(degrees, on_done=None)`（worker finally 触发回调）。

**⚠️ 实机执行失败根因与修复（2026-08-12）**：首版实机点击 Execute 失败且 events.jsonl 无任何记录——根因是 `_set_viewpoint_execute_enabled` 对 `ttk.Treeview` 误用 `configure(state=...)`（Treeview 不支持该选项，`state` 是 ttk widget 的独立方法），`_tkinter.TclError: unknown option "-state"` 被 Tk 吞掉（仅打印到 `log/desktop_teach_panel/teach_panel_*.log`），且异常发生在写事件与启动运动**之前**，导致两者均未执行。修复：① Treeview 改用 `state(["disabled"])`/`state(["!disabled"])`；② 顺序调整——先写 `viewpoint_execute` 事件、先启动运动，再禁用 UI；③ `_set_viewpoint_execute_enabled` 加异常兜底（仅 warn 不阻断运动）。修复后实机执行通过（见 §5）。

**验证（2026-08-12）**：py_compile + 全量构建 17 包 + headless-check + 新增 7 例离线单测（`src/arachne_operator/test/test_viewpoint_plan_parse.py`：结构/弧度→度/可达标记/异常路径）全绿；实机：规划 32 视点 5~6 可达 → Execute 运动到位 → RViz marker 对照合理（用户确认）。


- **Phase 1 包脚手架**：新建 `src/viewpoint_planner/`（纯 `ament_python`：package.xml + setup.py、`launch/`、`config/`）+ 独立接口包 `src/viewpoint_planner_msgs/`（`ament_cmake`，含 `PlanViewpoints.srv`/`Viewpoint.msg`，仿 `arachne_hardware` L9-12 接口生成）；声明 console_scripts 入口（节点 + `--dry-run-check`）。**注：colcon 递归发现 `package.xml`，无需 `src/CMakeLists.txt` 登记（初版此处有误）**；只需保证两包在 `src/` 下即可被 `build_workspace.sh` 构建。
- **Phase 2 策略核心**：`strategies/base.py`（抽象 + 注册表）、`strategies/fibonacci_sphere.py`（纯函数）；单测覆盖球面分布、朝向、参考轴退化。
- **Phase 3 IK 可达过滤**：复用 `AuboI5Kinematics`（import `arachne_operator.real_hardware_acceptance_test.AuboI5Kinematics`，或视需要抽离为公共模块）；订阅关节状态作 IK 种子、补充关节限位校验（见 §2.4）；实现 base_link → aubo_base_link 变换与奇异处理。
- **Phase 4 service 节点 + launch**：`ViewpointPlannerNode`（`create_service(PlanViewpoints, ...)`，默认参数 radius/count/strategy/filter 从 launch/参数读取）；launch 文件；`--dry-run-check` 离线验证入口。
- **Phase 5 测试与验收**：`python3 -m py_compile`、`--dry-run-check`、离线单测；全量构建走 `./scripts/build/build_workspace.sh`；真机验证（对真实 `taco_instances` 目标调用 service、检查输出视点合理性）**留待用户确认后执行**。

---

## 5. 验收标准（2026-08-06 状态）

- [x] `viewpoint_planner`（节点包）与 `viewpoint_planner_msgs`（接口包）构建通过，launch 可启动，`/arachne/viewpoint/plan` service 可调用（`ros2 service call` 可完成一次规划；已冒烟验证请求解析与错误路径应答）。
- [x] 斐波那契策略默认参数（0.5 m / 32 点）输出 32 个视点：全部落在球面上、光轴指向目标、无重复（离线单测断言）。
- [x] 默认开启 IK 过滤时，输出视点的 `ik_reachable=true` 且 `ik_joints` 为有效关节解；关闭过滤时全量输出但逐点标记可达性（dry-run 自位姿断言 + 单测覆盖）。
- [x] 扩展性验证：注册表机制可用（模拟注册第二个策略，service 的 `strategy` 字段可切换；未知策略名返回明确错误）。
- [x] `--dry-run-check` / 单测路径不触碰真实硬件（符合 AGENTS.md）。
- [x] teach_panel 入口（2026-08-06 追加，§4.1）：Home 页 `Runtime Services` 按钮组「Viewpoint Plan」按钮 + 弹窗目标输入（手动 xyz / 最新检测值预填），编译 + headless-check + 全量构建 + service 回归冒烟通过；GUI 点击链路待真机确认。
- [x] teach_panel 详细视点结果落盘（2026-08-10，§4.2）：规划成功写入 `<session>/viewpoint_plan_<时间戳>.json`（请求参数 + 全部 Viewpoint 字段），`events.jsonl` 记录 `result_file`/`result_file_error`；py_compile + 全量构建 + 离线冒烟 + headless-check 通过；RViz Marker 可视化未做（可选后续）。
- [x] RViz Marker 可视化（2026-08-10，§4.3）：服务端发布 `/arachne/viewpoint/markers`（DELETEALL + 目标球 + 每视点箭头绿/红，失败只清空），`test_markers.py` 11 例 + 全量构建通过；RViz display 未改配置文件，留待用户手动添加后真机视觉确认。
- [x] 真机验证项（2026-08-12 用户确认完成）：对真实目标调用 service，规划 32 视点、5~6 个 IK 可达，视点相对目标/机械臂工作空间合理；Viewpoint tab `Execute` 运动到位（含 §4.4 的 `ttk.Treeview` state bug 修复后复测通过）。

---

## 6. 安全与流程约束

- 本模块**只规划不运动**，天然不触碰 AGENTS.md「真机运动需显式确认」红线；未来若增加执行能力，必须沿用 `/arachne/aubo/move_joint` + teach 门禁流程，不得默认路径直驱。
- 不修改正常操作入口 `teach_panel.launch.py`；不在 `scripts/` 添加稳定运行时 wrapper（仅 setup/field 辅助）。
- 构建与代码风格遵循 AGENTS.md：`./scripts/build/build_workspace.sh`、`pathlib`/包 share 路径、不硬编码仓库相对路径、硬件标定值走参数。
- 不依赖 debug 工具（如 `depth_to_pointcloud`）；与 `taco_instances` 感知解耦（目标位置由调用方传入）。

---

## 7. 附录

### 附录 A：与 `hand_eye_calibration_next_steps.md` 的衔接

- 该文档「背景与目标」（L40）明确：视点规划的"期望相机位姿 → tool0 目标位姿"变换依赖可信的 `tool0→camera` 变换（由 URDF 固定链 + 相机外参合成，见 §1.2 frame 澄清），视点规划是手眼标定的下游用户。
- 当前外参为经验试调值（§1.2）。本文档实现后，外参一旦可信（阶段 B/C 完成），仅需更新相机外参参数（经 TF 生效，`viewpoint_planner` 实时解析 `tool0→camera`），即可提升视点绝对精度，**无需改代码**。

### 附录 B：相关文件索引

| 文件 | 角色 |
|---|---|
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | 目标定位现成代码；发布 `/arachne/perception/taco_instances`（`_publish_preview_detection_event` L3393-3433，`base_grasp_xyz` L3406） |
| `src/arachne_operator/arachne_operator/step_cleanup_demo.py` | 定位输出消费方先例（L59/L406/L457） |
| `src/arachne_operator/arachne_operator/real_hardware_acceptance_test.py` | `AuboI5Kinematics`（L45）/ `solve_pose`（L152），IK 复用来源 |
| `src/arachne_hardware/action/AuboMoveJoint.action` | 运动执行 action（本模块不调用） |
| `src/arachne_hardware/CMakeLists.txt` | 自定义接口生成先例（L4-12） |
| `src/arachne_operator/setup.py` / `src/arachne_sensors/package.xml` | ament_python 包先例 |
| `config/hand_eye_camera_extrinsics/current_effective.json` | 当前生效相机外参（经验值，`ee_camera_link→camera_color_optical_frame`，经 URDF 链+TF 合成 `tool0→camera`） |
| `src/arachne_sensors/launch/gemini335.launch.py` | 相机 launch；`camera_optical_*` 外参默认值（L195-201） |
| `src/arachne_description/urdf/sensors/ee_camera.xacro` | eye-in-hand 安装链（L33-62） |
| `docs/todo/hand_eye_calibration_next_steps.md` | 手眼标定主线；阶段 B/C（可信外参）与阶段 E（视点规划衔接） |
| `docs/todo/grasp_offset_calibration_next_steps.md` | 附录提及视点规划衔接（命名 `docs/todo/viewpoint_planning.md` 未创建；本文档即该衔接任务的正式规划文件） |

### 附录 C：备注

- 视点规划的目标位置语义：`base_grasp_xyz` 是"抓取点"而非"物体中心"，两者可能差一个抓取偏移。调用方若以物体中心为球心，应传入自行修正后的坐标；该语义差异由调用方负责，`viewpoint_planner` 只把 `target_xyz_base` 当球心使用。
