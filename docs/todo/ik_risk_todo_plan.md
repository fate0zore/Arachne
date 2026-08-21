# IK 实现风险评估与修复计划（ik_risk_todo_plan）

> 状态：**评估完成，暂不实施（2026-08-12 用户决定）**——已完成 IK 实现全链路梳理与 12 项风险评估（§2）；第三方依赖可行性评估完成（§3）；修复计划按性价比排定（§4），实施优先级与范围待用户后续确认
> 创建：2026-08-12
> 更新：2026-08-12
> 来源：2026-08-12 只读代码审查（IK 实现梳理 + 风险评估）+ 用户决策"暂不实施，仅保留评估"
> 目标读者：后续接手 IK 加固/改造的开发者；文档应可独立执行或续接

---

## 0. 结论摘要

- **IK 实现方式**：项目不依赖外部 IK 库（MoveIt 附带的 KDL 仅作 OMPL 规划兜底），核心是一份共享的**纯 numpy 阻尼最小二乘（DLS）数值 IK**——`AuboI5Kinematics`（`src/arachne_operator/arachne_operator/kinematics.py`），被 teach panel / 抓取规划 / 视点规划 / 验收测试四处复用。
- **风险评估结论**：12 项风险（§2），其中 4 项高风险（R1-R4）涉及真机安全/正确性，4 项中风险（R5-R8）+ 2 项中风险（R9-R10），2 项低风险（R11-R12）。
- **第三方依赖结论**：TRAC-IK 替换 KDL 可解决约一半风险（R3/R4/R8/R9，缓解 R1/R5/R10），但 R2（接受未收敛解）、R6（参数漂移）、R7（时间预算）、R12（权重不一致）是业务/架构问题，**换库无效、必须改代码**。
- **用户决策（2026-08-12）**：暂不实施，仅保留评估。本文档即评估结论的落盘，后续可从中任选 Phase 续接。

---

## 1. IK 实现现状（2026-08-12 只读审查结论，带 file:line）

### 1.1 核心：`AuboI5Kinematics`（唯一真正 IK 求解器）

`src/arachne_operator/arachne_operator/kinematics.py`：纯 numpy、无 ROS 依赖，从 `real_hardware_acceptance_test.py` 抽取共享。

- **FK**：手工按关节位姿链（`RevoluteJoint` 的 xyz/rpy/axis，L37-94）从 `aubo_base_link` 连乘到 `tool0`（含固定 `TOOL0_FIXED_RPY=(0,0,π/2)`，L96），`fk`（L98）。
- **数值雅可比**：对每关节加 `eps=1e-5` 有限差分（`_numeric_position_jacobian` L195 / `_numeric_pose_jacobian` L205）。
- **`solve_position`**（L120）：3 维位置 IK。
- **`solve_pose`**（L152）：6 维位姿 IK；姿态误差用旋转向量（`_rotation_vector` L237），`orientation_weight`（默认 0.5）权衡位置/姿态；迭代解 `lhs = J·Jᵀ + damping²·I` 再 `Jᵀ·solve`，每步 clip 到 `max_step`，关节 wrap 到 [-π,π]（`_wrap` L288）。
- **限位不内建**：数值 IK 不强制关节限位；`AUBO_I5_JOINT_LIMITS`（L27-34，名义限位，URDF 仅 ±2π）由调用方单独检查。

### 1.2 各使用方

| 模块 | 用途 | 方式 |
|---|---|---|
| `grasp_preview_pipeline.py` | 抓取规划 | `self.kinematics = AuboI5Kinematics()`（L857）；`_solve_tool0_goal_joints`（L4913，本地 IK + `orientation_weight=0.25`）；`local_first` 模式直接接受（L4599-4636）；否则转 MoveIt OMPL joint/pose goal；另有纯本地路径（L5259 附近，`--planner-backend local`） |
| `teach_panel.py` | 手动 jog / TCP / 速度 | 点位 jog 用 `solve_pose`（L3246/L3424）；速度级 IK `_joint_velocity_from_twist`（L4033，数值 twist 雅可比 + 阻尼求解 + 峰值限速） |
| `viewpoint_planner/ik_filter.py` | 视点可达性过滤 | `solve_pose` + `_within_limits` 限位检查（L114）；串行 seed（L81-112）+ `ik_time_budget_sec`（节点参数，`viewpoint_planner_node.py:76`） |
| `real_hardware_acceptance_test.py` | 真机验收 | `solve_position`（3 维，L308/L365） |
| `agent_bridge.py` | `arm_cartesian_jog` 工具 | 速度级阻尼 IK（L542-550，与 teach_panel 同套路） |
| `arachne_demo/gazebo_autopick_planner.py` | Gazebo 演示 | 独立实现的位置级阻尼 IK（`IK_DAMPING=0.08`/`IK_MAX_STEP=0.08`，L35-37；`_solve_ik_position` L500），**不共享** kinematics.py |

### 1.3 MoveIt 侧（KDL + OMPL）

`src/arachne_moveit_config/config/kinematics.yaml:2-5`：`kdl_kinematics_plugin/KDLKinematicsPlugin`（search_resolution 0.005 / timeout 0.05s / attempts 3）。MoveIt 在此是**路径规划器（OMPL）**而非 IK 主解：本地 `solve_pose` 失败时经 `_call_moveit_pose_plan`（L4944）退化为 pose-goal IK（`--moveit-pose-goal-on-ik-failure` 默认开），或把本地 IK 解作为 joint goal 交 OMPL 避障。

### 1.4 关键设计特征

1. 无解析 IK，全数值 DLS，`kinematics.py` 是唯一共享 IK 源；
2. 限位外置：数值 IK 不约束关节范围，`_within_limits` / 调用方各自查限位；
3. 连续性靠 seed 链：抓取序列与视点过滤都"上一解作下一 seed"；
4. 时间/预算防护：视点 `ik_time_budget_sec`，抓取 `moveit_service_timeout_padding` + 迭代上限；
5. 先验可达性粗筛：`_moveit_unreachable_note`（L1119）按 tool0 半径（球面）先拒超程目标。

---

## 2. 风险评估（2026-08-12，按严重程度分层）

### 🔴 高风险（真机异常运动 / 损坏可能）

| # | 风险 | 代码证据 | 后果 | 缓解建议 |
|---|---|---|---|---|
| **R1** | 数值 IK 不强制关节限位，且主要执行路径不检查 `AUBO_I5_JOINT_LIMITS` | `kinematics.py` 无限位逻辑（`ik_filter.py:46` 注释自认）；`grasp_preview_pipeline.py` 本地 IK（L5259）与 `_solve_tool0_goal_joints`（L4931）都无限位检查；`_generate_limited_joint_frames`（L5739）只 clip 速度/加速度/加加速度，不查位置限位 | IK 解出限位外关节角，SDK/ros2_control 拒绝或**截断** → 意外运动 | **最高优先级**：执行链补一处 `AUBO_I5_JOINT_LIMITS` 校验（复用 `ik_filter._within_limits` 逻辑），零依赖、改动极小 |
| **R2** | "未收敛解"可被接受执行 | `grasp_preview_pipeline.py:4567` `fallback_ok` 只查误差 + 无碰撞，**不要求 `ik_ok`**；`_moveit_fallback_tolerances`（L4868） | IK 跑满迭代未收敛，只要误差落 fallback 容差内，`best_q` 照样进执行链 | `fallback_ok` 加 `ik_ok` 门控或显式降级告警 |
| **R3** | 奇异位形 / 数值雅可比退化 | 全部分支 `eps=1e-5` 有限差分 + 固定 `damping=0.08`；速度 IK（`teach_panel.py:4033`、`agent_bridge.py:542`）同理 | 奇异附近解跳跃；速度 IK 峰值限速是**整体缩放**，保方向但改变实际笛卡尔速度 | TRAC-IK 替换（§3）可实质改善；或提高阻尼自适应性 |
| **R4** | FK/DH 参数手写，与 URDF 存在漂移风险 | `kinematics.py` `JOINTS` 常量（L57-94）与 `TOOL0_FIXED_RPY`（L96）手写；唯一自动化对照是 `test_ik_filter.py` 的 self-pose 自检 | URDF 更新后 IK 与真机失配 → IK 判定"可达"的位姿真机不可达/位姿错 | 加 FK↔TF/URDF 对照离线测试；或换 URDF 驱动的库（TRAC-IK） |

### 🟡 中风险（行为 / 可靠性）

| # | 风险 | 代码证据 | 后果 | 缓解建议 |
|---|---|---|---|---|
| **R5** | seed 链污染，无多 seed 重试 | `ik_filter.filter_poses`（L81）与抓取序列都"上一可达解作下一 seed" | 一次坏构型解（腕翻转/近限位）污染后续所有 pose | 多 seed / 候选重试 |
| **R6** | IK 参数多副本漂移 | `PREVIEW_IK_*`（`grasp_preview_pipeline.py:116-121`，iter=180、weight 0.5/0.25）vs `IkParams`（`ik_filter.py:19-30`，iter=100、weight 0.5）vs teach_panel `arm_ik_*` 参数 vs demo 常量 | **同一 tool0 pose 在视点规划判"不可达"、抓取规划判"可达"** 的行为不一致 | 收敛为单一参数来源（统一 `IkParams` 或全走参数） |
| **R7** | 时间预算耗尽 → 剩余 pose 全部误判不可达 | `ik_filter.py:103`（超预算直接标 unreachable）；`viewpoint_planner_node.py:76` `ik_time_budget_sec=12.0`；不可达 pose 每点跑满 100 次迭代（注释自述 0.6-0.9s/个） | 采样点多时服务超时风险；**可能误拒可达点**（保守方向正确） | 增大预算 / 提前终止策略 / 并行化 |
| **R8** | 验收路径与生产路径 IK 不一致 | `real_hardware_acceptance_test.py` 用 `solve_position`（3 维，L308/L365），生产走 `solve_pose`（6 维）/速度 IK | 验收通过 ≠ 生产 IK 路径可靠 | 统一 IK 入口 |
| **R9** | MoveIt 兜底很弱 | `kinematics.yaml:4-5` KDL timeout=0.05s / attempts=3；`_call_moveit_pose_plan` 的 `workspace_parameters` 硬编码（L4973-4976，base 系 corner） | pose-goal IK 易频繁失败，全靠 `--moveit-pose-goal-on-ik-failure`（默认开）兜底；workspace 与部署布局耦合，超界即失败 | TRAC-IK 替换（§3） |
| **R10** | 球面可达性粗筛只查半径不查姿态 | `_moveit_unreachable_note`（L1119）仅按 tool0 半径 vs `moveit_max_tool0_reach` 粗筛 | 参数偏大时接近极限的解被接受，真机执行近奇异/超速 | TRAC-IK 判定更快更准后可放宽/去掉粗筛 |

### 🟢 低风险 / 已知限制

| # | 风险 | 说明 |
|---|---|---|
| **R11** | `gazebo_autopick_planner.py` 是独立 IK 实现（58 次迭代、0.012m 阈值、HOME 吸引项） | 仅 Gazebo 演示用，但易造成"同一机械臂两套 IK"认知误导 |
| **R12** | `orientation_weight` 抓取用 0.25、视点用 0.5 | 姿态收敛优先级不同、误差分布不同（设计上有意，需知悉） |

---

## 3. 第三方依赖可行性评估（2026-08-12）

### 3.1 值得引入：TRAC-IK 替换 MoveIt 的 KDL

环境为 ROS 2 Humble + MoveIt，`kinematics.yaml:2` 一行切 `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` + `package.xml` 加 `trac_ik` 依赖，是 MoveIt 社区标准做法。

| 风险 | TRAC-IK 能否解决 | 说明 |
|---|---|---|
| R1 限位不强制 | 🟡 大幅缓解 | 内部 KDL 闭环 + SQP 数值双求解器，限位处理优于裸 DLS；仍建议保留最终限位校验 |
| R3 奇异/数值退化 | 🟢 解决 | SQP 求解器奇异鲁棒性远强于固定 damping=0.08 的 DLS |
| R4 FK 手写漂移 | 🟢 解决 | 从 **URDF 直接建模型**，FK 与 URDF 天然一致，手写 DH 副本可删 |
| R8 验收/生产不一致 | 🟢 解决 | 全链路统一走 TRAC-IK |
| R9 MoveIt 兜底弱 | 🟢 解决 | 成功率与鲁棒性显著提升 |
| R5 seed 污染 | 🟡 缓解 | 内部多 seed/随机重试；调用方连续 seed 模式仍建议保留多候选 |
| R10 球面粗筛 | 🟡 缓解 | 可达性判定更快更准，粗筛可放宽/去掉 |

### 3.2 不能解决（必须改代码）

| 风险 | 为什么库救不了 |
|---|---|
| R2 未收敛解被接受 | `fallback_ok`（L4567）是调用方业务逻辑；库只返回 success/fail |
| R6 参数多副本漂移 | 架构问题；换库后反而要删本地 IK 参数副本 |
| R7 时间预算误判 | 调度/超时策略，与求解器无关 |
| R12 权重不一致 | 设计决策 |

### 3.3 不推荐路线

- **IKFast**（解析解）：需针对 Aubo i5 重新生成 + 维护 MoveIt 定制插件，arm64 交叉编译麻烦；6 轴标准球腕收益有限。
- **pinocchio / ikpy**：pinocchio 数值质量高但引入新依赖树 + 自写 MoveIt 桥接；ikpy 仍是数值法、与现状同构。除非后续做笛卡尔速度级规划，否则不必。

### 3.4 现实约束

- AGENTS.md 禁止"为简单 stdlib/ROS 原生工作加依赖"——IK 不属于此类（自写数值 IK 已有实质复杂度），引入 TRAC-IK 在规则内，但属架构级改动，须走完整构建验证。
- Jetson arm64 + Humble：`trac_ik` 有 rosdep 条目，但 arm64 二进制不一定齐全，可能需源码编译（纯 C++/Eigen，可行，增加构建时间）。
- 换库后本地 `AuboI5Kinematics.solve_pose` 的快速离线/预览路径（视点过滤、teach jog）可保留或统一——保留则 R6 依旧，统一则失去"纯 numpy 无 ROS"轻量性，需权衡。

---

## 4. 修复计划（按性价比排定，每 Phase 独立可验收）

### Phase 1：零依赖限位检查（R1）——最高优先级，预计 <30 行

- **改动点**：
  1. `grasp_preview_pipeline.py`：`_solve_tool0_goal_joints`（L4931 后）与本地 IK 路径（L5259 后）各加一处 `AUBO_I5_JOINT_LIMITS` 校验，超限判 IK 失败/标记降级；
  2. 复用 `ik_filter._within_limits`（L114）逻辑或抽为 `kinematics.py` 公共方法（避免 import 方向依赖）。
- **验证**：py_compile + 抓取规划 dry-run/单测（构造超限解断言被拒）+ 全量构建；真机确认留待用户。
- **收益**：直接堵住硬限位缺口，零依赖、无架构改动。

### Phase 2：`fallback_ok` 门控（R2）

- **改动点**：`grasp_preview_pipeline.py:4567` 增加 `ik_ok` 要求，或把"未收敛但误差达标"显式记为降级模式并告警（`--allow-unconverged-ik-fallback` 参数显式开启）。
- **验证**：单测覆盖 ik_ok=False 且误差达标时的行为分支。

### Phase 3：IK 参数统一（R6）

- **改动点**：将 `PREVIEW_IK_*`（L116-121）、`IkParams`（ik_filter.py:19-30）、teach_panel `arm_ik_*` 参数收敛为单一来源（统一 `IkParams` 或全部走参数），消除"同一目标两处判定不一致"。
- **验证**：对比统一前后同一 tool0 pose 的判定一致性单测。

### Phase 4：FK ↔ URDF 对照测试（R4）

- **改动点**：新增离线测试：采样多组 q，对比 `AuboI5Kinematics.fk` 与 TF/URDF（`robot_state_publisher` 或 `kdl_parser` 加载 URDF 后的 FK）的 `tool0` 位姿，防手写 DH 静默漂移。
- **验证**：测试纳入常规单测，URDF 变更后 CI/本地可发现失配。

### Phase 5（可选，架构级）：TRAC-IK 替换（R3/R4/R8/R9/R10）

- **前置**：确认 Jetson arm64 上 `trac_ik` 可用性（rosdep / 源码编译）。
- **改动点**：`kinematics.yaml:2` 切 TRAC-IK + `package.xml` 加依赖 + 全量构建；本地 `AuboI5Kinematics` 保留作预览/降级（或按 §3.4 决策统一）。
- **验证**：抓取/视点/验收全链路回归 + 真机确认（用户明确确认机械臂就绪后）。

---

## 5. 验收标准（全部满足才算完成；当前状态：未实施）

- [ ]（Phase 1）IK 执行链超限解被拒绝：构造关节解越限用例断言不进入执行链；dry-run/headless 不触碰真机；
- [ ]（Phase 2）未收敛解默认不执行，降级路径有显式参数与告警；
- [ ]（Phase 3）IK 参数单一来源，同一 tool0 pose 在各调用方判定一致（单测断言）；
- [ ]（Phase 4）FK↔URDF 对照测试通过并纳入常规测试；
- [ ]（Phase 5，若实施）TRAC-IK 替换后全链路回归通过，`./scripts/build/build_workspace.sh` 构建通过；
- [ ] 相关 `--dry-run-check` 通过；`git status --short` 已检查；本文档状态回写。

---

## 6. 安全与流程约束（沿用 AGENTS.md）

- 真机运动/抓取验证前，用户明确确认机械臂就绪；本文档所有 Phase 的 dry-run/headless 路径不触碰真实硬件。
- 改动后经 `./scripts/build/build_workspace.sh` 重建；正常操作入口保持 `teach_panel.launch.py`。
- 不引入依赖除非 Phase 5 经用户确认；不加 shell 运行时 wrapper；不改与本次无关的标定参数。
- 所有 Phase 先"验证后生效"，不自动改写 launch/config；架构级改动（Phase 5）必须先与用户确认。

---

## 7. 附录

### 附录 A：相关文件索引

| 文件 | 角色 |
|---|---|
| `src/arachne_operator/arachne_operator/kinematics.py` | 核心 FK/IK：`AuboI5Kinematics`（`solve_position` L120 / `solve_pose` L152 / 数值雅可比 L195/L205 / `_wrap` L288）、`AUBO_I5_JOINT_LIMITS`（L27-34） |
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | 抓取规划主链路：`_solve_tool0_goal_joints`（L4913）、`fallback_ok`（L4567）、`_moveit_fallback_tolerances`（L4868）、`_call_moveit_pose_plan`（L4944）、`_moveit_unreachable_note`（L1119）、`_generate_limited_joint_frames`（L5739）、本地 IK 路径（L5259 附近）、`PREVIEW_IK_*`（L116-121） |
| `src/arachne_operator/arachne_operator/teach_panel.py` | 手动 jog `solve_pose`（L3246/L3424）、速度 IK `_joint_velocity_from_twist`（L4033）、`arm_ik_*` 参数（L619-621） |
| `src/arachne_operator/arachne_operator/real_hardware_acceptance_test.py` | 真机验收：`solve_position`（L308/L365）、`arm_ik_*` 参数（L74-76） |
| `src/arachne_agent_bridge/arachne_agent_bridge/agent_bridge.py` | `arm_cartesian_jog`：速度级阻尼 IK（L542-550）、`_numeric_twist_jacobian`（L552） |
| `src/viewpoint_planner/viewpoint_planner/ik_filter.py` | 视点 IK 过滤：`IkParams`（L19-30）、`check`（L62）、`filter_poses` 串行 seed + 预算（L81-112）、`_within_limits`（L114） |
| `src/viewpoint_planner/viewpoint_planner/viewpoint_planner_node.py` | `ik_time_budget_sec`（L76/L219） |
| `src/viewpoint_planner/test/test_ik_filter.py` | IK 过滤离线单测（含 self-pose 自检） |
| `src/arachne_moveit_config/config/kinematics.yaml` | MoveIt IK 配置：KDL（L2-5），TRAC-IK 替换点（Phase 5） |
| `src/arachne_demo/arachne_demo/gazebo_autopick_planner.py` | 独立 IK 实现（L35-37、L500），仅 Gazebo 演示 |
| `docs/todo/viewpoint_planning_todo_plan.md` | 视点规划任务（已实现）；其 §1.3/§2.4 记录 IK 复用与限位补充决策，与本文档 R1/R6 关联 |

### 附录 B：与既有 todo 文档的衔接

- `viewpoint_planning_todo_plan.md` §2.4 已记录"`solve_pose` 不强制关节限位 → `ik_reachable` 判定需额外校验限位"的设计决策，`ik_filter.py` 已落地该决策（`_within_limits`）；本文档 R1 指出**同一限位检查在 grasp_preview 执行链缺失**，属同源问题的另一侧。
- `hand_eye_calibration_next_steps.md` 涉及外参（影响 IK 目标位姿精度，非 IK 本身），不在本计划范围内。

### 附录 C：备注

- 本文档风险评估均基于 2026-08-12 静态代码审查，未做真机/仿真验证；R1/R3 的实际影响取决于 SDK 侧限位行为与奇异位形出现频率，实施前建议先做一次日志侧观察（grasp_preview 执行日志中的 IK 误差/迭代次数分布）。
