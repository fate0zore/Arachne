# 末端相机碰撞事故分析与修复评估（ee_camera_collision_analysis）

> 状态：**已实施 A+B（2026-08-13）**——真实事故（结构性损坏）已确认；根因已定位；本次修复向 teach_panel keepout 检查与 grasp_preview 碰撞模型补齐"前篮筐 keepout + 末端相机/支撑板偏心包络"（共享 `ee_collision.py`），已通过 dry-run、离线回归（含真实事故 #9 姿态）与用户真机复现（视点 #9 执行被拦截、机械臂未运动）。C（视点规划级过滤）/D（moveJoint server 兜底）未实施。
> 创建：2026-08-13
> 更新：2026-08-13
> 来源：2026-08-13 只读代码审查（视点执行链路 + 抓取碰撞模型梳理）+ 用户确认的事故场景（视点规划之后的执行阶段，机械臂下降时末端相机挂到后货架/篮筐沿，造成结构性损坏）
> 目标读者：后续接手末端碰撞防护的开发者；文档应可独立执行或续接

---

## 0. 结论摘要

- **事故**：视点规划之后的执行阶段，机械臂携带末端相机下降时，相机（侧向悬臂件）挂到后货架/篮筐沿，**结构性损坏**（用户确认 2026-08-13）。
- **根因（两层叠加）**：
  1. **碰撞模型末端偏心件几何缺失**：teach_panel 的 keepout 检查（`_arm_keepout_violation` → `_arm_keepout_samples`）只含后货架 box + 一个与真实安装不符的 `camera_envelope` 单点；`grasp_preview_pipeline._arm_collision_clearance_base` 的 FK 链止于 `tool0`，末端只追加一个"抓取点"。相机/支撑板侧向悬出 7.4 cm 无体积，"相机 vs keepout 沿"这个最危险的碰撞对永远不成立；
  2. **前篮筐 keepout 缺失**：teach_panel `_arm_keepout_boxes` 只有 `rear_rack_keepout`，篮筐侧（事故的"篮筐沿"）无保护。
- **修复前事实澄清**（相对初版分析）：teach_panel 视点执行链路**并非零碰撞检查**——`_move_arm_to_positions_velocity` 入口已有 `_arm_keepout_violation`；缺口在几何（缺前篮筐 box、相机偏心件建模错误），而非"没有检查"。
- **与 `ik_risk_todo_plan.md` 的关系**：本次问题**不属于 IK 求解缺陷**，而是"IK 可达但碰撞"这一类——恰好是 ik_risk_todo_plan（2026-08-12，用户决定暂不实施）覆盖范围之外的新风险；其中 `fallback_ok` 的"无碰撞"条件（R2）所依赖的正是本次揭示的残缺碰撞模型。
- **优先度**：**本次修复优先于 ik_risk_todo_plan 的全部 Phase**（详见 §6）。

---

## 1. 事故链路（用户确认场景，2026-08-13）

```
viewpoint_planner_node      "Only plans — never moves"（viewpoint_planner_node.py:3）
  └─ ik_filter.check        IK 求解 + _within_limits 限位（ik_filter.py:62/114）——无碰撞
teach_panel Viewpoint 页
  └─ _execute_viewpoint_selected（teach_panel.py:5748）
       只检查 row["ik_reachable"]（:5764）→ 弹确认框（:5771）→ 直接执行
  └─ move_arm_to_joints_degrees（:3351）
  └─ _move_arm_to_joints_worker（:3402）→ _move_arm_to_positions_velocity（:3447）——无碰撞
/arachne/aubo/move_joint
  └─ aubo_move_joint_action_server → SDK execute_move_joint（无任何碰撞/规划检查）
```

**关键事实：整条视点执行链路的碰撞检查次数 = 0。**
- `viewpoint_planner_node.py` 与 `strategies/*.py` 全文无 collision/obstacle/keepout 相关代码（grep 无结果）；
- `ik_filter.py` 只做 `solve_pose` 可达 + `_within_limits` 限位（ik_risk_todo_plan §1.2 已有记载）；
- teach_panel 视点执行仅检查 `ik_reachable` 后直接发关节角；
- `aubo_move_joint_action_server.py` 是 SDK `moveJoint` 的 action 包装，无碰撞查询。

## 2. 末端真实几何（URDF）——为什么挂的是相机

`src/arachne_description/urdf/sensors/ee_camera.xacro`：

| 件 | 碰撞几何 | 位置 |
|---|---|---|
| `ee_camera_support_link` | box `0.063×0.113×0.005`（:17-22） | tool0 附近薄板 |
| `ee_camera_link`（Gemini） | box `0.030×0.089×0.025`（:44-49） | `camera_xyz=(0, -0.0741, 0.005)`（:33），**侧向悬出 7.4 cm** |

- 安装旋转 `ee_support_rpy=2.356 rad`（135°，`arachne.urdf.xacro:6`）→ 相机中心在 tool0 系约 `(0.052, 0.052, 0.005)`，加上本体半长，**最外缘离工具轴线 >12 cm**；
- 相机是末端唯一的大体积**侧向偏心**件（夹爪在轴线上），wrist2/wrist3 转动 + 下降时扫掠半径最大；
- 真实沿高：前篮筐碰撞 box 顶面 z≈0.0155 m（`front_basket.xacro:14-19`）；后货架 box z 范围 0.155~0.777 m（`rear_rack.xacro:17-22`）。

## 3. 现有抓取碰撞模型对末端的建模缺陷（即使接入也漏检）

`grasp_preview_pipeline.py` 的 `_arm_collision_clearance_base`（:5675）：

```python
link_points = [机械臂 link 原点 ...]          # FK 链止于 tool0（kinematics.py:105-114）
tool_to_grasp = self._tool_to_grasp_matrix()  # :4080
link_points.append(抓取点)                    # 只追加一个点，无体积
```

- `link_transforms` 只到 `tool0`，`ee_camera_support_link` / `ee_camera_link` / `ms42dc_*` 不在 FK 链上；
- 末端唯一碰撞体是 `tool_to_grasp` 返回的**一个点**（默认沿 tool0 z 平移，或 TF 的 `END_EFFECTOR_FRAME` 抓取点）——**方向是抓取轴线，与相机方向不一致**；
- 相机 box、支撑板、夹爪 STL **零建模**；
- `_joint_segment_collision_clearance_base`（:4842）、`tool_ground_hit`（:4520 附近）、`_basket_path_safe`（:3862，只查底盘路径）同样不含末端偏心体积。

**结论**：障碍物侧（篮筐/货架）keepout 建模是**保守的**（`basket_keepout` 顶面 z=0.0635，比真实沿高 4.8 cm，`grasp_preview_pipeline.py:449-452`），但机械臂侧的末端偏心件在数学上不存在——"相机 vs 沿"这个最危险的碰撞对永远不成立。

## 4. 与 ik_risk_todo_plan.md 的关系

- 本次问题**不是 IK 求解缺陷**：`AuboI5Kinematics` / KDL 只解运动学方程，"解得出"与"不碰撞"是两件事；
- ik_risk_todo_plan 的 12 项风险（R1-R12）全部围绕 IK 求解本身，**没有覆盖"IK 可达但碰撞"这一类**；
- 最接近的关联：R2 中 `fallback_ok`（`grasp_preview_pipeline.py:4567`）的"无碰撞"条件所调用的正是 §3 的残缺模型——本次分析是 R2 底层缺陷的补充证据；
- 视点执行链路零碰撞检查是**独立于 ik_risk_todo_plan 的新发现**（原文档未涉及 teach_panel 视点执行路径）。

## 5. 优先度分析（本次修复 vs ik_risk_todo_plan Phase 1-5）

| 维度 | 本次（末端相机碰撞） | ik_risk_todo_plan Phase 1-5 |
|---|---|---|
| 事故性质 | **真实事故 + 结构性损坏**（已发生损失） | 潜在风险（评估完成、用户决定暂不实施） |
| 影响面 | 末端硬件损坏、停机、维修成本 | 可能引发异常运动，尚未实机触发 |
| 修复成本 | 局部：末端几何包络 + 视点链路接入检查（预计 <100 行，无新依赖） | Phase 1 <30 行；Phase 5 架构级（TRAC-IK） |
| 修复与 ik Phase 的关系 | 与 Phase 1（限位）**互补不冲突**，可并行 | — |

**结论**：
1. **本次修复优先级最高**（紧急）：已造成结构损坏，且修复是局部改动，性价比高于 ik_risk_todo_plan 任一 Phase；
2. ik_risk_todo_plan 的 **Phase 1（R1 限位检查）建议紧随其后**：同一执行链（视点/抓取）上最便宜的硬安全缺口；
3. Phase 2-5（收敛门控、参数统一、FK 对照、TRAC-IK）维持原计划节奏，不受本次影响；
4. 本次修复**不能替代** ik_risk_todo_plan：堵住碰撞漏检 ≠ 解决 IK 求解鲁棒性。

## 6. 修复方向（A/B 已实施，C/D 未实施）

- **A. 末端偏心包络（已实施，治本）**：新增 `arachne_operator/ee_collision.py`（纯 numpy 共享模块），把相机/支撑板相对 tool0 的旋转后 AABB 包络（URDF 推导：support 中心 ≈(-0.0177,-0.0177,0.0025) 半尺寸 (0.0622,0.0622,0.0025)；camera 中心 ≈(0.0488,0.0488,0.0175) 半尺寸 (0.0421,0.0421,0.0125)，默认 margin 0.01）与采样点（8 角点 + 6 面心）参数化导出；
- **B. 视点执行链路接入检查（已实施，补缺口）**：teach_panel `_arm_keepout_boxes` 新增前篮筐 keepout（参数 `front_basket_keepout_min/max_xyz`，默认与 grasp_preview `--basket-keepout-*-base` 一致）；`_arm_keepout_samples` 用 `ee_envelope_points_tool0` 替换错误单点 `camera_envelope=(0.025,-0.090,0.050)`（与真实相机位置不符）；grasp_preview `_arm_collision_clearance_base` 追加末端包络独立点检查（`--ee-camera-collision-enabled` 默认 true、`--ee-camera-collision-margin` 默认 0.01）；
- **C. 视点规划级过滤（未实施，预防）**：viewpoint_planner 在 IK 过滤后增加视点级碰撞过滤（需在 planner 侧引入障碍物几何）；
- **D. 执行端兜底（未实施）**：`aubo_move_joint_action_server` 执行前加一次碰撞预检。

## 7. 待用户确认项（实施后更新）

- [x] 修复范围：已确认 **A+B**（教面板 + 抓取模型同补），C/D 暂不实施；
- [x] 是否与 ik_risk_todo_plan Phase 1 同步：已确认**本次只修碰撞**；
- [ ] 事故视点的具体行/姿态（如有 teach_panel 日志 `viewpoint_execute` 事件可反查）——**用户表示稍后提供**，用于真机/离线回归确认该视点被拦截；
- [ ] 真机验证：机械臂就绪并用户确认后执行（dry-run/离线验证已通过）。

## 8. 安全与流程约束（沿用 AGENTS.md）

- 本文档在实施前**不修改任何项目代码**；所有修复 Phase 实施前须经用户确认；
- 真机运动/验证前，用户明确确认机械臂就绪；dry-run/headless 路径不触碰真实硬件；
- 改动后经 `./scripts/build/build_workspace.sh` 重建；正常操作入口保持 `teach_panel.launch.py`。

## 8.1 实施记录（2026-08-13，A+B）

改动文件：

| 文件 | 改动 |
|---|---|
| `src/arachne_operator/arachne_operator/ee_collision.py` | **新增**：URDF 推导的末端相机/支撑板 tool0 系 AABB 包络 + 采样点（纯 numpy） |
| `src/arachne_operator/arachne_operator/teach_panel.py` | `_arm_keepout_boxes` 新增前篮筐 keepout；`_arm_keepout_samples` 用共享包络点替换错误 `camera_envelope` 单点；新增参数 `front_basket_keepout_min/max_xyz`、`arm_keepout_ee_margin_m`；safety teach config 读写同步 |
| `src/arachne_operator/launch/teach_panel.launch.py` | 透传 `front_basket_keepout_min/max_xyz` 参数（默认与 grasp_preview basket_keepout 一致） |
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | `_arm_collision_clearance_base` 追加末端包络独立点检查（重构为 `check_point` 闭包）；新增 `--ee-camera-collision-enabled/margin` 参数 |

验证：

- `./scripts/build/build_workspace.sh` 全量构建通过（17 packages，两次：初版点采样 + 盒判定升级）；
- `ros2 run arachne_operator teach_panel --headless-check`、`grasp_task_server --dry-run-check`、`arachne check entrypoints` 全部通过；
- 离线回归（合成 + 真实事故姿态，真机未动）：
  - **真实事故视点 #9**（`log/teach_panel/20260812_153349`，`viewpoint_execute` 15:43:36，joints `-7.10,-44.45,156.95,80.25,28.24,-10.61` deg）：teach_panel 与 grasp_preview 均拦截（`ee_camera_support` 盒斜插 front basket keepout，渗透约 2.2 cm）；
  - 同会话已执行且未出事的视点 #0/#1/#2/#4/#6：**全部放行，无误报**；
  - 合成姿态（front_basket 相机专属、rear_rack、仅相机包络触发）均拦截；home 放行。

**升级记录（盒判定）**：初版末端包络用 14 采样点（8 角点 + 6 面心）检查，真实事故 #9 下 `ee_camera_support` AABB 与篮筐 keepout 相交但 14 个采样点无一落入——**点采样对斜向穿透漏检**（事故即此形态）。升级为盒-盒判定：teach_panel `_arm_keepout_ee_box_violation` 与 grasp_preview `_box_box_clearance` 直接做 AABB 相交/距离判定，点采样仅保留用于机械臂 link 段与夹爪 tool 点。

- 回归用合成姿态（供后续参考）：
  - front_basket（相机专属）：`36.654,116.488,-102.502,-95.123,31.505,-164.202`（deg）
  - rear_rack：`-130.844,-10.800,-71.486,-31.053,11.281,66.101`（deg）

**真机复现（2026-08-13，用户操作，机械臂未发生运动）**：teach_panel 正常入口复现事故场景，日志 `log/teach_panel/20260813_165504/`：

- 16:55:46 `viewpoint_plan_button`：参数与事故当天一致（target `(0.9, 0.0, -0.2)`、radius 0.5、32 视点、filter_ik=True）；
- 16:56:09 `viewpoint_plan_result`：**7/32 ik-reachable，与事故当天（20260812_153349）完全一致**；新规划文件 `viewpoint_plan_20260813_165609_446.json` 中 #9 的 `ik_reachable=true`、`ik_joints` 与事故当天逐位一致（IK seed 相同）；
- 17:01:35 `viewpoint_execute #9` → 17:01:36.012 被拦截：
  `arm target blocked by safety zone: joint target enters front basket keepout: tool_envelope_x- at base_link xyz=(0.551,0.108,0.058), path sample 20/25`，23 ms 后 `viewpoint execute finished`；
- 拦截路径：从当前姿态插值到 #9 的路径第 20/25 采样点，工具包络点 `tool_envelope_x-`（tool0 系 (-0.070,0,0.090)）进入前篮筐 keepout（y=0.108，距沿 0.11 m 仅 2 mm）——对应事故"下降途中挂到篮筐沿"；端点 #9 另有相机/支撑板盒判定兜底（见上）。

**结论**：`ik_reachable=true`（json）只表示 IK 可解 + 限位内，碰撞检查在执行阶段由 teach_panel `_arm_keepout_violation` 完成——复现时"json 可达但执行不成功"正是修复后的预期行为，且路径中途拦截先于端点盒判定触发（两层保护叠加）。

---

## 附录 A：相关文件索引

| 文件 | 角色 |
|---|---|
| `src/arachne_operator/arachne_operator/teach_panel.py` | 视点执行入口：`_execute_viewpoint_selected`（L5748）、`move_arm_to_joints_degrees`（L3351）、`_move_arm_to_joints_worker`（L3402）、`_move_arm_to_positions_velocity`（L3447） |
| `src/viewpoint_planner/viewpoint_planner/viewpoint_planner_node.py` | 视点规划节点（L3 注释 "Only plans — never moves"），无碰撞逻辑 |
| `src/viewpoint_planner/viewpoint_planner/ik_filter.py` | IK 可达过滤：`check`（L62）、`_within_limits`（L114），无碰撞逻辑 |
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | 抓取碰撞模型：`_make_collision_boxes`（L1023）、`_arm_collision_clearance_base`（L5675）、`_joint_segment_collision_clearance_base`（L4842）、`_tool_to_grasp_matrix`（L4080）、keepout 参数（L449-452） |
| `src/arachne_operator/arachne_operator/kinematics.py` | FK/IK：`link_transforms`（L105-114，止于 tool0） |
| `src/arachne_description/urdf/sensors/ee_camera.xacro` | 末端相机几何：support box（L17-22）、相机 box + 悬臂偏移（L33/L44-49） |
| `src/arachne_description/urdf/accessories/front_basket.xacro` | 前篮筐碰撞 box（L14-19） |
| `src/arachne_description/urdf/sensors/rear_rack.xacro` | 后货架碰撞 box（L17-22） |
| `src/arachne_hardware/arachne_hardware/aubo_move_joint_action_server.py` | SDK moveJoint action 包装，无碰撞检查 |
| `docs/todo/ik_risk_todo_plan.md` | IK 求解风险评估（R2 与本文档 §3 关联）；Phase 1-5 见 §6 对比 |

## 附录 B：备注

- 本文档基于 2026-08-13 静态代码审查 + 用户确认的事故场景，未做真机/仿真复现；
- §3 的几何推导（相机相对 tool0 偏移）来自 URDF 静态变换，未实测验证；修复前建议用 RViz（`moveit_grasp_demo.rviz` 已开 `Collision Enabled: true`）核对末端包络与实际硬件。

---

## 9. 视点执行链路改走 MoveIt 规划（方案 A，自碰撞修复）

> 状态：**已实施并真机验证通过（2026-08-21，用户确认机械臂恢复正常移动）**。
> 全链路：teach_panel 自轮询 aubo 关节角 → 完整 12 关节状态喂给 move_group → MoveIt(OMPL)
> 规划（自碰撞安全）→ 轨迹抽稀 → SDK moveJoint 逐 waypoint 执行。真机部署层问题与修复史见
> §9.11~9.15，最终验证记录见 §9.16。
> 来源：2026-08-21 日志分析——`log/teach_panel/20260821_104352` 最后一次机械臂操作（11:04:41 **视点 #7**）发生**两节手臂互相碰撞（机械臂自身 link 间自碰撞）**。属上一节 A/B 修复（末端相机 vs 篮筐/货架）**未覆盖的新类别**。
> 用户已选定 **方案 A**：把 teach_panel 的视点执行从"裸 SDK 速度控制"改走 **MoveIt（OMPL）碰撞检查通道**。
> 集成方式已定：**选项 1 = 共享 `moveit_plan.py` + grasp/teach_panel 两端薄接入**（§9.4）。实施记录见 §9.10。
> 目标读者：后续接手/验证本方案的开发者。

### 9.0 事故与根因回顾（2026-08-21）

- 会话 `log/teach_panel/20260821_104352`，规划文件 `viewpoint_plan_20260821_104537_934.json`（target (0.7,0,0)，radius 0.3，16 视点/8 ik 可达/6 碰撞自由）。
- 视点 #7 目标关节 `[20.2, 44.2, -108.8, 21.6, -42.4, 49.0] deg`，规划器给出 `collision_free=true` 但 **`collision_clearance` 仅 0.024 m**（半径 0.018、margin 0）——终点自碰撞余量几乎为零。
- 上一视点 #6 `[-28.9, -42.7, 97.9, -24.4, 47.0, 34.7]` → #7 需关节3 反向折叠 206.7°；执行走 `_move_arm_to_positions_velocity` 的**比例速度关节插值，无路径/自碰撞检查**，中途前臂与上臂相碰。
- 叠加因素：运动期间多次 `aubo sdk speedJoint busy; stopping previous motion and retrying`（11:04:49/11:04:52）→ SDK 停/重启导致实际轨迹去同步、比理想插值更危险；12 s 后 `arm velocity target timeout`，从未确认到达，佐证中途被碰撞卡停。
- **根因一句话**：视点执行只验证"端点碰撞自由"，且用不带任何自碰撞检查的裸速度运动走完整条关节路径。

### 9.1 为什么选 MoveIt（官方标准做法）

- 仓库已具备官方自碰撞检查通道：`src/arachne_moveit_config/launch/moveit_planning.launch.py` 启动 `move_group`（group=`aubo_arm`，OMPL），MoveIt 规划阶段做**完整碰撞检查（含自碰撞 + 环境碰撞）**。
- 抓取链路 `grasp_preview_pipeline.py` 已接入同一通道：`GetMotionPlan` 客户端调 `/plan_kinematic_path`（`--moveit-plan-service`、`--planner-backend moveit`、`_call_moveit_plan`/`_call_moveit_pose_plan`，L4950/5036）。
- 低速 SDK 本身**没有**对用户开放的"主动自避障/路径自碰撞规划"API；裸 `speedJoint`/`moveJoint` 只有被动碰撞检测 + 限位，无法预防本条路径自碰。MoveIt 是官方生态里唯一能"在动之前排除自碰路径"的机制。

### 9.2 关键代码事实（实施依据）

| 项 | 现状 | 来源 |
|---|---|---|
| 规划服务 | `GetMotionPlan` → `/plan_kinematic_path` | grasp_preview L496/961/4956 |
| 规划组/关节名 | 组 `aubo_arm`；`aubo_shoulder_joint, aubo_upperArm_joint, aubo_foreArm_joint, aubo_wrist1_joint, aubo_wrist2_joint, aubo_wrist3_joint` | `arachne_ms42dc.srdf.xacro`、`kinematics.py` JOINTS |
| teach_panel 现有执行客户端 | `arm_action_client`（`FollowJointTrajectory` → `/joint_trajectory_controller/follow_joint_trajectory`），已在回放链路使用 | teach_panel L1017-1020、L4543 |
| teach_panel 当前视点执行 | `_execute_viewpoint_selected` → `move_arm_to_joints_degrees` → `_move_arm_to_positions_velocity`（裸速度） | teach_panel L3804/3849/6284 |
| teach_panel 现有 keepout | `_arm_keepout_violation`（篮筐/货架 + 末端相机盒），**只查环境不查自碰撞** | teach_panel L5150 |
| 关节命名差异 | teach_panel 命令名**无 `aubo_` 前缀**（`shoulder_joint`…）；MoveIt/URDF 为**有前缀** | teach_panel L56、kinematics L57 |
| 当前关节状态 | teach_panel 可从 `/joint_states` 读当前 6 关节角 | teach_panel L1316/L1663 |

### 9.3 目标行为

视点执行时优先走 MoveIt 规划出一条**全程自碰撞 + 环境碰撞自由**的关节轨迹再动臂：

```
_execute_viewpoint_selected（保持确认框）
  └─ move_arm_to_joints_degrees
       └─ [新增] 先走 MoveIt（若可用）
            1. 读当前关节 q_start（aubo_* 顺序）
            2. 用 aubo_* 组名/关节名构造 GetMotionPlan（joint goal = 目标关节，
               与回放/抓取一致的 start_state + workspace + 速度/加速缩放）
            3. 服务返回 motion_plan_response.trajectory.joint_trajectory
            4. 把轨迹关节名映射回 teach_panel 命令关节名，经 arm_action_client
               (FollowJointTrajectory) 发送 → 等完成/到达
       └─ [回退/兜底] MoveIt 不可用/规划失败时：
            - 若可明确判定"低自碰撞风险"，才退回现速度路径；
            - 否则拒绝执行（安全优先），并在状态栏给出原因。
       └─ 现有 _arm_keepout_violation 保留为环境（篮筐/货架）兜底闸，两者叠加。
```

**分层逻辑**：
- **MoveIt** = 自碰撞 + 环境（规划场景内）的主防线；
- **teach_panel `_arm_keepout_violation`** = 末端相机/篮筐/货架的环境兜底（保留，不删）；
- **速度回退仅为降级容错**，且须有"低风险判定"门控，默认更倾向拒绝而非裸跑。

### 9.4 规划改动点（待实施，均最小面；**已定方案：选项 1 = 共享 `moveit_plan.py` + 两端薄接入**）

> 背景：`GetMotionPlan` 调用当前只存在于 grasp 节点（`grasp_preview_pipeline.py` L961 客户端 + L5033 `_call_moveit_plan` 私有方法），是类内私有辅助、未暴露成可调用服务；teach_panel 是独立节点，现没有任何 MoveIt 客户端，其 `arm_action_client` 只是 FollowJointTrajectory 执行器（能发轨迹、不能规划）。因此必须新增客户端，但**规划逻辑复用共享模块、不复制为两套**。

1. **新增共享模块 `src/arachne_operator/arachne_operator/moveit_plan.py`**（纯 RCL/URDF 无关构造，仿 `ee_collision.py` 共享几何的惯例）
   - `build_joint_goal_motion_request(GetMotionPlan.Request, q_start, q_goal, *, group_name='aubo_arm', planner_id, pipeline_id='ompl', planning_time, planning_attempts, workspace, velocity_scale, accel_scale, joint_tolerance, joint_names=aubo_* 顺序)`：构造 joint-goal 规划请求（start_state=当前关节、goal_constraints=JointConstraint、workspace 与 grasp_preview L4956-4986 一致）；
   - `call_moveit_joint_plan(client, request, timeout_sec) -> (ok, motion_plan_response, msg)`：`call_async` + 轮询 + 超时 + `error_code==1` 判定（贴近 grasp L4998-5017）；
   - 关节名/限位直接引用 `kinematics.JOINTS`（`aubo_*`），避免另写命名表。

2. **grasp_preview_pipeline.py（最小改动，防漂移）**
   - `_call_moveit_plan` 改为改调共享 `build_joint_goal_motion_request` + `call_moveit_joint_plan`（joint goal 路径）；`_call_moveit_pose_plan` 保留（pose goal 属抓取特定，不强行共用）。
   - 目标：grasp 与 teach_panel 的 joint-goal 规划构造走同一份代码。

3. **teach_panel.py**
   - 新增 `GetMotionPlan` 服务客户端（`create_client(GetMotionPlan, moveit_plan_service)`，默认 `/plan_kinematic_path`）。
   - `move_arm_to_joints_degrees`（或独立 `_move_arm_to_joints_moveit`）入口先试 MoveIt：读当前关节 → 调共享 `call_moveit_joint_plan`。
   - 新增发送轨迹到 `arm_action_client` 的路径（复用/贴近现有 `_send_arm_positions` 的 action 分支：`send_goal_async` → 等完成 → `_wait_for_arm_target`），并做 **`aubo_*` → 命令关节名映射**。
   - 新增参数（见 §9.6）；回退门控：MoveIt 规划失败时按"自碰撞风险低"才走原速度路径。
   - 在 `_execute_viewpoint_selected` 驳回条件下补充"规划失败/拒绝"提示位（不影响 UI 状态机，`_viewpoint_execute_done` 照常恢复 UI）。

4. **teach_panel.launch.py**：透传新增 `moveit_plan_service`、`moveit_*` 相关参数（默认值）；**move_group 的启动策略见 §9.5 待确认项**。

### 9.5 待确认项（2026-08-21 已拍板，均取推荐值）

- [x] **move_group 的常驻方式 = 选项 1（教面板自带）**：在 teach_panel.launch 里按条件拉起 `moveit_planning.launch.py`（`launch_rviz:=false`）；若服务不可用则教面板回退（见回退门控一项）。
- [x] **回退门控 = 规划失败即拒绝（保守）**：MoveIt 不可用/规划失败 → 视点执行**拒绝**并在状态栏给原因；不静默回退到裸速度。裸速度仅保留在**手动 jog/manual stream**（操作员实时看得到），批量/回放式执行不做无声降级。
- [x] **关节命名映射 = 先映射后发送**：`/joint_trajectory_controller/follow_joint_trajectory` 现有执行路径（grasp `REAL_ARM_JOINT_NAMES`、teach_panel `arm_command_joint_names`）都用**无前缀名**（`shoulder_joint`…）；MoveIt 返回 `aubo_*` 轨迹，发送前映射回无前缀名。
- [x] **规划场景 = 维持分层**：MoveIt 管自碰撞；teach_panel `_arm_keepout_violation` 管环境（篮筐/货架/末端相机）。不向 move_group 注入环境物体（改动小）。
- 注：裸速度回退仍保留的入口仅限**手动 jog/stream**（实时受操作员控制）；视点 Execute / 回放 / 预设位批量运动一律走 MoveIt，失败即拒。

### 9.6 新增参数（已落地，teach_panel.py）

| 参数 | 默认 | 说明 |
|---|---|---|
| `moveit_plan_service` | `/plan_kinematic_path` | MoveIt 规划服务 |
| `moveit_plan_enabled` | true | 是否启用 MoveIt 路径（关闭则回退裸速度路径） |
| `moveit_refuse_on_plan_failure` | true | 规划失败即拒绝（不回退裸速度），默认开启 |
| `moveit_planning_time` | 2.0 (s) | 规划时限 |
| `moveit_planning_attempts` | 3 | 规划尝试次数 |
| `moveit_planner` | RRTConnect | OMPL 规划器 |
| `moveit_joint_goal_tolerance` | 0.025 (rad) | joint goal 容差 |
| `moveit_velocity_scale` / `moveit_accel_scale` | 0.5 / 0.5 | 轨迹速度/加速缩放 |
| `moveit_plan_timeout_padding` | 0.25 (s) | 规划服务超时 padding |

执行语义：`_move_arm_to_joints_worker` 内先走 `_move_arm_to_joints_moveit`（MoveIt 规划 +
FollowJointTrajectory 执行 + 到达确认）；失败时若 `moveit_refuse_on_plan_failure` 为真则返回
False（拒绝，不裸跑），仅手动 jog/stream 保留裸速度。

### 9.7 验证清单（实施后执行）

- 构建：`./scripts/build/build_workspace.sh`；`python3 -m py_compile` 相关文件；`ros2 run arachne_operator teach_panel --headless-check`、`arachne check entrypoints`。
- 离线回归（真机不动）：
  - 事故链路 6→7 `[-28.9,-42.7,97.9,-24.4,47.0,34.7]` → `[20.2,44.2,-108.8,21.6,-42.4,49.0]`：真实 move_group 下
    MoveIt 规划**成功给出 101 点自碰撞自由路径**（不再走裸速度直线，事故不复现）；
  - 安全视点 #1：规划成功，无误报；
  - dry-run/headless 路径不触碰真实硬件。
- 真机验证（2026-08-21 已通过，用户确认机械臂恢复正常移动）：
  - 视点 Execute 全链路实动：move_group 就绪 -> MoveIt 规划 -> SDK moveJoint 逐 waypoint；
  - **执行通道是 SDK moveJoint，不是 FollowJointTrajectory**（本部署无 ros2_control/`use_aubo_ros2_driver:=false`）；
  - `aubo_*` 轨迹关节名 → 无前缀命令关节名映射、逐点到达校验在真机执行中正常；
  - 抽稀默认 `moveit_waypoint_joint_step_rad=0.35`（约 20°）：典型视点移动 5~6 步（用户要求），
    极端 206° 折叠约 11 步；可参数微调（调大更快更少步、调小更贴路径）。
- `git status --short` 检查。

### 9.8 风险与开放问题

- **规划失败 ≠ 假阳性**：MoveIt 找不到合法路径时应以拒绝/提示为主，避免静默回退到裸速度（除非低风险判定成立）。回退门控是本次最关键的安全决策点。
- **MoveIt 场景 ≠ keepout**：若 move_group 规划场景没有篮筐/货架/末端相机碰撞体，MoveIt 只管"自碰撞"，环境碰撞仍靠 `_arm_keepout_violation`；两者需同时在线，缺一不可。
- **执行不等于规划**：MoveIt 只保证规划轨迹无碰撞；本部署实际执行经 SDK moveJoint 逐 waypoint
  （无 ros2_control），相邻 waypoint 间 moveJoint 直线插值的切角由抽稀步长决定——步长越大越偏离
  规划路径，极端折叠（206°）不宜贪快；调参时以实际安全性为准。
- **真机待观察项（已不复现事故，但保留判断）**：MoveIt 与真实控制器姿态一致性、规划→执行延时与
  教面板 UI 等待态交互、moveJoint 逐点节奏对操作体验的影响——随正常使用持续观察，必要时调
  `moveit_waypoint_joint_step_rad` 或改造为 SDK movePath 合并连续段。

### 9.9 与既有修复的关系

- 不取代 §6 A/B（末端相机 vs 篮筐/货架 keepout）——A/B 继续管"环境碰撞"；
- 与 ik_risk_todo_plan（IK 求解鲁棒性）互补，不交叉；
- 本方案补齐的是"手臂 link 间自碰撞"这一类，目前在 teach_panel 视点执行链路上**没有任何检查**，是独立缺口。


---

## 9.10 实施记录（2026-08-21，方案 A + 选项 1）

用户已确认四项决策（§9.5）并按推荐实施。改动文件：

| 文件 | 改动 |
|---|---|
| `src/arachne_operator/arachne_operator/moveit_plan.py` | **新增**共享模块：`build_joint_goal_constraint`、`build_joint_goal_motion_request`（joint-goal 规划请求构造，aubo_* 名）、`call_moveit_joint_plan`（call_async+轮询+error_code 判定）。纯构造，不含运动学，两端共用防漂移。 |
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | `_call_moveit_plan`（joint-goal 路径）改调共享 `build_joint_goal_motion_request` + `call_moveit_joint_plan`（保留原 workspace/超时/路径约束语义）；删除已下沉的 `_moveit_joint_goal_constraint`；`_call_moveit_pose_plan` 保留不动。 |
| `src/arachne_operator/arachne_operator/teach_panel.py` | 新增 `GetMotionPlan` 客户端；新增参数（§9.6）；`_move_arm_to_joints_worker` 先走 `_move_arm_to_joints_moveit`；新增 `_move_arm_to_joints_moveit`（MoveIt 规划 + 发送 `arm_action_client`）+ `_send_moveit_trajectory`（aubo_* → 无前缀命令关节名映射 + FollowJointTrajectory 执行 + `_wait_for_arm_target`）；规划失败默认拒绝（`moveit_refuse_on_plan_failure=true`）。 |
| `src/arachne_operator/launch/teach_panel.launch.py` | 新增 `with_moveit_planning`（默认 true）按条件拉起 `moveit_planning.launch.py`（`launch_rviz:=false`、`with_robot_state_publisher:=false`，joint 状态走 teach 同一 topic）；透传 `moveit_plan_service`；新增 launch 参数。 |
| `src/arachne_operator/package.xml` | 新增 `exec_depend`：`moveit_msgs`、`arachne_moveit_config`。 |

验证（桌面/dry-run，真机未动）：

- `python3 -m py_compile`（moveit_plan.py / teach_panel.py / grasp_preview_pipeline.py / teach_panel.launch.py）通过；
- `colcon build --packages-select arachne_operator` 通过；
- `ros2 run arachne_operator teach_panel --headless-check` 通过（含新模块导入）；
- `ros2 run arachne_operator grasp_task_server --dry-run-check` 通过；
- `ros2 run arachne_operator arachne check entrypoints` 通过；
- 共享 builder 离线单测：构造 joint-goal 请求（group/planner/tolerance/start/goal 关节名）符合预期。

**待真机/运行时验证（需用户确认机械臂就绪）**：

- [ ] `moveit_planning.launch.py` 随 teach_panel 正常拉起，`/plan_kinematic_path` 可见；
- [ ] MoveIt 规划场景能跟踪当前臂姿态（joint 状态 topic 贯通）；
- [ ] 复现事故视点 #7（自 #6 `[-28.9,-42.7,97.9,-24.4,47.0,34.7]` → #7 `[20.2,44.2,-108.8,21.6,-42.4,49.0]`）：MoveIt 应规划失败/给安全轨迹，不复现裸速度自碰；
- [ ] FollowJointTrajectory 执行、`aubo_*`→无前缀名映射、到达判定正确；
- [ ] home/预设/关节→目标等批量入口在新路径下正常。

### 9.11 实施补记（2026-08-21，真机首跑暴露的两个部署问题）

用户真机首跑：视点规划正常、Execute 全部 `MoveIt plan service unavailable`、机械臂不动。排查并定位两个问题：

1. **move_group 配置崩溃（既有缺陷）**：`arachne_moveit_config/config/ompl_planning.yaml` 的 `ompl.request_adapters`/`response_adapters` 是 YAML 列表，MoveIt2(humble) 期望 **string**，move_group 启动即 `InvalidParameterTypeException` 崩溃、`/plan_kinematic_path` 永远不存在。→ 改为折叠字符串标量；重建包后 move_group 正常启动、服务可见（`GetMotionPlan`）。
2. **launch 漏挂 include（本次引入）**：§9.10 的 `moveit_planning = IncludeLaunchDescription(...)` 定义了变量但**未加入 `return LaunchDescription([...])`**，导致 move_group 在 teach_panel.launch 下从未被拉起。→ 已在返回列表补 `moveit_planning,` 并重建。

验证：
- `build --packages-select arachne_moveit_config` + `--packages-select arachne_operator` 通过；
- move_group 独立拉起：正常启动（"You can start planning now!"）、`/plan_kinematic_path` 服务类型 `GetMotionPlan`；
- 端到端规划回归（后台 spin executor 等同 teach_panel）：6→7 当年自碰场景 `ok=True, 101 pts`（MoveIt 找出绕过自碰的路径）；到 #1 安全视点 `ok=True`；
- teach_panel.launch 返回列表现含 5 个 Include（含 moveit_planning.launch.py），安装目录同步；
- headless/dry-run 检查全部通过。

**真机使用注意**：move_group 启动需数秒（加载模型/插件）；若在 "You can start planning now!" 之前点击 Execute，会得到一次 `MoveIt plan service unavailable`——稍候或再次 Execute 即可。仍待真机确认：FollowJointTrajectory 执行、关节名映射、到达判定。

### 9.12 实施补记 2（2026-08-21，真机二轮：执行通道与 state monitor）

方案 A 首跑后仍有问题，日志显示两条新线索，已修复：

1. **本部署无 ros2_control 轨迹控制器**：`use_aubo_ros2_driver:=false`（默认），不存在
   `/joint_trajectory_controller/follow_joint_trajectory`；手臂真机通道是 aubo SDK（速度桥
   speedJoint + `aubo_move_joint_action_server`/内部 RPC moveJoint）。原 `_send_moveit_trajectory`
   发 FollowJointTrajectory → 报 `arm action server unavailable`、机械臂不动。
   → 改为：MoveIt 轨迹（aubo_* 关节名）映射为无前缀命令关节序列 → **抽稀**（新增参数
   `moveit_waypoint_joint_step_rad`，默认 0.15）→ 逐点经 `_send_arm_sdk_move_joint`（带控制权/
   teach 门禁/keepout/到达校验，grasp real `sdk_move_joint` 同款模式）执行。离线模拟：6→7
   场景 101 点 → 18~25 个 waypoint，最大步内关节差 ~9~12°，沿 MoveIt 校验路径走。
2. **move_group state monitor 循环告警**：喂给 move_group 的关节状态只有 6 手臂关节，模型含
   `ms42dc_left/right_finger_joint` → 一直刷 `Missing ms42dc_right_finger_joint`。
   → teach_panel 把收到的 JointState **补齐两指关节（0.0）** 转发到 `moveit_joint_states_topic`
   （默认 `/arachne/moveit/joint_states`）；launch 中 move_group 改听该 topic；规划请求
   start_state 同步补两指（0.0），避免"start state 不完整"。

另修复：MoveIt 轨迹关节名 `aubo_*` → 命令名（无前缀）的映射曾用错 key（全部取 0.0），
已改为按 `removeprefix("aubo_")` 映射（v1 带同一 bug，因 action server 缺失从未外发）。

验证：py_compile / build / teach_panel --headless-check / launch include=5 通过；
映射+抽稀离线模拟通过。真机待确认：SDK moveJoint 逐 waypoint 执行节奏、到达校验、机械臂实际运动。

### 9.13 实施补记 3（2026-08-21，真机三轮：关节状态自给自足）

真机重跑后错误变为 `joint target: no joint state for moveit plan`（Aubo 可达但 `current_arm` 空）。
根因：**SDK 版 bringup 没有任何节点发布 `/joint_states`**（real_bringup 只有 scout/gripper/status/velocity
桥/moveJoint server），teach_panel 的关节状态一直依赖 `teach_visualization` 转发
`/joint_states -> /arachne/teach_visualization/joint_states`；一旦 `with_visualization` 为 false 或
转发缺失，teach_panel 就无关节数据，所有运动（含原速度 jog）都会卡。

修复（teach_panel 自给自足，不依赖任何 ROS topic）：
- 新增定时轮询 `_poll_aubo_joint_state`（参数 `arm_sdk_joint_poll_sec`，默认 0.5s）：Aubo 可达时
  直接 RPC `RobotState.getJointPositions` 刷新 `current_arm`/`tool_position`（FK），并把补齐
  夹爪两指的关节状态（`aubo_*` 名 + 两指 0.0，匹配 MoveIt 模型）发布到 `moveit_joint_states_topic`
  —— 同时解决"无关节状态"与 move_group 缺 `ms42dc_*` 循环告警，与可视化/RSP 无关。
- 保留原 `_joint_state_callback`（ROS topic 可用时冗余更新，不冲突）。

验证：SDK 直连真机（192.168.127.128:30004）`getJointPositions` 返回 6 关节角成功；
py_compile / build / headless 通过。真机待确认：轮询节奏、MoveIt 规划 + SDK moveJoint 逐点执行全链路。

### 9.14 实施补记 4（2026-08-21，真机四轮：整车关节状态补全，仿 grasp_preview）

用户建议“直接仿 grasp_preview 的路径”。核实后采纳其关键做法：
grasp 的 MoveIt start_state 是**整车完整关节**（`_moveit_start_state_joint_values`：
6 臂 `aubo_*` + 4 轮 BASE_JOINT_DEFAULTS + 夹爪 GRIPPER_JOINT_DEFAULTS，共 12 关节）。

本轮改动（teach_panel.py）：
- 新增 `MOVEIT_AUX_JOINTS`（front_left/right, rear_left/right wheel + ms42dc 两指）；
- `_publish_moveit_joint_states(arm_values)`：发布**完整 12 关节** joint_states 给
  `moveit_joint_states_topic`（臂 6 + 轮 4 + 指 2，后 8 个 0.0）——SDK 轮询与 ROS
  callback 共用；move_group state monitor 状态完整、循环告警消失；
- `_move_arm_to_joints_moveit` 的 start_state 补全 12 关节（仿 grasp），避免
  “start state 不完整/缺关节”。

验证（真实 move_group 端到端）：
- 发布 12 关节状态 10s → move_group 日志 `Missing/not yet known` **0 次**；
- 完整 start 状态下规划：6→7 自碰场景 ok=True(101 pts)、安全视点 ok=True；
- py_compile / build / headless 通过。
真机待确认：轮询节奏、SDK moveJoint 逐 waypoint 执行全链路实动。

### 9.15 实施补记 5（2026-08-21，真机五轮：moveJoint result=13 = 零位移起点被拒）

真机执行推进到 SDK moveJoint：`wp1/13` 即报 `result=13`、机械臂未动。
排查与结论：
- 控制器健康（mode=Running safety=Normal，只读查询）；历史上 moveJoint 大量成功
  （`Aubo SDK moveJoint reached: wp_7/12/15...`, 2026-06-27 grasp 会话），RPC 参数/门禁/速度
  均为有效路径；
- SDK 源码 `aubo_sdk-src/include/aubo/error_stack/system_error.h` 及 `share/doc/error_code.md`：
  小数字 13 = `RUNTIME_CONTEXT`（上下文错误），非普通运动语义；
- **失败点 wp1 = MoveIt 轨迹首点 = 请求 start_state = 当前姿态**：对零位移目标发
  moveJoint 被 SDK 拒绝。→ 修复：`_send_moveit_trajectory` 抽稀时跳过与当前姿态差值
  < `moveit_waypoint_min_delta_rad`（默认 0.008 rad）的起始点；整条轨迹都在起点附近则视为
  “已到位”直接成功。后续 waypoint 与上一点差值 ≥ `moveit_waypoint_joint_step_rad`（默认 0.15），
  均为非零位移，moveJoint 可正常执行。

验证：py_compile / build / headless 通过；逻辑已由历史“非零位移 moveJoint 成功 + 零位移失败”
证据支撑。真机待确认：跳过 wp1 后逐 waypoint 实动。

---

## 9.16 最终验证记录（2026-08-21，用户确认机械臂恢复正常移动）

**结论**：方案 A 全链路在真机验证通过，视点执行恢复正常移动，自碰撞防护生效。事故链路不复现。

**最终运行链**（与最初"裸速度"方案的差异即防护所在）：

```
teach_panel（GUI 点击 Execute）
  -> 自轮询 aubo SDK 关节角（RobotState.getJointPositions，0.5s；不依赖 ROS joint_states）
  -> 完整 12 关节状态（aubo_* 臂 6 + 4 轮 + 2 指）发布到 /arachne/moveit/joint_states
  -> move_group(OMPL) 规划 joint-goal（自碰撞 + start/环境校验；服务 /plan_kinematic_path）
  -> 轨迹 aubo_* -> 无前缀命令名映射 + 抽稀（moveit_waypoint_joint_step_rad=0.35，典型 5~6 步）
  -> SDK moveJoint 逐 waypoint（控制权/teach 门禁/keepout/到达校验，result=0 才算到）
```

**真机部署层问题（本轮暴露，均已修复，见补记）**：
1. §9.11 ompl_planning.yaml request/response_adapters 类型 -> move_group 崩溃；已改字符串。
2. §9.11 teach_panel.launch 漏挂 moveit_planning include；已入返回列表。
3. §9.12 无 ros2_control，执行改 SDK moveJoint 逐点（非 FollowJointTrajectory）。
4. §9.12-13 SDK bringup 无 /joint_states 源 -> teach_panel 自轮询关节，自给自足。
5. §9.14 MoveIt 模型需完整 12 关节（臂+轮+指）-> 发布/start_state 均补全（仿 grasp_preview）。
6. §9.15 moveJoint 对零位移起始点报 result=13 -> 抽稀时跳过与当前姿态重合的起点。

**未完成/可选改进（不在本次范围）**：
- C：视点规划级自碰撞过滤（planner 侧）——现由执行端 MoveIt 规划承担，非必需；
- D：aubo_move_joint_action_server 执行前碰撞预检——现由 MoveIt 规划承担，非必需；
- 逐点 moveJoint 节奏优化（换 SDK movePath 合并连续段、或提高抽稀步长）——按使用体验可选；
- ik_risk_todo_plan（IK 求解鲁棒性）与本次互不冲突，维持原计划。

**最终改动文件**（相对 §9 实施起点）：
`moveit_plan.py`(新增)、`teach_panel.py`、`teach_panel.launch.py`、`grasp_preview_pipeline.py`、
`ompl_planning.yaml`、`package.xml`；文档 `ee_camera_collision_analysis.md`（§9 全）。

**回归状态**：py_compile / colcon build / teach_panel --headless-check / grasp dry-run / entrypoints 均通过；
真机正常移动已由用户确认。
