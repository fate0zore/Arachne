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
