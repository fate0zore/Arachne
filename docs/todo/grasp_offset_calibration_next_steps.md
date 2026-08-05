# 抓取偏移排查与修复计划（grasp_offset_calibration_next_steps）

> 状态：**已修复（2026-08-05 真机确认可夹取）**：主偏差（grasp_base_offset=-0.23）已置零修复；残余 2-5cm 偏近侧经多轮排查后以 grasp_base_offset=+0.03 参数补正收敛；Phase C（color/depth 对齐）/D/E 转为后续优化项
> 创建：2026-08-05
> 更新：2026-08-05（Phase A 实验 1/3 完成，环节 1 确认；Phase B 置零落地；残余排查 ①-⑤ 完成；补正方案（base_offset=+0.03）落地并真机验证可夹取）
> 来源：2026-08-05 只读代码审查结论（沿 teach_panel 启动线排查；不修改任何代码）+ 2026-08-05 真机实验 1/3（teach_panel 触发）
> 目标读者：后续接手抓取偏移排查/修复的开发者；文档应可独立执行或续接

---

## 0. 结论摘要（2026-08-05 审查）

> **0.1 实测结论（2026-08-05 全流程，替代早期“未定位”状态）**
> - 环节 1 **实锤并已修复**：实验 3 置零对照（`grasp_base_offset:=0.0,0.0,0.0`）后，前后偏差从 ~20 cm（偏机器人侧）降到 2-5 cm → `grasp_base_offset=(-0.23, 0.01, 0.16)` 是“极其严重偏移”的主因；置零后残余经多轮排查（§1“残余 2-5cm 来源分析”），最终以 `grasp_base_offset=(0.03, 0, 0)` 正参数补正收敛，**真机确认可夹取**。
> - 残余 2-5 cm（仍偏机器人/相机近侧）来源：grasp 目标点取点链路多因素叠加——平放物体可见轮廓不对称致 bbox 中心偏近侧、`grasp_standoff` 沿 camera 光轴（≈水平）做减法而非垂直、近大远小采样偏差（详见 §1 残余分析）。
> - **修复落地**：`grasp_base_offset` 默认值 4 处统一为 `0.03,0.0,0.0`（grasp_task_server.launch.py:60 / grasp_task_server.py:145 / teach_panel.launch.py:650 / teach_panel.py:630）；`grasp_preview_pipeline.py` grasp 取点改为 ROI 点云 3D 中位数、camera 系 `grasp_standoff` 不再应用（垂直补偿由 `grasp_final_z_offset_m=-0.055` 承担）；包已重建，真机验证通过。

**一句话**：teach_panel 线上 RViz 点云达标（用户真机确认点云贴地、与实物对齐）只验证了"**深度像素 → base_link 3D 点**"这条链路（depth 内参 + 外参 static TF + 深度图方向）自洽；而 **grasp 目标点**还额外经过 `grasp_base_offset=(-0.23, 0.01, 0.16)` m 常量平移与 color/depth 映射两步，**这两步在 teach_panel 默认 RViz 中完全不可见**，是最可能的"极其严重偏移"来源。

### 关键事实

1. 当前生效相机外参仍是**经验试调值**（非可信标定结果）：`xyz=(0.0201, 0, 0.05)`、`rpy=(0, 0, -1.570796327)`（与 `hand_eye_calibration_next_steps.md` §四.2 一致；Phase 3/4 可信标定**从未接入**）。
2. teach_panel 线 RViz 点云来自 `depth_to_pointcloud`（`demo_orchestrator.launch.py` 的 `depth_pointcloud_command`，`projection_flip_x/y:=false`、`target_frame:=base_link`，发布 `/arachne/debug/depth_points`）。
3. grasp 像素→3D 反投影（`grasp_preview_pipeline.py::_pixels_to_xyz`）与 depth_to_pointcloud **同向同源**（都用 depth camera_info + `u-cx`），所以 grasp 的"深度像素→base 点"与 RViz 显示一致。
4. 差异只出现在 grasp 特有环节：`grasp_base_offset` 常量平移、YOLO color bbox → depth 图映射（未补偿 color/depth 基线）、grasp 后处理参数、color/depth 帧同步。
5. **2026-08-05 收尾**：环节 1 已修复（置零 + 补正）；残余 2-5cm 以 `grasp_base_offset=+0.03` 参数补正收敛；Phase C/D/E 保留为后续优化项（color/depth 基线对齐未做、可信手眼标定未接入）。

### 差异定位表

| 链路 | 深度像素→base 点 | grasp_base_offset | color bbox→depth 映射 | RViz 中可见 |
|---|---|---|---|---|
| RViz 点云（depth_to_pointcloud） | ✅（用户确认达标） | 无 | 无 | ✅ |
| grasp 目标点（grasp_preview） | ✅ 相同 | **+(-0.23,0.01,0.16) m** | **直接缩放映射，无基线补偿** | ❌（teach 默认 rviz 无 marker） |

---

## 1. 根因环节（嫌疑分级，含代码证据）

### 环节 1（✅ 已修复 2026-08-05）：`grasp_base_offset` 常量平移

- **代码事实**：
  - `src/arachne_operator/launch/grasp_task_server.launch.py:60`：`DeclareLaunchArgument("grasp_base_offset", default_value="-0.23,0.01,0.16")`
  - `src/arachne_operator/arachne_operator/grasp_task_server.py:145`：`self.declare_parameter("grasp_base_offset", "-0.23,0.01,0.16")`（与 launch 默认一致）
  - `grasp_task_server.py:611` 与 `:1968`：`env["ARACHNE_GRASP_BASE_OFFSET"] = str(...)`
  - `grasp_task_server.py:2039`：启动 grasp_preview 时拼入 `--grasp-base-offset=<env值>`
  - `grasp_preview_pipeline.py:848`：`self.grasp_base_offset = _xyz(args.grasp_base_offset)`
  - `grasp_preview_pipeline.py:5830`（`_make_base_path`）：`base_grasp_path = [self._apply_grasp_base_offset(self._transform_point(transform, point)) for point in camera_path]`——**先 TF 到 base_link，再加常量平移**
  - `grasp_preview_pipeline.py:6024`（`_estimate_pointcloud_grasp_shape_base`）：`shifted_center = self._apply_grasp_base_offset(...)`——聚类中心同样被平移
  - `grasp_preview_pipeline.py:6115-6125`：`_apply_grasp_base_offset` 为纯常量加法
- **为什么是嫌疑**：点云已经 `target_frame:=base_link`（TF 完备），在 base 系点云上再叠 0.23 m 常量偏移属双重/错误映射。grasp_preview 单独运行时默认 `--grasp-base-offset` 为 `0,0,0`（`grasp_preview_pipeline.py` 参数默认），**只有 grasp_task_server（road_cleanup/urban_trash 场景服务器）传入 -0.23**——高度疑似为"相机在 AGV 上、粗略映射 base 作业区"时代的遗留 hack。
- **与"标定参数改变后出现偏移"的耦合**：若旧外参下"点云位置 + offset"恰好落回正确作业点，外参改变后点云位置变了而 offset 未变 → 目标点系统性错 23 cm 量级。
- **为什么 RViz 不可见**：teach_panel 默认 RViz（`src/arachne_description/rviz/arachne_lidar_fusion.rviz`）只显示 Grid/RobotModel/TF/LSlidar C16/Grasp ROI Cloud/Depth Debug Cloud，**没有订阅 grasp 目标 marker**；`grasp_preview.rviz` 才有 `GraspPreviewMarkers`（teach 线不用）。
- **影响量级**：0.23 m 恒定偏移，远大于夹爪容差，足以造成"极其严重偏移"。
- **验证方法**：见 §2 实验 1/2/3。
- **修复落地（2026-08-05）**：置零对照实锤后，默认值 4 处统一为 `0.03,0.0,0.0`（补偿残余偏近侧）；真机验证可夹取。

### 环节 2（次嫌疑，5~10 cm 级）：YOLO color bbox → depth 图映射未补偿 color/depth 基线

- **代码事实**：
  - `config/gemini335_intrinsics/factory_calibration.json`：color `fx=461.147/fy=461.369/cx=323.604/cy=246.426`、depth `fx=fy=372.082/cx=319.400/cy=241.5`，且 `transform_depth_to_color.trans = (-13.67, -0.14, -2.23)` mm——**color 与 depth 是不同传感器，存在 ~13.7 mm 基线**
  - `grasp_preview_pipeline.py`（`_make_preview`，约 L3485 起）：YOLO 在 color 图检测 bbox/mask，`x1d = x1 * sx`、`y1d = y1 * sy`（`sx=dw/cw`）**按分辨率缩放直接映射到 depth 图，无视差/基线补偿**
  - `src/arachne_sensors/launch/gemini335.launch.py`：color 与 depth 两个 static TF 使用**完全相同**的 `camera_optical_*` 值（`camera_color_optical_frame` 与 `camera_depth_optical_frame` 被当作同一位置）；`camera_info` 的 P 矩阵 `Tx=0`（假设 rectified 对齐，实际未对齐）
- **影响**：0.5 m 处视差约 10 px；对贴地/小目标（road_cleanup 目标在地面），ROI 内深度采样可能取到背景/地面 → 深度误差 5~10 cm+，且横向错位约 `Δu·z/fx`。
- **验证方法**：同一静止物体，分别记录 color bbox 中心像素与对应 depth 图"物体实际像素"之差；或做 §2 实验 3 后对比残余误差。

### 环节 3（次要）：color 帧与 depth 帧时间不同步

- grasp_preview 独立订阅 color/depth（grasp_task_server 以 subprocess 启动，`ARACHNE_GRASP_START_CAMERA=false`），检测用最新 color 帧与最新 depth 帧之间允许一定帧龄差（存在 `--max-color-frame-age-sec` 类参数）；检测瞬间机械臂/抖动时 bbox 与深度时空错位。
- **影响**：动态场景 cm 级；静止采样时可忽略。

### 环节 4（排除环节 1/2 后再查）：执行侧 tool0 ← grasp_frame / SDK FK

- `grasp_preview_pipeline.py:56`：`END_EFFECTOR_FRAME = "grasp_frame"`；`_tool0_target_from_grasp_target`（L4096-4113）：`tool_in_base = grasp_in_base @ inv(tool_to_grasp)`；`_frame_candidates`（L4113）经 TF `tool0 ← grasp_frame` 取工具矩阵。
- 工具坐标来自 URDF（本次标定未改动），SDK 执行与 TF 同树，代码上自洽；**仅当 §2 实验确认 RViz marker 在物体上而夹爪仍偏时**再查此环节（如 URDF grasp_frame 偏移、SDK 关节角 vs URDF FK 差异）。

### 残余 2-5cm 来源分析（2026-08-05 已排查；以参数补正收敛，未改取点算法）

> 置零后残余 2-5cm（仍偏机器人/相机近侧）的排查过程与结论；最终用户决策以 `grasp_base_offset=+0.03` 补正收敛，未继续深挖。

| # | 尝试 | 结果 | 证据 |
|---|---|---|---|
| ① | `depth_percentile` 35→50 | 无效（落点不变） | `depth_band=0.08` 本就保留 ROI 全部点，percentile 只影响统计中心 |
| ② | grasp 点改 bbox 中心像素 + ROI 中位深度反投影 | 无效（仍碰瓶底） | 日志 grasp_camera 变化 <1cm |
| ③ | bbox 中心 + 邻域实际深度反投影 | 仍碰末端 | 邻域深度 0.723m < ROI 中位 0.748m → **bbox 中心像素本身落在瓶子近侧段** |
| ④ | 删除 `grasp_standoff`（camera 系光轴方向 -3.5cm） | 仍碰末端 | bbox 中心偏近侧为主因；standoff 方向错误（光轴≈水平）是叠加项 |
| ⑤ | grasp 点改 ROI 点云 3D 中位数（物体几何中心） | 该次 YOLO 检测全被 edge 过滤（瓶子贴图像底边）→ warm execute timeout，未执行 | 日志 `YOLO detections filtered out ... edge=6` |
| ⑥ | 用户决策：`grasp_base_offset` x=+0.03 推离 AGV 侧（4 处） | **真机确认可夹取** ✅ | base_offset=(0.030,0.000,0.000) 生效 |

**根因链**：平放瓶子从上前方看可见轮廓前后不对称（近侧端面可见、远侧端面被顶面遮挡）→ bbox 中心像素偏近侧；`grasp_standoff` 在 camera 系沿光轴（≈水平）做减法，等效水平拉近 AGV 侧 ~3.3cm；近大远小使表面点采样偏近。最终以参数补正收敛（用户决策），未继续调整取点算法。

### 已排除环节（teach_panel 实际运行环境下不成立，勿再投入排查）

| 环节 | 排除理由 |
|---|---|
| flip 镜像矛盾（相机节点 `/camera/points` 用 `cx-u` vs grasp 用 `u-cx`） | teach 线 `publish_pointcloud:=false`，相机节点点云根本不发布；RViz 点云与 grasp 反投影同为 `u-cx`（`depth_to_pointcloud` 的 `projection_flip_x/y:=false`） |
| 外参 z=0.05 点云整体偏移 | RViz 点云贴地真机确认达标，说明"深度像素→base"链路自洽 |
| depth 内参 fx 576→372 几何放大 | grasp 与 depth_to_pointcloud 用同一 `depth_info`，RViz 点云达标即证明该内参使用一致 |

---

## 2. 无侵入验证步骤（不改任何代码，先做；需真机，确认机械臂/相机就绪）

> 目的：在修复前先确认"偏移发生在哪一环"，避免盲目改动。所有实验按附录 B 模板记录。

### 实验 1：grasp 日志核对（零风险）
- 跑一次 grasp（road_cleanup / step_cleanup_demo 或 teach_panel 触发），看 grasp_preview 日志行：
  `grasp_camera=(gx,gy,gz) ... base_offset=(-0.230, 0.010, 0.160) ...`
  （打印点：`grasp_preview_pipeline.py:3870-3871`）
- 记录：`grasp_camera` 是否指向物体方向；`base_offset` 是否如预期 -0.23。

### 实验 2：RViz 手动添加 grasp marker（零风险）
- 在 teach_panel 的 RViz 中手动添加 MarkerArray display，订阅 `/arachne/grasp_preview/markers`（grasp_preview 发布点：`grasp_preview_pipeline.py:976`）。
- 观察：marker（approach/grasp 目标点）相对点云中物体的位置。
  - marker 在物体上、点云也贴地 → 目标计算对，嫌疑转执行侧（环节 4）
  - marker 明显偏出物体（≈0.23 m 级）→ 嫌疑锁定环节 1
  - marker 偏几 cm 级 → 嫌疑在环节 2/3

### 实验 3：`grasp_base_offset` 置零对照（仅 launch 参数，不改代码）
- 临时用 launch 参数覆盖重跑一次 grasp：
  `grasp_base_offset:=0.0,0.0,0.0`（对应 grasp_task_server.launch.py 的 launch argument）
- 对照实验 1/2 的偏移量与方向：
  - 偏移消失（目标点回到点云物体上）→ 环节 1 实锤，进入 Phase B
  - 偏移减小但仍有残余（cm 级）→ 环节 1 + 环节 2/3 叠加
  - 偏移不变 → 环节 1 非主因，优先查环节 2/3，再查环节 4

### 实验 4（可选，判断 color/depth 对齐）
- 静止物体，导出同一时刻 color 图与 depth 图，手动比较物体在两张图中的像素中心差（预期 ~10 px@0.5 m，方向由 `transform_depth_to_color` 决定）。
- 或临时用 `--depth-only-projection`（若存在）绕过 color bbox 映射直接验证。

---

## 3. 修复路线（按依赖顺序执行；每阶段验证通过再进下一阶段）

### Phase A：定位确认（需真机）
- 执行 §2 实验 1~3（+可选实验 4），按附录 B 模板记录数据。
- 判定表：
  | 实验结果 | 判定 | 下一步 |
  |---|---|---|
  | 置零后偏移消失 | 环节 1 主因 | Phase B |
  | 置零后残余 cm 级 | 环节 1 + 2/3 | Phase B → Phase C |
  | 置零后偏移不变且 marker 在物体上 | 执行侧（环节 4） | Phase E（执行侧专项） |
- 完成条件：结论写回本文档（§0/§1 标注"已确认"或修正）。

### Phase B：修正 `grasp_base_offset`（改动面小，预计 <30 行）
- 方案（按优先级）：
  1. **置零/恢复默认**：把 `grasp_task_server.launch.py:60` 与 `grasp_task_server.py:145` 默认改为 `0.0,0.0,0.0`（与 grasp_preview 独立运行默认一致）；同步 `teach_panel.launch.py` 帮助文本（L650 附近 `grasp_base_offset:=-0.23,0.01,0.16` 字符串）。
  2. 若场景确需作业偏移（如 AGV 前移后的作业区），改为**显式命名参数 + 文档说明**，不再沿用"相机粗略映射"语义。
- 同步检查：`grasp_task_server.py:611/1968/2039` 传递链无需改（参数值随 launch 变）。
- 验证：重跑 grasp，目标点回到点云物体上；对照 §4 验收。

### Phase C：color/depth 对齐补偿（若 Phase A 判定位环节 2）
- 方案候选（需先调研可行性，勿盲目实现）：
  1. 在 grasp 的 bbox→depth 映射处补偿 `transform_depth_to_color`（13.7 mm 基线 → 像素视差），或对 depth 像素坐标做重映射；
  2. 验证 Gemini 335 是否支持硬件深度对齐到 color（若支持，改驱动/链路输出 aligned depth，则 bbox 直接对齐）；
  3. 最小改动：ROI 在 depth 图中按基线方向平移后再采样。
- 验证：实验 4 的像素差归零；抓取目标点与实物偏差进入 cm 级。

### Phase D：全链路回归与验收（需真机）
- 复测 §4 全部验收项；记录 3 次独立抓取的数据。
- 回归确认无影响：RViz 点云仍贴地达标；teach_panel 其他功能（Camera Cal tab、手眼标定流程）不受影响。

### Phase E：配置/文档同步与收尾
- 外参一致性核查：8 处 `camera_optical_*`（gemini335.launch.py / depth_to_pointcloud.launch.py / display.launch.py / teach_panel.launch.py 帮助文本 / teach_panel.py `CURRENT_HAND_EYE_*` / hand_eye_calibration_check.py `PHYSICAL_PRIOR_*` / config current_effective.json）与 `physical_parameters.yaml` 的 archived 字段，确保与 `hand_eye_calibration_next_steps.md` §四.2 清单一致。
- 与 `hand_eye_calibration_next_steps.md` 衔接：Phase 3/4（可信手眼标定接入）仍未完成——若后续完成可信标定，需在本任务 Phase D 一并复测。
- 清理：检查 `.bak` 备份文件是否需归档；更新本文档状态为"已修复 + 证据"。

---

## 4. 验收标准（全部满足才算完成）

- [ ] 抓取目标点（grasp marker / 日志 `grasp_camera` 换算 base 点）与点云中物体位置偏差 < 3 cm（对照 `hand_eye_calibration_next_steps.md` 阶段 C 验收口径）；
- [ ] 夹爪实际到达位置与目标物体偏差 < 3 cm（3 次独立抓取均满足）；
- [ ] RViz 点云保持贴地达标（回归无破坏）；
- [ ] 修改面最小、无遗留硬编码；launch 参数默认值有注释/文档说明语义；
- [ ] `./scripts/build/build_workspace.sh` 构建通过；相关 `--dry-run-check` 通过；
- [ ] `git status --short` 已检查；本文档状态更新并附实验数据。

---

## 5. 安全与流程约束（沿用 AGENTS.md）

- 真机抓取/运动验证前，用户明确确认机械臂就绪。
- dry-run/headless 路径不触碰真实硬件。
- 改动后经 `./scripts/build/build_workspace.sh` 重建。
- `teach_panel.launch.py` 保持为正常操作入口。
- 不改动与本次无关的标定参数；不动 `config/hand_eye_camera_extrinsics/current_effective.json` 除非 Phase E 明确需要。
- 所有实验先"验证后生效"，不自动改写 launch/config。

---

## 附录 A：`grasp_base_offset` 完整传递链与修改点

```
grasp_task_server.launch.py:60   DeclareLaunchArgument("grasp_base_offset", default="-0.23,0.01,0.16")
        ↓ (teach_panel.launch.py:650 帮助文本同值；demo_orchestrator.launch.py 未覆盖，沿用默认)
grasp_task_server.py:145         declare_parameter("grasp_base_offset", "-0.23,0.01,0.16")
grasp_task_server.py:611/1968    env["ARACHNE_GRASP_BASE_OFFSET"] = str(param)
grasp_task_server.py:2039        "--grasp-base-offset=" + env 值
        ↓ (subprocess 启动 grasp_preview_pipeline)
grasp_preview_pipeline.py:848    self.grasp_base_offset = _xyz(args.grasp_base_offset)
grasp_preview_pipeline.py:5830   base_grasp_path 每点加 offset（TF 到 base 之后）
grasp_preview_pipeline.py:6024   shifted_center 加 offset（聚类中心）
grasp_preview_pipeline.py:6115   _apply_grasp_base_offset：常量加法
```

修改点（Phase B）：`grasp_task_server.launch.py:60`、`grasp_task_server.py:145`、`teach_panel.launch.py:650` 帮助文本。

## 附录 B：排查记录模板

| # | 日期 | 实验（1/2/3/4） | 条件（外参/内参/场景/机械臂位姿） | grasp_camera | base_offset | marker 相对物体偏移 (x,y,z) | 夹爪实际偏移 | 结论/备注 |
| 1 | 2026-08-05 | 实验1：grasp 日志核对 | teach_panel 触发；外参 0.0201/0/0.05/yaw=-90°；物体=平放饮料瓶（extent 8.8×4.2×5.2cm，z_bias 3cm） | (-0.019, 0.266, 0.710) conf=0.72 | (-0.230, 0.010, 0.160) | 未做（实验2） | **偏机器人侧 ~20cm**；左右无偏；高度未观察 | 链路自洽；offset 生效；方向/量级与 offset.x=-0.23 吻合 |
| 2 | 2026-08-05 | 实验3：offset 置零对照 | 同上（物体尽量同位置） | 见 idle_preview 日志（warm-execute 复用进程） | (0.000, 0.000, 0.000) | 未做 | **偏机器人侧 2-5cm**（碰瓶近侧瓶底）；左右/高度可接受 | **环节 1 实锤**：20cm→2-5cm；残余= grasp 点取近表面层（depth_percentile=35% + depth_band 过滤 + 中位数），非 offset 残留 |
| 3 | 2026-08-05 | 残余①：depth_percentile 35→50 | 同上 | 见 idle_preview 日志 | (0.000, 0.000, 0.000) | 未做 | 仍碰瓶底 | 无效：`depth_band=0.08` 本就保留 ROI 全部点，percentile 只影响统计中心 |
| 4 | 2026-08-05 | 残余②：bbox 中心像素 + ROI 中位深度反投影 | 同上 | (-0.021, 0.266, 0.710)（depth 0.739） | (0.000, 0.000, 0.000) | marker 落在瓶子尾部（RViz 观察） | 仍碰瓶底 | bbox 中心=可见轮廓几何中心，轮廓前后不对称（近侧端面可见、远侧端面被遮挡）→ 偏近侧 |
| 5 | 2026-08-05 | 残余③：bbox 中心 + 邻域实际深度反投影 | 同上 | (-0.050, 0.261, 0.686)（depth 0.748） | (0.000, 0.000, 0.000) | 未做 | 仍碰末端 | **邻域深度 0.723m < ROI 中位 0.748m** → bbox 中心像素本身落在瓶子近侧段 |
| 6 | 2026-08-05 | 残余④：删除 camera 系 grasp_standoff | 同上 | (-0.050, 0.262, 0.723) | (0.000, 0.000, 0.000) | 未做 | 仍碰末端 | standoff 沿光轴（≈水平）做减法等效水平拉近 AGV 侧 ~3.3cm；删除后 bbox 中心偏近侧仍是主因 |
| 7 | 2026-08-05 | 残余⑤：grasp 点改 ROI 点云 3D 中位数 | 同上 | 无（未执行） | (0.000, 0.000, 0.000) | 未做 | 未执行（warm execute timeout） | 该次 YOLO 检测全被 edge 过滤（瓶子贴图像底边，conf 0.42-0.60），非代码问题 |
| 8 | 2026-08-05 | 补正：`grasp_base_offset` x=+0.03（用户决策） | 同上 | 见 idle_preview 日志 | (0.030, 0.000, 0.000) | 未做 | **真机可夹取** ✅ | 参数补正收敛；微调：偏近→加大（0.04-0.05），偏远→减小（0.02） |

## 附录 C：相关文件索引

| 文件 | 作用 |
|---|---|
| `src/arachne_operator/launch/grasp_task_server.launch.py` | grasp 服务器 launch；`grasp_base_offset` 默认 **0.03,0.0,0.0**（L60，原 -0.23,0.01,0.16 已置零+补正） |
| `src/arachne_operator/arachne_operator/grasp_task_server.py` | grasp 服务器节点；参数默认（L145）、env 传递（L611/1968/2039） |
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | grasp 主链路：`_pixels_to_xyz`（约 L3743）、`_make_preview` bbox 映射（约 L3485，grasp 取点现为 **ROI 点云 3D 中位数**，camera 系 `grasp_standoff` 不再应用）、`_make_base_path`（L5802）、`_apply_grasp_base_offset`（L6115）、`_tool0_target_from_grasp_target`（L4096）、marker 发布（L976） |
| `src/arachne_operator/launch/demo_orchestrator.launch.py` | `camera_command`（L33-45，外参 0.0201/0/0.05 + flip=true + publish_pointcloud=false）、`depth_pointcloud_command`（L46-55，flip=false、target=base_link、/arachne/debug/depth_points）、`grasp_server_command`（L72-98） |
| `src/arachne_operator/launch/teach_panel.launch.py` | 正常操作入口；camera include（L37-59）；帮助文本（L650 含 grasp_base_offset 串） |
| `src/arachne_sensors/launch/gemini335.launch.py` | 相机 launch；color/depth static TF 同值（ee_camera_link←camera_*_optical_frame）；`factory_intrinsics_file` 默认 `config/gemini335_intrinsics/factory_calibration.json` |
| `src/arachne_sensors/launch/depth_to_pointcloud.launch.py` | debug 点云工具（非 teach 依赖）；flip 默认 false |
| `config/gemini335_intrinsics/factory_calibration.json` | 出厂内参；color fx=461.147、depth fx=372.082、`transform_depth_to_color` trans=(-13.67,-0.14,-2.23) mm |
| `src/arachne_description/rviz/arachne_lidar_fusion.rviz` | teach_panel 默认 RViz：仅点云，无 grasp marker |
| `src/arachne_description/rviz/grasp_preview.rviz` | grasp 专用 RViz：含 GraspPreviewMarkers/GraspRoiCloud/GraspToBasketPath |
| `docs/todo/hand_eye_calibration_next_steps.md` | 手眼标定主线文档（Phase 3/4 可信标定未接入；外参 8 处同步清单 §四.2） |
| `docs/todo/viewpoint_planning.md` | 视点规划任务（后续衔接） |
| `src/arachne_description/config/physical_parameters.yaml` | `hand_eye_tool0_to_camera_color_optical` 仍 archived（未更新） |
