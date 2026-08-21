# 视点规划第二阶段（phase2）任务规划：碰撞过滤 + 一键采集（viewpoint_planner_phase2_todo_plan）

> 状态：**已完成（2026-08）**——Phase 1-4 全部实现并【实机测试验收通过】（用户 2026-08 确认）。此前暂停的“碰撞异常 + 首轮只采到视点 0”问题已由用户自行修复后顺利完成真机复测；随后的三项补强（批次落盘区分、RViz 视点编号 + 常驻预览、Viewpoint 列表列宽修复）也已落地并实机验收通过。
> 进度回写（2026-08，本会话补强 + 实机验收通过）：① **批次落盘区分**——新增 `make_viewpoint_run_id`（时间戳目录，同秒自动追加 `_2`/`_3`），每次 `Acquire All` 落盘到 `<session>/viewpoint_acquisition/<run_id>/`，manifest 增 `run_id`/`session_stamp`，`viewpoint_acquisition/latest` 软链指向最新批，四个 `viewpoint_acquire_*` 事件均带 run_id（多批互不覆盖、事后可关联）；② **RViz 编号 + 常驻预览**——每视点新增 `TEXT_VIEW_FACING` 编号文本（0 基、三色同箭头、ns=`viewpoint_labels`，`marker_label_*` 参数），planner 缓存最近 MarkerArray 按 `markers_republish_period_sec`（默认 1.0s）周期重发，操作员入口 `arachne_lidar_fusion.rviz` 预置 `/arachne/viewpoint/markers` Display（Transient Local）——启动即自动订阅、无需手动 Add；③ **Viewpoint 列表列宽修复**——Treeview 列宽误用 5/8/24/24/32 px（挤在左上角），改为 40/72/140/140/240 + joints 列 stretch 铺满。以上均通过 py_compile/离线单测（采集 16 项 + planner 64 项）/全量构建（17 pkg）/headless/dry-run 验证，并【实机验收通过】（用户 2026-08 确认）。
> 进度回写（2026-08-21，一键采集实机诊断 + 暂停）：首次真机 Acquire All **只采到视点 0（共 6 个可达视点）**。两层原因：① **主因**——视点 #0 到位采集成功，但 **#1~#5 全部 `arm velocity target timeout/cancel`**（约 11.8s 误差未降至 `arm_goal_tolerance=0.04`，非 keepout，无任何 `safety zone` 消息；timeout 而非 keepout 拦截），按"超时跳过"设计故只采 1 组；② **副因**——视点 0 数据不完整：`cloud` 为 null，`errors=["no point cloud frame"]`，根因是相机以 `publish_pointcloud:=false` 启动（`teach_panel.launch.py:436` 默认 false，`/camera/points` 无发布）。timeout 已排除 keepout/角度回绕（`_angle_diff` 标准 wrap、6 目标差值 <±2 rad）；最反常的是 #0 大行程 5.1s 到位而 #1 小行程 12s 达不到 → 疑似速度流式在连续链式运动中未让机械臂持续到位。**已落地诊断增强**（采集层、只加事件不改运动行为）：`_acquire_start_motion` 记录起始关节角，失败视点写 `viewpoint_acquire_skip` 事件（含 `start/end/target_joints_rad`），重跑一次可判别"机械臂没动(≈start) vs 动了但未到位(end 接近 target 带残差)"。诊断增强已通过 py_compile/全量构建(17 pkg)/headless-check/20 unittest/59 pytest。**在等待重跑判别前，用户手动测试中发现碰撞异常，自行修复中，本任务暂挂起。**
> 进度回写（2026-08）：代码层已完成 Phase 1-2 全部实现项（msg/srv 字段、teach_panel 下游消费链 6 处、geometry.py、collision_filter.py、节点接线、markers 三色、17 个离线单测），并通过 dry-run-check / headless-check / 全量构建 / round-trip 验证。**【实机测试通过】**：teach_panel 与 viewpoint_planner 两进程同停同启后 service 可调用、无类型不兼容报错；碰撞过滤对真实目标输出 `collision_free` 逐点标记合理；RViz 三色箭头抽查通过（用户确认）。
> 进度回写（2026-08，Phase 3 完成）：**一键采集已完成代码层 + 离线验证**——teach_panel 新增 depth/points/camera_info 订阅与缓存；模块级落盘纯函数（`serialize_pcd` / `depth_image_to_uint16`(16UC1 mm 不除 depth_scale) / `pointcloud_xyz_hw`(纯 numpy 解析 create_cloud_xyz32) / `decimate_cloud_points` / `camera_info_to_dict`）；`move_arm_to_joints_degrees` 新增可选 `on_success(success:bool)` 回调（向后兼容单点 Execute 的 `on_done`）；Viewpoint tab 新增 **Acquire All** 按钮 + 总确认框 + `on_success` 链式逐点运动采集 + 独立 `_viewpoint_acquire_busy` 门禁 + 异常容错；落盘 `<session>/viewpoint_acquisition/<n>/{color,depth,cloud}+manifest.json`（每点写后即时刷新）+ `events.jsonl`。离线 20 单测全绿（含新增 13 项）、59 项 planner pytest 全绿（并修复 `test_markers` 缺 `collision_free` 字段的 Phase 2 遗留失败）、headless/dry-run/全量构建/回归 dry-run 全过。真机项（`Acquire All` 顺序运动逐点落盘）待用户实机验收。
> 接续 `viewpoint_planning_todo_plan.md`（第一阶段已全部完成，含真机验收）。此前已完成 2026-08 第二轮审查修订（补 ik_joints 下游消费链、msg 接口破坏性变更提示、路径安全/落盘成败信号、碰撞盒集合含车体、点云/深度落盘细节；新增用户确认的 3 项补充决策 §0.2）。
> 创建：2026-08（本阶段）
> 来源：用户需求（① 在 IK 过滤基础上新增碰撞过滤，两次过滤分别对应两个 JSON 字段；② teach_panel 视点规划界面新增"一键采集"，按可达视点顺序逐点运动并采集存储）+ 2026-08 只读代码审查结论 + 用户确认的 4 项设计决策（§0.2）+ 用户确认的 3 项补充决策（§0.2）。
> 目标读者：后续接手实现本阶段两功能的开发者；文档应可独立执行或续接。

---

## 0. 任务总结

### 0.1 需求原文（本阶段）

1. **修改视点规划过滤功能**：在既有 IK 可达过滤的基础上，**再增加一次碰撞过滤**；两次过滤分别在 JSON 中对应**两个独立的字段**（`ik_reachable` 为第一段、碰撞字段为第二段），互不覆盖，逐点标记。
2. **新增一键采集功能**（视点规划界面）：让机械臂**按每个可达视点的顺序**逐个运动到位，在每点**采集信息并存储**；采集哪些信息经用户确认（见 §0.2）。

### 0.2 已确认设计决策（用户 confirm，见 §0.4 问答摘要）

| 决策项 | 结论 |
| --- | --- |
| 碰撞过滤实现 | **自研纯 numpy 几何**（仿 `ik_filter.py` 建独立 `collision_filter.py`），复用 `AuboI5Kinematics.link_transforms` / `ee_collision.py` 与 keepout 盒/地面，**不接入 MoveIt PlanningScene**（仓库主链路未用 move_group 做碰撞） |
| 碰撞检查范围 | **keepout 环境盒（车体/前篮筐/后货架）+ 地面 + 末端偏心相机盒 + 机械臂连杆自碰撞**（A 级完整；见 §2.2 备选降级） |
| 采集信息 | **全采**：RGB 彩色图（bgr8）、深度图 depth（16bit）、点云 PointCloud2、相机内参 camera_info、每视点关节角 / tool0 / camera 位姿快照 |
| 落盘形式 | **`runtime_log_dir` 下按视点子目录 + 一个伴随 JSON 清单**（每视点独立子目录存图像/点云，清单含位姿/关节角/内参/时间戳），不新增第三方依赖（点云 `.pcd` 用手写 ASCII 写盘） |
| 一键采集执行门禁 | **沿用现有运动链路 + 弹一个总确认框**（列出将访问的可达视点数与目标位置），确认后链式逐点运动采集；keepout 检查保持开启 |
| **路径安全**（第二轮新增） | **运动链路实时拦截兜底**：不改运动链路，依赖 teach_panel 现有 `_arm_keepout_violation`（含 `_arm_keepout_path` 沿途插值）在运动时实时拦截；计划补写拦截后的链行为（跳过该点继续下一视点）+ 在风险项明确 `collision_free` 仅为规划参考、非执行授权（见 §3.1 路径安全、§3.4 实现要点） |
| **碰撞盒集合**（第二轮新增） | **含车体盒，对齐 `grasp_preview._make_collision_boxes` 4 盒**：`scout_base_main` + `scout_base_center_ridge` + `front_basket_keepout` + `rear_sensor_rack`（带各自 margin），**而非** teach_panel `_arm_keepout_boxes` 的 2 盒（无车体）。规划阶段更保守更安全（见 §2.2、§2.4） |
| **on_done 成败信号**（第二轮新增） | **改运动入口传结果位**：给 `move_arm_to_joints_degrees` 增加可选成功回调参数（内部根据 `_move_arm_to_positions_velocity` 返回 True/False 传递），向后兼容现有单点 `Execute`（见 §3.4 实现要点） |
| **批次落盘区分**（2026-08 补强） | 每次 `Acquire All` 独立一批：`<session>/viewpoint_acquisition/<run_id>/`（run_id = 采集启动时间戳，同秒追加 `_2`/`_3`）；manifest 增 `run_id` / `session_stamp`；`viewpoint_acquisition/latest` 软链指向最新批；`viewpoint_acquire_*` 事件带 run_id（用户确认：自动时间戳、一次采集=一批、仅 run_id+session、仅 latest 软链） |
| **RViz 编号 + 常驻预览**（2026-08 补强） | 编号=`TEXT_VIEW_FACING` 文本（0 基 index 对齐 teach_panel 列表行、三色同箭头、相机原点上方偏移/字号参数化）；常驻=预置 `.rviz` Display（无需手动 Add）+ planner 周期重发最近 MarkerArray（`markers_republish_period_sec` 默认 1.0s）；Display 仅预置操作员主入口 `arachne_lidar_fusion.rviz`（用户确认：0 基编号、预置+周期重发、全部视点、仅主入口配置） |

### 0.3 合理性评估

- ✅ **需求 1 符合常识**：IK 只保证关节解存在，不保证机械臂/末端不与环境碰撞或自碰撞。主链路（`grasp_preview_pipeline`/`teach_panel`）已有纯 numpy 几何碰撞（keepout 盒 + 末端盒 + 地面），在 IK 之后叠加碰撞过滤是视线规划"可达且安全"的正当补强，且与既有安全链路一致。
- ⚠️ **风险项 1（几何近似）**：碰撞几何为简化盒/球近似（无 Mesh 级精确判定、无 MoveIt FCL）。可能有过判/漏判，属于"规划前离线粗滤"精度，非真机安全兜底。**真机安全仍由运动链路的 keepout 挡板与 teach 门禁兜底**，本条仅在规划阶段做粗过滤，不改变既有运动安全机制。**`collision_free` 仅为规划参考/排序依据，不作为执行授权**——执行安全仍由运动链路的路径级 keepout 实时拦截负责（见 §3.1 路径安全、§3.4 实现要点）。
- ⚠️ **风险项 2（自碰撞无先例）**：仓库现有检查是"连杆点 vs 固定障碍盒"，**无连杆间自碰撞检查**。本阶段新增自碰撞属全新实现（用 `link_transforms` 各连杆对盒距离），需在计划中明确近似模型（连杆→圆柱/线段采样点，杆-杆最小距离 vs 半径和）。
- ⚠️ **风险项 3（共享几何常量漂移）**：keepout 盒默认值当前在 `teach_panel.py:73-77` 与 `grasp_preview_pipeline.py:451-454` **重复声明**。碰撞过滤复用会引入第三处。为**最小化本阶段改动与回归风险**，`viewpoint_planner` 以**节点参数**接收 keepout 盒（默认值与两处对齐并注明来源），**不改动既有 teach_panel/grasp_preview 声明**；共享抽取（统一为单一常量源）列为可选后续项（§6）。**注意**：teach_panel 的 keepout 盒是**节点参数**（`teach_panel.py:632-646`）、grasp_preview 是 **argparse 默认**（`grasp_preview_pipeline.py:451-454`），viewpoint_planner 再加**第三处节点参数**；三处默认值要逐一抄齐，任何一处现场被改（teach_panel 支持运行时改参）其余两处不会跟——这是本阶段接受的已知不一致。
- ⚠️ **风险项 4（新增，msg 接口破坏性变更）**：`Viewpoint.msg` 新增字段是**破坏性接口变更**（md5 变）。teach_panel 与 viewpoint_planner 是**两个独立进程**（teach_panel 还能 auto-start planner，见 `_call_viewpoint_plan_worker` `teach_panel.py:2307-2330`），`install/` 里的旧生成代码与运行中的旧实例**必须同停同启**，否则类型不兼容。Phase 4 必须跑全量 `build_workspace.sh` 并重启两个进程（`--dry-run-check` 不会重生成 msg）。
- ⚠️ **风险项 5（新增，下游消费链遗漏）**：第一阶段计划曾误判"`_serialize_viewpoint_plan` 遍历 Viewpoint 全字段、新增字段自动入盘"。实际 `teach_panel.py:469-478` 是**逐字段硬编码** dict，新字段不写就不进 `viewpoint_plan_*.json`；且下游 `_parse_viewpoint_plan_file`（`teach_panel.py:505`）白名单重解析会丢弃 `ik_joints` 外字段、`_render_viewpoint_rows`（`:5838`）只看 `ik_reachable`、`_execute_viewpoint_selected`（`:5873`）只挡 `ik_reachable`——**不改这些，`collision_free` 命中的点照样能被单点 Execute**。见 §1.5。
- ⚠️ **风险项 6（新增，路径安全）**：碰撞过滤只验证每个视点的**静态终点位形 q**，不查**点到点之间的运动路径**。`move_arm_to_joints_degrees` → `_move_arm_to_positions_velocity` 是速度流式、内部插值的运动，两点间路径可能撞 keepout。用户决策：**沿用运动链路 `_arm_keepout_violation`（含 `_arm_keepout_path` 沿途插值）实时拦截兜底**，计划补写拦截后的链行为（见 §3.4）。
- ⚠️ **风险项 7（新增，落盘体积）**：`/camera/points` 是 `create_cloud_xyz32` 全分辨率（640×480，发布端无 decimation），手写 ASCII `.pcd` 每点约 27 字节 → 单点约 **8.6 MB**，32 视点约 **275 MB**。采集侧已加点云 decimation 参数（§3.4），写盘耗时/磁盘仍需预算（见 §3.2、§3.4）。
- ✅ **需求 2 符合常识**：eye-in-hand 相机绕目标多视角采集是主动感知标准做法；复用现成运动入口（`move_arm_to_joints_degrees` + `on_done` 链）与 RGB 落盘范式（`capture_intrinsic_image`），depth/点云/camera_info 落盘为新增（无先例，见 §3.2）。

---

## 1. 现状调研（2026-08 只读代码审查结论，带 file:line）

### 1.1 viewpoint_planner 现有过滤管线与消息

- 包结构 `src/viewpoint_planner/`：`viewpoint_planner_node.py`（服务端）、`ik_filter.py`（纯 numpy IK 过滤）、`strategies/{base,fibonacci_sphere}.py`、`tf_utils.py`、`markers.py`、`test/`。
- **IK 过滤接口**（`viewpoint_planner/ik_filter.py`）：
  - `ik_filter.py:43` `class IkReachabilityFilter`，构造 `:52`（默认 `AuboI5Kinematics` + `AUBO_I5_JOINT_LIMITS`）。
  - `:62` `check(tool0_pose_aubo, q_seed) -> IkResult`（`solve_pose` + 关节限位校验 `:74`）。
  - `:81` `filter_poses(tool0_poses_aubo, q_seed, time_budget_sec) -> list[IkResult]`（串行播种 + 时间预算 `:96-107`）。
  - 依赖 `arachne_operator.kinematics`（`ik_filter.py:15`）。
- **调用位置**：节点 `_plan_inner` `viewpoint_planner_node.py:186-236`：采样(`:186`) → TF 变换(`:190`) → tool0 合成(`:204-212`) → **IK 过滤**(`:216-220`) → 构造 `Viewpoint`(`:222-235`)，`ik_reachable`/`ik_joints` 逐点标记(`:233-234`)。
  - **注意**：`filter_ik_reachable=false`（`viewpoint_planner_node.py:169`）时 `_plan_inner` **仍无条件执行 IK** 逐点标记，但不可达点 `ik_joints=[0.0]*6`（`:234`）——此数据**不能**用于碰撞检查（见 §2.4 组合矩阵）。
- **消息**：`src/viewpoint_planner_msgs/msg/Viewpoint.msg` 字段 `stamp/camera_xyz_base/camera_quat_xyzw/tool0_xyz_base/tool0_quat_xyzw/ik_reachable/ik_joints[6]`（`:7-8`）；**无碰撞字段，需新增**。
- `src/viewpoint_planner_msgs/srv/PlanViewpoints.srv`：请求含 `filter_ik_reachable`（`:6`，默认 true）；**无碰撞开关，需新增**（如 `filter_collision_free`）。

### 1.2 既有可复用碰撞能力（纯 numpy，无 ROS）

- `AuboI5Kinematics`（`src/arachne_operator/arachne_operator/kinematics.py`）：
  - `:98` `fk(q) -> 4x4`（tool0）、`:105` `link_transforms(q) -> list[(name, 4x4)]`（**8 项：基座 + 6 连杆 + tool0**，`kinematics.py:105-118`）——**自碰撞/几何判定基础**。
  - `:15` `REAL_ARM_JOINT_NAMES`、`:27` `AUBO_I5_JOINT_LIMITS`、`:48-94` 连杆/关节几何常量。
  - 该对象**本身不做碰撞**；碰撞判定靠外部用 `link_transforms` + `ee_collision` 包络 + keepout 盒。
- `src/arachne_operator/arachne_operator/ee_collision.py`（纯 numpy 无 ROS，teach_panel 与 grasp_preview 共用）：
  - `:74` `ee_camera_boxes_tool0(margin) -> list[box]`（tool0 系相机/支撑板 AABB）、`:107` `ee_envelope_points_tool0(margin)`（采样点）。
  - **历史教训**（`grasp_preview_pipeline.py:5733` 注释）：点采样对斜向穿透会漏检（2026-08-13 事故：support 板斜插篮筐沿，14 个采样点无一落入 keepout），故 teach_panel/grasp_preview 的末端包络判定**已改用盒-盒**——本阶段碰撞过滤同样**必须用盒-盒，不能只靠 `ee_envelope_points_tool0` 点采样**。
- `teach_panel.py` keepout 链路（实时运动拦截，纯 numpy）：
  - 默认常量 `teach_panel.py:73-77`：`DEFAULT_REAR_RACK_KEEPOUT_MIN/MAX`、`DEFAULT_FRONT_BASKET_KEEPOUT_MIN/MAX`（与 `grasp_preview_pipeline.py:451-454` 的 `--*.keepout-*.base` 默认一致）。
  - `:207` `class KeepoutBox`；参数声明 `:632-646`；判定函数 `_arm_keepout_violation`(`:4806`)、`_arm_keepout_ee_box_violation`(`:4835`)、`_arm_keepout_boxes`(`:4876`)、`_arm_keepout_path`(`:4895`)、`_arm_keepout_samples`(`:4905`)。
  - `_arm_keepout_boxes`(`:4876`) 只返回 **rear_rack + front_basket 2 个盒**（**无车体盒**）；`_arm_keepout_path`(`:4895`) 是当前位姿到目标的**关节空间插值采样**（运动路径 keepout 预检）。
  - `arm_base_xyz`/`arm_base_rpy`（`teach_panel.py:633-634`）即 base_link→aubo_base_link 变换，teach_panel 侧几何判定在 base_link 系。
- `grasp_preview_pipeline.py` 更完整的 numpy 几何算法（**类方法，未抽离**，需参数化抽为独立函数）：
  - `:258` `CollisionBox` dataclass、`:1032` `_make_collision_boxes()`（**4 盒**：`scout_base_main` + `scout_base_center_ridge` 两个车体盒 + `front_basket_keepout` + `rear_sensor_rack`，车体/basket 用 `collision_margin=0.015`、rear_rack 用 `rear_rack_collision_margin=0.005`，见 `:1033/:1062`）。
  - `:5777` `_box_box_clearance`（AABB 距离/渗透）、`:5794` `_point_box_clearance`。
  - `:5684` `_arm_collision_clearance_base(q, base_from_aubo)`（`link_transforms` 连杆点 + `arm_collision_radius=0.035` + 地面 `ground_min_z_base=-0.22` + `ground_clearance=0.02` + 末端偏心 ee 盒，采样密度 `arm_collision_samples_per_link=6`，默认见 `grasp_preview_pipeline.py:457-474`）。
  - 这些是**类方法**，深度绑定 `self.args/self.kinematics/self.collision_boxes`，不能直接 import 复用。

### 1.3 一键采集所需现成机制

- **运动入口（串行链路天然勾子）**：`teach_panel.py` `TeachPanelNode.move_arm_to_joints_degrees(degrees, on_done=None)`（`:3409`）→ `_start_worker`(:4600) → `_move_arm_to_joints_worker`(:3460) → `_move_arm_to_positions_velocity`(:3505，阻塞式速度流式 + keepout `:3509`)；到位 = 循环 `max_error<=tolerance` 返回 True（`:3538`），随后 `finally` 触发 `on_done()`（`:3414-3422`）。**链式 `on_done` 即"顺序运动"的天然实现。**
  - **关键缺陷（第二轮新增）**：`move_arm_to_joints_degrees` 的 `on_done` 在 `finally` 里**无条件触发**（`:3414-3422`），**无法区分"到位成功"与"keepout 拦截/超时失败"**；`_move_arm_to_joints_worker` 无返回值。一键采集需要成败信号（见 §3.4 实现要点：改入口传结果位）。
- **相机帧缓存 + RGB 落盘先例**：
  - `TeachPanelNode` 已订阅 `camera_image_topic`（默认 `/camera/color/image_raw`，参数 `:665`，订阅 `:971-975`），回调 `_intrinsic_image_cb`(`:1278-1280`) 原子缓存 `self.intrinsic_latest_image`（bgr8 `Image`，字段 `:1014`）。
  - `capture_intrinsic_image`(`:2513-2571`)：取最近帧 → 校验 bgr8/几何 → `cv2.imwrite` PNG + manifest（`_intrinsic_write_manifest` `:2493`）。**depth/点云/camera_info 无此先例，需新增。**
- **相机发布话题**（`src/arachne_sensors/arachne_sensors/gemini335_v4l2_node.py`）：
  - `/camera/color/image_raw`（`:305`，`sensor_msgs/Image` bgr8）、`/camera/color/camera_info`（`:306`）。
  - `/camera/depth/image_raw`（`:307`，`Image`，encoding=`16UC1`（`_publish_depth` `:767`，**单位 mm**，Z16 原始），`depth_scale=0.001` 用于 mm→m 换算）、`/camera/depth/image_color`（`:308`）、`/camera/depth/camera_info`（`:309`）。
  - `/camera/points`（`:311`，`PointCloud2` xyz32，`create_cloud_xyz32` **全分辨率 640×480**，`_publish_pointcloud` `:790`，发布端**无 decimation**；`pointcloud_decimation=6` 参数 `:189` 不作用于发布端）。launch 全开（`gemini335.launch.py:167-181`）。
  - QoS 均为 `create_publisher(..., 10)`（`gemini335_v4l2_node.py:305-311`），teach_panel 订阅 `camera_image_topic` 用裸 `10`（`teach_panel.py:971-975`），兼容。
  - **注意**：以上为相机节点默认话题；若实际部署经 RViz/别处重映射以 launch 为准，采集订阅话题沿用 teach_panel 的 `camera_image_topic` 参数并新增 depth/points/camera_info 参数。
- **数据落盘目录**：`TeachPanelNode.runtime_log_dir`（`_make_runtime_log_dir` `:1069-1079`，`<runtime_log_root>/<YYYYmmdd_HHMMSS>` + `latest` 软链），用于 `events.jsonl`（`:1029`）、`viewpoint_plan_*.json`（`:2398`）、受管进程 log。采集落盘宜在其下建子目录（如 `viewpoint_acquisition/`）。
- **事件记录**：`_write_event(kind, message, **fields)`（`:1081-1093`，append JSON 行到 `events.jsonl`，`sort_keys=True`）。Viewpoint plan 结果 JSON 序列化 `_serialize_viewpoint_plan`、写入 `_write_viewpoint_plan_result`（`teach_panel.py`，模块级纯函数可离线单测）。
- **Viewpoint tab 现状**：`_build_viewpoint_tab`(`:5617-5665`) → `_build_viewpoint_service_section`(`:5667`) / `_build_viewpoint_plan_params_section`(`:5687`，含 `_plan_viewpoint_from_tab` `:5727`) / `_build_viewpoint_results_section`(`:5786`，toolbar `Load JSON` `:5836` / `Execute`，`ttk.Treeview` 五列 `index/reach/camera/tool0/joints`，`_render_viewpoint_rows` `:5845-5857`)。单点执行 `_execute_viewpoint_selected`(`:5873-5919`)，busy 门禁 `_viewpoint_execute_busy` + `_set_viewpoint_execute_enabled`(`:5859`)，on_done 恢复 `_viewpoint_execute_done`(`:5921`)。
  - **第二轮修正**：`_render_viewpoint_rows` 实际在 `:5838`（`:5845` 是循环内取 joints），`_execute_viewpoint_selected` 在 `:5873`，`_viewpoint_execute_done` 在 `:5921`。Treeview "reach" 列只显示 `yes/no`（`:5852`），无碰撞维度。

### 1.4 二次过滤对应字段（核心：两字段独立）

- **既有 JSON 字段（IK 段）**：`Viewpoint.ik_reachable`(bool)、`Viewpoint.ik_joints[6]` —— 由 IK 过滤逐点标记（`viewpoint_planner_node.py:233-234`），teach_panel 落盘 `viewpoint_plan_*.json` 由 `_serialize_viewpoint_plan`（`teach_panel.py:461-494`）**逐字段显式序列化**（`:469-478` 是硬编码 dict，含 `stamp/camera_*/tool0_*/ik_reachable/ik_joints`）。
- **新增 JSON 字段（碰撞段）**：`Viewpoint.collision_free`(bool)、`Viewpoint.collision_clearance`(float，最小净距/渗透量，便于调参)。
  - **第二轮修正**："全部 Viewpoint 字段自动进 JSON"**不成立**——`_serialize_viewpoint_plan` 是逐字段硬编码，新增字段**必须显式补进 `:469-478`**，否则不进 `viewpoint_plan_*.json`；且下游重解析/展示/执行链（§1.5）也要同步改。
- **最终可达判定**：`ik_reachable && collision_free`。UI / 一键采集 / results 列表的"可达行"语义需相应调整（见 §1.5、§3）。

### 1.5（新增）ik_joints 下游消费链：新增字段必须同步的 6 处

`Viewpoint` 从 planner 产出到被执行，经过一条**逐字段手工对接**的消费链。新增 `collision_free`/`collision_clearance` 后，下列每一处都必须显式改动，否则字段在某一环被静默丢弃、或碰撞命中的点仍被执行：

| 环节 | 位置 | 现状 | 本阶段改动 |
| --- | --- | --- | --- |
| ① 服务端标记 | `viewpoint_planner_node.py:222-235` | 只标 `ik_reachable`/`ik_joints` | IK 后插碰撞过滤，逐点标 `collision_free`/`collision_clearance`（§2.3） |
| ② 序列化落盘 | `_serialize_viewpoint_plan` `teach_panel.py:461-494` | **逐字段硬编码 dict**（`:469-478`） | 显式补 `collision_free`/`collision_clearance` 两键 |
| ③ 重解析 | `_parse_viewpoint_plan_file` `teach_panel.py:505-543` | **白名单**重建行，只保留 `ik_reachable/camera/tool0/ik_joints_*`（`:526-535`） | 白名单放行 `collision_free`/`collision_clearance`；旧 JSON 缺 `collision_free` → 默认 `true`（向后兼容，§3.4） |
| ④ 列表渲染 | `_render_viewpoint_rows` `teach_panel.py:5838-5857` | "reach" 列只看 `ik_reachable`（`:5850`） | "可达"语义改 `ik_reachable && collision_free`；列/标签体现碰撞维度（如 `yes/no` → `ik/col/ok` 或加列） |
| ⑤ 单点执行门禁 | `_execute_viewpoint_selected` `teach_panel.py:5873-5919` | 只挡 `not row["ik_reachable"]`（`:5891-5894`） | 同时挡 `collision_free=false` 的行（防碰撞命中点被单点 Execute） |
| ⑥ 状态/事件统计 | `call_viewpoint_plan` `teach_panel.py:2368-2372`、`_load_viewpoint_result` `:5774-5783` | 只统计 `ik_reachable`（`:5781`） | 状态行/事件补 `collision_free_count` / `reachable_count=ik&&collision` |

**漏改后果**：只改 ①② 不改 ③-⑥，则 JSON 有碰撞字段但 UI/执行仍按 `ik_reachable` 单字段，碰撞命中点可被单点 `Execute`/一键采集执行——与需求"两字段独立生效"直接冲突。

---

## 2. 需求一：碰撞过滤（IK 之后叠加）

### 2.1 整体设计

- 管线变为：采样(`strategy.plan`) → TF/`tool0` 合成 → **IK 过滤**(`filter_poses`，标记 `ik_reachable`) → **碰撞过滤**（仅对 `ik_reachable` 的点，标记 `collision_free`) → 构造 `Viewpoint`。
- 不可达（无关节解）的点**不进入碰撞检查**（无 q 可查），`collision_free` 默认 `false`（或 `null` 语义置 false，计划采用 false 以减少消息歧义）。
- 碰撞过滤**不删除采样点**（与 IK 逐点标记语义一致）：保留全部点，逐点标记 `collision_free`。

### 2.2 碰撞检查内容（用户确认 A 级完整）

对每个待检点的关节解 `q`（`aubo_base_link` 系，需经 base_link→aubo_base_link 变换后的同一坐标系），检查：

1. **环境 keepout 盒**（**用户确认：含车体，对齐 `grasp_preview._make_collision_boxes` 4 盒**——`scout_base_main` + `scout_base_center_ridge` + `front_basket_keepout` + `rear_sensor_rack`，车体/basket 用 `collision_margin=0.015`、rear_rack 用 `rear_rack_collision_margin=0.005`；**非** teach_panel `_arm_keepout_boxes` 的 2 盒（无车体）。`aubo_base_link` 或 base_link 系，统一到 base_link 系判定，仿 `grasp_preview`）：
   - 各连杆包络点（`link_transforms` 得连杆位姿 → 连杆上采样点，半径 `arm_collision_radius=0.035`，采样密度 `arm_collision_samples_per_link=6`）与各盒 `point_box_clearance`；
   - 末端偏心相机盒（`ee_camera_boxes_tool0(margin)` → 经 tool0 变换到世界）与盒 `box_box_clearance`（**必须盒-盒，不能只靠点采样**，见 §1.2 历史教训）。
2. **地面**：连杆/末端最低点 `z` 不得低于 `ground_min_z_base=-0.22`（`+ ground_clearance=0.02` + `arm_collision_radius`，即 `grasp_preview_pipeline.py:5699-5701` 的 `ground_threshold` 口径）。
3. **机械臂自碰撞**（全新）：相邻连杆豁免，非相邻连杆对用 `link_transforms` 各连杆采样段，两段最小距离 vs 两半径之和；若任何一对 < 阈值判碰撞。**连杆→线段近似**需在计划中明确（见 §2.4 精度说明）。

> 降级选项（若 A 级引入回归/耗时过高可回退）：只做 ①②（keepout+地面）不做自碰撞，作为参数/开关保留扩展位，默认开 A 级。

### 2.3 拆分与对接

| 交付 | 说明 |
| --- | --- |
| `src/viewpoint_planner/viewpoint_planner/collision_filter.py`（新） | `CollisionFilter`，纯 numpy 无 ROS，仿 `ik_filter`：`check(q, base_from_aubo, world_boxes,...) -> CollisionResult`（`collision_free` + `collision_clearance`）；`filter_joints(ik_results, q_seed..., params...) -> list[Viewpoint-ish]`。可离线单测 |
| 碰撞几何纯函数（新，`viewpoint_planner/geometry.py`） | 把 `grasp_preview_pipeline` 的 `_box_box_clearance`/`_point_box_clearance`/`_arm_collision_clearance_base` **参数化抽为独立函数**（不再依赖 `self`）；keepout 盒默认常量落本模块，节点以参数可覆盖 |
| 自碰撞检查（新） | `link_transforms`（8 项）→ 各连杆采样线段 → 非相邻杆-杆最小距离 vs 半径和（详 §2.4） |
| `Viewpoint.msg` 增加字段 | `bool collision_free` + `float64 collision_clearance`（**默认两者都加**，便于调参）。**破坏性接口变更**：md5 变，teach_panel/planner 两进程须同停同启 + 全量 rebuild（见 §0.3 风险项 4） |
| `PlanViewpoints.srv` 增加开关 | 请求 `bool filter_collision_free`（默认 true，语义同 `filter_ik_reachable`：`filter = request or 节点参数`，见第一阶段 Phase 4 注释） |
| `viewpoint_planner_node.py` 接线 | IK 过滤后插入碰撞过滤，逐点标记两字段；`_plan_inner` 传 keepout 参数 |
| **teach_panel 下游消费链同步（第二轮展开，6 处）** | 按 §1.5 表逐处改：② `_serialize_viewpoint_plan`（`teach_panel.py:469-478`）显式补两字段；③ `_parse_viewpoint_plan_file`（`:505-543`）白名单放行 + 旧 JSON 缺 `collision_free` 默认 `true`；④ `_render_viewpoint_rows`（`:5838`）"可达"=`ik_reachable && collision_free`；⑤ `_execute_viewpoint_selected`（`:5873`）同挡 `collision_free=false`；⑥ `call_viewpoint_plan`（`:2368`）/ `_load_viewpoint_result`（`:5774`）状态行补碰撞统计 |
| **RViz Marker（第二轮新增）** | `markers.py:100-102` 当前 `ik_reachable ? 绿 : 红`；新增碰撞维度后建议：绿=`ik_reachable && collision_free`，红=ik 不可达，黄/橙=ik 可达但碰撞命中（便于现场直观看清"碰撞过滤滤掉了哪些点"） |

### 2.4 实现要点

- **坐标系**：IK 过滤在 `aubo_base_link` 系（tool0 目标先经 base_link→aubo_base_link，`tf_utils.py`）。碰撞过滤**统一到 base_link 系判定**（keepout 盒/地面/scout 车体盒的默认常量都是 base_link 系，teach_panel 侧 `arm_base_xyz`/`arm_base_rpy` 即 base_link→aubo_base_link）：把 `q` 经 `base_from_aubo` 变换后在 base_link 系做判定（仿 `grasp_preview` 的 `base_from_aubo`，`grasp_preview_pipeline.py:5684`）。
- **自碰撞近似**：`link_transforms` 返回 **8 项**（基座 + 6 连杆 + tool0，`kinematics.py:105-118`）。每连杆用相邻两个关节位姿成线段（基座→L1→...→L6→tool0，共 7 段），非相邻线段对求最小距离 `d`，若 `d < r_i + r_j`（各连杆近似半径，默认对齐 `arm_collision_radius=0.035`）判碰撞；**相邻线段豁免**（共享关节点）。**注意**：aubo 的 L1/L2 有偏置（`kinematics.py:48-94`）、tool0 有固定 `Rz(90°)`（`TOOL0_FIXED_RPY`），"哪些算相邻"按线段在链上的索引差 ≥2 判定，而非按名字猜。半径/采样数作参数。**注明是保守/近似**，不替代真机安全。
- **耗时控制**：碰撞过滤是几何运算，应远快于数值 IK。若 A 级全量（含自碰撞）过慢，可给 `collision_time_budget_sec`（默认宽松，如 5s），超预算剩余点标记 `collision_free=false` 不再判（**与 IK 预算 `:214-220` 的"超预算剩余点标不可达"语义一致**）。
- **开关语义（2×2 组合矩阵，第二轮新增）**：

| `filter_ik_reachable` | `filter_collision_free` | 行为 |
| --- | --- | --- |
| true | true | 全量：IK 逐点标记 + 碰撞逐点标记；最终"可达"=`ik_reachable && collision_free` |
| true | false | IK 逐点标记；碰撞仍逐点 `check` 并标记（保持 JSON 二次字段有值），但最终"可达"退化为单看 `ik_reachable` |
| false | true | IK 仍逐点标记（`_plan_inner` 无条件执行），但**不可达点 `ik_joints=[0.0]*6`（`viewpoint_planner_node.py:234`）**——这些点**不做碰撞检查**（无有效 q），`collision_free=false`；仅对 `ik_reachable=true` 的点做碰撞检查；最终"可达"=`ik_reachable && collision_free` |
| false | false | 两过滤都只做标记不做过滤；最终"可达"=`ik_reachable` |

  即：**碰撞检查永远只对有有效 `ik_joints` 的点进行**；`filter_collision_free` 只控制"碰撞字段是否参与最终可达判定"，不控制是否执行碰撞检查（与 IK 段"关闭过滤仍逐点标记"语义对齐）。

- **回归防护**：新增/改动仅限 `viewpoint_planner`（+ 一个共享 geometry 模块，可能放 `arachne_operator` 因 `kinematics`/`ee_collision` 已在该包被引用）。不动 teach_panel 运动链路 / grasp_preview 类方法（避免回归）——共享抽取列可选项 §6。

---

## 3. 需求二：一键采集（teach_panel Viewpoint tab）

### 3.1 交互与门禁（用户确认：沿用现有链路 + 总确认框）

- Viewpoint tab 结果区 toolbar（与 `Execute` 并列）新增 **`Acquire All`（一键采集）** 按钮。
- 点击 → 收集当前已加载结果中 `可达` 的行（`ik_reachable && collision_free`，且 `ik_joints` 有效）→ 弹**总确认框**（展示：将依次访问的视点数 N、目标 `camera_xyz_base`、每个点的 `tool0_xyz` 摘要、6 关节角）→ 确认后启动链式执行；取消则不动作。
- 链式执行：`move_arm_to_joints_degrees(joints[0], on_done=采集第 0 点并驱动 joints[1])`，采集回调内写盘 → 驱动下一个；全部完成后 `on_done` 收尾（事件 + 状态 + 释放 busy）。
- **busy 门禁（第二轮修正）**：**不复用** `_viewpoint_execute_busy`——该标志在 `_viewpoint_execute_done`（`:5921`）里会把 UI 恢复使能，链式采集若复用，**每个点结束都会误触发"完成"恢复 UI**。新增独立 `_viewpoint_acquire_busy` 标志 + 禁用 `Execute`/`Acquire All`/Treeview（`_set_viewpoint_execute_enabled`），全部完成后才恢复；keepout 检查保持开启。
- **GUI 更新回 UI 线程**：采集回调在 worker 线程，状态/进度更新用 `self.root.after(0, ...)`（参照 `:5914-5918`）。
- **运动路径安全（第二轮新增，用户决策：实时拦截兜底）**：碰撞过滤只验证每个视点的**静态终点位形**，不查**点到点之间的运动路径**。一键采集是**连续多运动点**，点到点路径可能撞 keepout。**沿用运动链路 `_arm_keepout_violation`（含 `_arm_keepout_path` 沿途插值）实时拦截兜底**：某点运动被 keepout 拦截（`_move_arm_to_positions_velocity` 返回 False）→ 跳过该点采集，记录 error，继续下一视点。**`collision_free` 仅为规划参考/排序依据，不作为执行授权**——执行安全由运动链路的路径级 keepout 实时拦截负责。

### 3.2 每个视点采集的信息与保存（用户确认：全采）

| 信息 | 数据源 | 当前先例 | 落盘 |
| --- | --- | --- | --- |
| RGB 彩色图 | `node.intrinsic_latest_image`（bgr8 `Image`，已缓存） | ✅ `capture_intrinsic_image` PNG | `<acq>/<viewpoint>/color_<ts>.png`（bgr8 3 通道，校验 `step==width*3`，仿 `capture_intrinsic_image:2535`） |
| 深度图 | **新增订阅** `/camera/depth/image_raw` → `node.intrinsic_latest_depth`（`Image`，encoding=`16UC1` **单位 mm**） | ❌ 无 | `<acq>/<viewpoint>/depth_<ts>.png`（**16bit PNG 直存 mm 值**——订阅到的本来就是 `16UC1`(mm)，`np.frombuffer(dtype=np.uint16)` 直接写，**不再除 depth_scale**；`depth_scale=0.001`（mm→m）记录进清单供离线下载） |
| 点云 | **新增订阅** `/camera/points` → `node.intrinsic_latest_points`（`PointCloud2` xyz32 **全分辨率 640×480**） | ❌ 无 | `<acq>/<viewpoint>/cloud_<ts>.pcd`（**手写 ASCII `.pcd`**，不新增第三方依赖；用 `sensor_msgs.point_cloud2.read_points` 遍历写头+数据。**体积预算**：全分辨率单点约 8.6 MB、32 视点约 275 MB——采集侧加 `pointcloud_decimation` 参数（默认对齐发布端 `6`）降采样后再写盘） |
| 相机内参 | **新增订阅** `/camera/color/camera_info`、`/camera/depth/camera_info` | ❌ 无 | 写入清单 JSON 的该视点条目 |
| 关节角 / tool0 / camera 位姿快照 | 当前关节角（`_current_arm_vector` `:4583`）+ 该视点 `tool0_xyz_base/tool0_quat_xyzw/camera_xyz_base/camera_quat_xyzw` | ✅ 已在 plan JSON | 写入清单 JSON 的该视点条目 |

### 3.3 落盘结构与清单（2026-08 补强：批次分层）

```
<runtime_log_dir>/viewpoint_acquisition/<run_id>/     # run_id = 采集启动时间戳（同秒追加 _2/_3）
    manifest.json                                    # 本批清单：run_id / session_stamp / 目标 / 视点顺序表
    <视点号>/
        color_<ts>.png
        depth_<ts>.png
        cloud_<ts>.pcd
    <视点号>/
        ...
<runtime_log_dir>/viewpoint_acquisition/latest -> <run_id>   # 最新一批（仿会话层 latest 软链）
```

- 每次 `Acquire All` 生成**独立批次目录**（`make_viewpoint_run_id(root, now=None)` 纯函数，时间戳命名 + 同秒去重，可离线单测）；`viewpoint_acquire_begin` 返回的 `root` 即批次目录，`viewpoint_acquire_point` 无需感知批次层（`vp_dir = root/<视点号>` 自动落入本批）。
- 清单 `manifest.json`：`{stamp, run_id, session_stamp, depth_scale, target_xyz_base, num_viewpoints, total_reachable, viewpoints: [{index, tool0_xyz/quat, camera_xyz/quat, ik_joints_deg, joint_angles_snapshot, color/depth/cloud 相对路径, color_camera_info, depth_camera_info, acquired_stamp}]}`。
- 每个视点执行状态/结果写 `events.jsonl`：`_write_event("viewpoint_acquire_point", ..., index=..., path=..., error=...)`；整轮完成 `_write_event("viewpoint_acquire_all", ...)`。
- **中途异常容错**：某点采集/写盘失败不应中断整轮（记录 error 继续下一视点）——采集失败与运动失败分开处理；运动到位失败（`_move_arm_to_positions_velocity` 返回 False，含 keepout 拦截/超时）跳过采集，记录后继续下一视点（默认；可在总确认框说明该行为）。
- **运动成败信号（第二轮新增，用户决策：改入口传结果位）**：`move_arm_to_joints_degrees` 的 `on_done` 在 `finally` 里**无条件触发**（`:3414-3422`），无法区分"到位成功"与"keepout 拦截/超时失败"。**改运动入口**：给 `move_arm_to_joints_degrees` 增加可选 `on_success(success: bool)` 回调（内部根据 `_move_arm_to_positions_velocity` 返回 True/False 传递），向后兼容现有单点 `Execute`（单点仍用 `on_done` 恢复 UI）。一键采集用 `on_success` 决定是否采集。

### 3.4 实现要点

- **采集模块化 & 可离线单测**：新增模块级纯函数（teach_panel.py 内或独立 `viewpoint_acquisition.py`）：
  - `serialize_pcd(points_xyz: np.ndarray) -> str`（ASCII .pcd 头 + 数据，可单测头字段/点数）。
  - `viewpoint_manifest_entry(...)` / `manifest_append(...)`（校验字段、路径、round-trip 可测）。
  - 图像尺寸/编码校验复用 `capture_intrinsic_image` 的校验思路（`teach_panel.py:2529-2541`）。
  - **深度→16bit PNG（第二轮修正）**：订阅到的 depth 是 `16UC1`（**单位 mm**），`np.frombuffer(dtype=np.uint16)` 直接写 16bit PNG，**不再除 `depth_scale`**（`depth_scale=0.001` 是 mm→m 换算，记录进清单供离线下载）。若误把 mm 当 m 再除 0.001 会溢出/错 1000 倍。
  - 点云帧若为 `None`（相机没起来）→ 该点跳过点云并记 error，不中断。
  - **点云 decimation（第二轮新增）**：`/camera/points` 发布端全分辨率 640×480 无 decimation（§1.3），采集侧加 `pointcloud_decimation` 参数（默认对齐发布端 `6`），`read_points` 后按步长降采样再写盘，控制单点 pcd 体积（全分辨率约 8.6 MB → decimation=6 约 0.24 MB）。
- **新增订阅参数**：`depth_topic`（默认 `/camera/depth/image_raw`）、`points_topic`（默认 `/camera/points`）、`color_camera_info_topic`、`depth_camera_info_topic`（默认 `/camera/{color,depth}/camera_info`），缓存字段 `intrinsic_latest_depth/points/color_info/depth_info` + 原子赋值（仿 `:1278-1280`）。**不依赖 debug 工具**（这些都是主链路相机话题）。
- **"可达"语义统一**：`Acquire All` 收集 `ik_reachable && collision_free` 的行；若已加载旧 JSON（无 `collision_free` 字段），容错按 `collision_free 缺失 → 视为 true` 处理（向后兼容，§1.5 ③）。
- **门禁/安全红线**：沿用 `move_arm_to_joints_degrees` 链路（keepout 保留），不新增直接运动入口；一键采集是连续多运动点，总确认框为唯一额外人工确认；不绕过 teach/control owner 门禁逻辑现状（该链路本身不走这两门禁，与单点 Execute 一致——见 §1.3 调研）。

---

## 4. 实现阶段（本阶段规划，实施时按此执行并回写）

### Phase 1：接口与字段扩展（需求一前置）—— ✅ 已实现并通过实机测试验收（2026-08，用户确认）

- ✅ `src/viewpoint_planner_msgs/msg/Viewpoint.msg` 增 `bool collision_free` + `float64 collision_clearance`（两者都加）。
- ✅ `src/viewpoint_planner_msgs/srv/PlanViewpoints.srv` 请求增 `bool filter_collision_free`（默认 true 语义同 IK 段）。
- ✅ **破坏性接口变更**：全量构建已通过（`viewpoint_planner_msgs` 重生成，新字段经 `ros2` import 确认）；teach_panel 与 viewpoint_planner 两进程重建完成。**实机验收通过**：两进程同停同启后 service 可调用、无类型不兼容报错（用户确认）。
- ✅ **teach_panel 下游消费链同步 6 处**：`_serialize_viewpoint_plan` 显式补两键 + 顶层 `collision_free_count`/`reachable_count`；`_parse_viewpoint_plan_file` 白名单放行两字段 + 旧 JSON 缺 `collision_free` 默认 `true`；`_render_viewpoint_rows` reach 列三态 `yes/collision/no`（橙色 highlight）；`_execute_viewpoint_selected` 同挡 `collision_free=false`；`call_viewpoint_plan`/`_load_viewpoint_result` 状态行与事件补碰撞统计；Viewpoint tab 新增 **Filter Collision** 勾选 + 参数/事件。round-trip 验证通过（新字段入 JSON、解析还原、旧 JSON 兼容）。

### Phase 2：碰撞过滤 —— ✅ 已实现并通过实机测试验收（2026-08，用户确认）

- ✅ 新建 `viewpoint_planner/geometry.py`：`CollisionBox`/`box_box_clearance`/`point_box_clearance`/`box_from_center_size`/`box_from_min_max`/`default_collision_boxes`（含车体 4 盒 + 两 margin）/`arm_collision_clearance_base`/`self_collision_clearance`（非相邻杆对距离 + 相邻豁免）。
- ✅ 新建 `viewpoint_planner/collision_filter.py`：`CollisionFilter.check -> CollisionResult` + `filter_joints(...)`；A 级范围（keepout+地面+末端盒+自碰撞）；`collision_time_budget_sec`；2×2 开关语义（碰撞检查只对有有效 `ik_joints` 的点，`filter_collision_free` 只控制是否参与最终可达判定）。
- ✅ `viewpoint_planner_node.py`：`_plan_inner` IK 后插碰撞过滤逐点标记两字段；`invert_rigid(base_in_aubo)` 统一到 base_link 系；keepout/地面/半径以节点参数传入并可 launch 覆盖（`_resolve_collision_params`）。
- ✅ `markers.py`：`_viewpoint_arrow` 三色——绿=`ik && col`、橙=`ik` 且碰撞、红=不可达。
- ✅ 单测：`test_collision_filter.py` + `test_geometry.py` **17 项全绿**（盒内判碰撞、地面以下判碰撞、自碰撞触发/相邻豁免、不可达点跳碰撞、预算跳过、盒几何正确性）。
- ✅ 集成：`viewpoint_planner --dry-run-check`、`teach_panel --headless-check` 均通过。

### Phase 3：一键采集 —— ✅ 已完成（代码层 + 离线验证；真机验收待用户确认）

- ✅ `TeachPanelNode`：新增 `depth/points/camera_info` 订阅与缓存字段（`depth_topic`/`points_topic`/`color_camera_info_topic`/`depth_camera_info_topic` + `_intrinsic_depth_cb`/`_intrinsic_points_cb`/`_intrinsic_color_info_cb`/`_intrinsic_depth_info_cb`）。
- ✅ 采集落盘纯函数（模块级，duck-typed 可离线单测）：`serialize_pcd`(ASCII .pcd 头/点数)、`pointcloud_xyz_hw`(纯 numpy 解析 create_cloud_xyz32，不依赖 `sensor_msgs_py`)、`decimate_cloud_points`(按步长降采样 + NaN/Inf 过滤)、`depth_image_to_uint16`(16UC1 mm 直接写 16bit，不除 `depth_scale`)、`camera_info_to_dict`（K/R/P/D/ROI）。
- ✅ **改 `move_arm_to_joints_degrees` 增加可选 `on_success(success: bool)` 回调**：`_move_arm_to_joints_worker` 改为返回 `_move_arm_to_positions_velocity` 的 True/False，`on_success` 存在时优先、否则回退旧 `on_done`——**向后兼容单点 `Execute`**。
- ✅ `TeachPanelApp`：Viewpoint tab 新增 **Acquire All** 按钮 + 总确认框（列出 N/目标/每点 tool0+joints）+ `_acquire_start_motion`/`_acquire_on_motion_done`(`on_success` 链，成功才采集，keepout 拦截/超时跳过并记录) + `_acquire_finish`；**独立 `_viewpoint_acquire_busy`** 门禁（**不复用** `_viewpoint_execute_busy`）；GUI 更新走 `root.after`。
- ✅ 落盘：`runtime_log_dir/viewpoint_acquisition/<n>/{color_<ts>.png,depth_<ts>.png,cloud_<ts>.pcd}` + `manifest.json`（每点写后即时刷新）+ `events.jsonl`(`viewpoint_acquire_point`/`viewpoint_acquire_all`)。
- ✅ 单测：`test_viewpoint_acquisition.py` 13 项（pcd 头/点数、深度 roundtrip/编码/几何、点云解析/坏布局、decimate+NaN 过滤、camera_info roundtrip）+ `test_markers` 补 `collision_free` 字段修复 Phase 2 遗留 6 处失败。
- 约束：不动运动安全链路；不新增第三方依赖；不脱离 teach_panel 现有入口（真机运动仍需总确认框）。

### Phase 4：测试与验收 —— ✅ 全部完成并【实机验收通过】（用户 2026-08 确认）

- ✅ `python3 -m py_compile`（改动 python）、`viewpoint_planner --dry-run-check`、`teach_panel --headless-check`、**全量 `./scripts/build/build_workspace.sh`**（msg 重生成确认）、17 个离线单测全绿。
- ✅ 相关回归：`grasp_task_server`/`road_cleanup_task_server`/`step_cleanup_demo --dry-run-check`（因 `kinematics`/`ee_collision` 共享，确认无回归）。
- ✅ 真机项（碰撞过滤相关，**用户 2026-08 已确认通过**）：两进程同停同启后 service 可调用、无类型不兼容；碰撞过滤对真实目标输出 `collision_free` 逐点标记合理；RViz 三色箭头抽查通过。
- ✅ 真机项（一键采集相关，**用户 2026-08 已确认通过**）：`Acquire All` 顺序运动到各可达视点并逐点落盘 RGB/depth/点云/内参/位姿 + `manifest.json` 完整——此前暂停的“运动 timeout（首轮只采到视点 0）+ 点云未发布 + 碰撞异常”均由用户修复后复测通过。
- ✅ `git status --short` 检查（本轮改动 5 modified + 4 new，无残留）。

### Phase 5（补强，2026-08，已实现并【实机验收通过】）

- ✅ **批次落盘区分**：每次 `Acquire All` 独立批次目录 `<session>/viewpoint_acquisition/<run_id>/`；manifest 增 `run_id`/`session_stamp`；`viewpoint_acquisition/latest` 软链；`viewpoint_acquire_all/point/skip` 四事件带 `run_id`——同一面板会话多次采集互不覆盖、可事后关联（+3 项单测：时间戳形状/同秒去重/默认时间）。
- ✅ **RViz 视点编号 + 常驻预览**：`markers.py` 新增 `_viewpoint_label`（`TEXT_VIEW_FACING`，0 基编号、三色同箭头、ns=`viewpoint_labels`、相机原点上方偏移/字号参数化）；节点缓存最近 MarkerArray 并按 `markers_republish_period_sec`（默认 1.0s）周期重发；`arachne_lidar_fusion.rviz` 预置 `/arachne/viewpoint/markers` Display（Transient Local）——RViz 启动即自动显示、planner/RViz 重启后仍常驻、无需手动 Add（+5 项单测，含 republish 缓存行为）。
- ✅ **Viewpoint 列表列宽修复**：Treeview 列宽误用 5/8/24/24/32 px 导致数据挤在左上角，改为 40/72/140/140/240 px 且 joints 列 stretch 占满剩余宽度。
- ✅ 验证：py_compile / 离线单测（`arachne_operator` 采集 16 项、`viewpoint_planner` 64 项）/ 全量构建（17 pkg）/ `teach_panel --headless-check` / `viewpoint_planner --dry-run-check` 全绿；rviz YAML 解析确认新 Display 就位；【实机验收通过】（用户 2026-08 确认）。

---

## 5. 验收标准（本阶段）

- [x] （碰撞）`Viewpoint.msg` 含 `collision_free`（+`collision_clearance`）、`PlanViewpoints.srv` 含 `filter_collision_free`；**全量构建通过**（msg 重生成）✅；teach_panel + viewpoint_planner **两进程同停同启后 service 可调用、无类型不兼容报错** ✅（**实机验收通过，用户确认**）。
- [x] （碰撞）IK 过滤后叠加碰撞过滤，JSON 中 `ik_reachable` 与 `collision_free` **两个字段独立逐点标记**，不互相覆盖；2×2 开关组合矩阵语义符合 §2.4（碰撞检查永远只对有有效 `ik_joints` 的点进行）。—— 代码实现 + 离线单测/round-trip 验证
- [x] （碰撞）A 级检查（keepout+地面+末端盒+自碰撞）离线单测通过；不可达点不进入碰撞检查且 `collision_free=false`。—— test_geometry/test_collision_filter 17 项全绿
- [x] （碰撞）`_serialize_viewpoint_plan` 显式带 `collision_free`/`collision_clearance`；`_parse_viewpoint_plan_file` 白名单放行两字段；`_render_viewpoint_rows`/`_execute_viewpoint_selected`/`call_viewpoint_plan` 的"可达"语义 = `ik_reachable && collision_free`（碰撞命中点不能被单点 Execute）。—— round-trip + headless 验证
- [x] （采集）Viewpoint tab 出现 `Acquire All`；仅对 `ik_reachable&&collision_free` 的行执行；总确认框列出视点数与目标；链式逐点运动（`on_success` 链），每点落盘 RGB/depth/点云/camera_info/关节角/位姿到 `<session>/viewpoint_acquisition/<n>/` + `manifest.json`。—— 代码层完成 + 离线纯函数单测；真机运动待验收
- [x] （采集）中途点采集/写盘失败不中断整轮，事件记录 error；**独立 `_viewpoint_acquire_busy`** 门禁防并发（不误触发单点 Execute 的 UI 恢复）；keepout 保留；运动被 keepout 拦截的点跳过采集并记录 error 后继续。—— 代码层完成；真机运动待验收
- [x] （无回归）`--dry-run-check`/`--headless-check`/**全量构建**/单测全绿 ✅（`arachne_operator` 20 项含新增采集 13 项、`viewpoint_planner` 59 项含修复 `test_markers` Phase 2 遗留失败）；`grasp_task_server`/`road_cleanup_task_server`/`step_cleanup_demo --dry-run-check` 回归 ✅；git status 已检查 ✅。
- [x] （真机，**用户 2026-08 已确认通过**）碰撞过滤标记合理 ✅；一键采集落盘文件可离线浏览 ✅（用户修复运动 timeout/点云发布/碰撞异常后复测通过）；批次落盘多组区分 ✅；RViz 视点编号 + 常驻预览（启动即见、重启常驻、无需手动 Add）✅；Viewpoint 列表列宽铺满 ✅。

---

## 6. 可选后续（本阶段不做）

- keepout 盒/几何常量**统一为单一共享源**（当前 teach_panel 与 grasp_preview 重复声明，本阶段避免回归未迁移，列为统一重构）。
- 精确 Mesh 级碰撞（FCL/MoveIt PlanningScene）、遮挡感知、NBV 视点优化。
- 采集结果回灌 / 重规划循环（采集→感知→更新视点）。

---

## 7. 安全与流程约束

- 本阶段需求一（碰撞过滤）仍是**只规划不运动**，不触碰真机安全红线。
- 需求二（一键采集）**新增了连续多运动点**，但沿用现有 `move_arm_to_joints_degrees` 链路（keepout 挡板 + 速度流式），并额外加**总确认框**；不新增绕过 gate 的直接运动入口；不修改 `teach_panel.launch.py` 正常入口；不在 `scripts/` 加稳定 runtime wrapper。
- 不依赖 debug 工具；不新增第三方 Python 依赖（点云手写 `.pcd`）。
- 遵循 AGENTS.md：最小改动、`pathlib`/参数、硬件标定值走参数、注释仅非显然硬件/安全行为。
