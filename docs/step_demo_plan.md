# Step Demo Refactor Plan

本文规划 `step_cleanup_demo` 后续如何修改，使动作连贯、目标选择逻辑自洽，并复用 `grasp_task` / `grasp_preview_pipeline` 已经实机调过的检测、可达性和抓取判断。本文只规划，不表示当前已经完成实现。

## 当前问题


当前 Step Demo 的执行链路大致是：

```text
grasp_preview_pipeline publishes /arachne/perception/taco_instances
        ↓
step_cleanup_demo subscribes raw/perception events
        ↓
step_cleanup_demo chooses candidate by confidence + base_x
        ↓
step_cleanup_demo decides approach/grasp
        ↓
step_cleanup_demo calls /arachne/grasp_task/start
        ↓
grasp_task_server runs its own current detection/planning/grasp logic
```

这让 Step Demo 复用了 `grasp_task_server` 的执行服务，但没有复用其最新的目标选择和可抓判断。主要风险：

- Step Demo 只按 `confidence` 和 `base_x` 选目标，绕开了最新的 label、mask、depth、ROI、reach、IK、collision 和 waypoint 逻辑。
- Step Demo 决策的候选不一定是 `grasp_task_server` 实际会抓的候选。
- Step Demo 调 `/arachne/grasp_task/start` 时没有传递或锁定 candidate id / target token。
- Step Demo 默认 `grasp_min_base_x_m=0.30`、`grasp_max_base_x_m=0.90`，可能和实机验证过的 road/grasp 参数不一致。
- 深度相机点云坐标计算目前不是简单精度问题，而是可能存在相机外参/TF 根因：系统可能没有准确知道真实 depth optical frame 相对末端和 `base_link` 的位置。如果 `base_grasp_xyz` 的坐标系链路错了，Step Demo 的 approach / too_far 判断和最终抓取都会被连带放大。
- 当前真机演示主要使用 topdown 抓取和锁定姿态，这对部分瓶罐/盒状物是低风险默认策略，但不能长期假设所有物体都适合从正上方抓。后续需要根据物体形态、朝向、可见点云和夹爪开口方向选择抓取策略。
- `_observe_once()` 打开 `real_search_scan` 后，候选可能来自扫描过程，不应被当成简单静止单帧检测。
- `too_close` 直接失败，未复用 grasp task 的恢复/可达性判断。

## 改造目标

目标不是重新写一套 road cleanup，而是让 Step Demo 成为一个小而清晰的任务策略：

```text
Step Demo owns:
  - stop-look-step-look sequencing
  - base step / backtrack policy
  - cancellation, timeout, status, logs

Grasp Task / Grasp Preview owns:
  - detection filtering
  - best target selection
  - depth / mask / ROI quality
  - pointcloud coordinate quality
  - grasp strategy selection
  - reach / IK / collision / waypoint validity
  - final target lock used by grasp execution
```

改造后的核心链路应为：

```text
step_cleanup_demo
        ↓ request observation / restart search
grasp_task_server + grasp_preview_pipeline
        ↓ publish evaluated target status
step_cleanup_demo
        ↓ approach / backtrack / grasp decision using evaluated status
grasp_task_server
        ↓ start grasp against the same locked target
```

## 关键原则

- 不替换实机验证过的抓取坐标和阈值；Step Demo 应引用或消费这些结果，而不是复制一套新阈值。
- 不让 Step Demo 直接使用 raw detection 作为最终决策源。
- 不让 Step Demo 用临时系数补偿深度点云坐标误差；必须先确认真实相机位姿和 TF 链路，坐标修正应收敛在 `grasp_preview_pipeline` / `depth_to_pointcloud` / TF 标定链路中。
- 不把 dry-run / mock 成功当成真实硬件验证。
- 不绕过 Aubo `/arachne/aubo/move_joint`、control owner、teach gate、stop/timeout 等安全边界。
- 不改变 `grasp_task_server` 的真实执行确认、安全变量和 fallback 默认策略。
- 不把 topdown 抓取写死为所有物体的唯一策略；它可以继续作为当前安全默认，但策略选择应由 grasp task / preview 评估产生。

## 建议的新目标状态合同

新增一个由 `grasp_task_server` 或 `grasp_preview_pipeline` 负责发布的目标状态。优先做 topic，保持轻量：

```text
/arachne/grasp_task/target_status
type: std_msgs/String(JSON)
```

建议 JSON 字段：

```json
{
  "stamp": "2026-06-28T12:00:00.000",
  "target_id": "candidate-xxxx",
  "state": "ready",
  "label": "bottle",
  "confidence": 0.82,
  "base_grasp_xyz": [0.72, 0.08, -0.10],
  "reach_radius_m": 0.73,
  "mask_area_px": 2600,
  "depth_valid": true,
  "coordinate_quality": "usable",
  "coordinate_error_hint_m": 0.02,
  "grasp_strategy": "topdown",
  "grasp_strategy_confidence": 0.74,
  "grasp_strategy_reasons": ["upright_or_flat_visible_surface", "topdown_ik_clear"],
  "object_shape_hint": "bottle_or_can",
  "object_major_axis_base": [0.98, 0.18, 0.0],
  "planning_ready": true,
  "reachable": true,
  "rejection_reasons": [],
  "suggested_base_step_m": 0.0,
  "suggested_search_step_m": 0.10,
  "lock_token": "target-lock-xxxx",
  "expires_sec": 2.0
}
```

`state` 建议枚举：

- `no_target`
- `stale`
- `depth_invalid`
- `coordinate_suspect`
- `grasp_strategy_unavailable`
- `too_far`
- `too_close`
- `lateral_out_of_bounds`
- `unreachable`
- `planning_failed`
- `ready`

Step Demo 只消费这个 evaluated status。`/arachne/perception/taco_instances` 可继续存在，但只作为 debug/preview event，不作为 Step Demo 的最终决策依据。

## Grasp Start 目标锁定

需要避免“Step Demo 判断 A 可抓，grasp task 实际抓 B”。建议增加最小目标锁定机制：

```text
/arachne/grasp_task/lock_target
/arachne/grasp_task/start_locked
```

如果不想新增 service，短期也可以让 `/arachne/grasp_task/start` 使用最近一次 `target_status.lock_token`，并要求：

- target 未过期。
- target still reachable。
- start 时执行的 candidate id 和 `target_status.target_id` 一致。
- 如果不一致，返回失败并说明 `target changed`。

推荐长期语义：

```text
Step Demo:
  waits target_status.state == ready
  stores lock_token
  calls grasp_task start with lock_token

Grasp Task:
  validates token
  executes exactly that planned/locked target
```

## 修改后的 Step Demo 状态机

建议状态：

```text
idle
preflight
prepare_search_pose
observe
evaluate_target
approach
search_step
backtrack
grasp
return_home
succeeded / failed / canceled
```

执行细节：

1. `preflight`
   - 检查 grasp task preflight。
   - 检查 base command/status service。
   - 检查 `/arachne/grasp_task/target_status` 是否可见。
   - 不直接检查 raw detection topic 作为通过条件。

2. `prepare_search_pose`
   - 可选移动 Aubo 到搜索姿态。
   - 使用 `/arachne/aubo/move_joint`。
   - 保留当前默认搜索关节，不随便改实机标定值。

3. `observe`
   - 发布 `restart_search`。
   - 根据需要打开 `real_search_scan`。
   - 等待 fresh `target_status`。
   - 如果 `state=stale`，丢弃旧目标并重新观察。
   - 如果 `state=no_target`，不要立刻失败；进入有边界的 `search_step`。

4. `evaluate_target`
   - 如果 `state=ready`，进入 grasp。
   - 如果 `state=too_far`，根据 `suggested_base_step_m` 或保守 step policy 做目标引导靠近。
   - 如果 `state=no_target`，根据 `suggested_search_step_m` 或固定 `search_step_m` 做盲搜前进。
   - 如果 `state=too_close`，允许一次小幅 backtrack，而不是直接失败。
   - 如果 `state=coordinate_suspect/depth_invalid`，优先重新 observe 或请求相机/点云诊断，不用不可信坐标发 grasp。
   - 如果 `state=grasp_strategy_unavailable`，不要用 Step Demo 的默认姿态硬抓；记录目标形态/原因并重新观察或安全失败。
   - 如果 `state=lateral_out_of_bounds/unreachable/planning_failed`，不要用 `base_x` 强行判断；记录原因并失败或请求 grasp task 的 recovery suggestion。

5. `approach`
   - 只用于已经看见目标但目标太远的 target-guided approach。
   - 通过 `/arachne/grasp_task/base_command` 发 `drive_relative`。
   - 等待 `/arachne/grasp_task/base_status` 确认同一个 request id 完成。
   - 每步后必须重新 observe，不复用旧 target。

6. `search_step`
   - 只用于没有看到目标时的 bounded blind search。
   - 每次只前进保守距离，例如 `search_step_m=0.10`。
   - 计入独立的 `search_steps`，不得无限前进。
   - 每步后必须 stop、重新 observe，并清空旧 target。
   - 如果连续 `max_search_steps` 后仍是 `no_target`，才进入 failed：`no target after N search steps`。

7. `grasp`
   - 调用锁定目标的 grasp start。
   - 等待 `/arachne/grasp_task/state` terminal result。
   - 成功后计数。

8. `return_home`
   - 可继续使用当前累计 progress 的反向 drive。
   - 只作为简化返航，不当成导航定位恢复。

## Approach 与 Search Step 的区别

Step Demo 应明确区分两种底盘前进：

```text
target-guided approach:
  已经看到目标，target_status=too_far
  使用 suggested_base_step_m 或根据目标距离算出小步
  目标是把同一个目标带入可抓范围

blind search step:
  没看到目标，target_status=no_target
  使用固定保守 search_step_m
  目标是扩大前方观察区域，不绑定任何旧目标
```

两者都必须遵守：

- 每步之前先停稳。
- 每步之后重新观察。
- 不复用旧 target。
- 有最大步数上限。
- stop/cancel 时立即停止 base 和 search scan。

## 目标状态的推荐计算位置

优先方案：在 `grasp_task_server` 内聚合。

原因：

- `grasp_task_server` 已经管理 grasp preflight、preview runner、real execution、base command、task state。
- Step Demo 本来就是调用 grasp task 的 higher-level policy。
- `grasp_task_server` 可以把 preview/pipeline 输出转换成 task-level target status。

备选方案：由 `grasp_preview_pipeline` 直接发布更完整的 evaluated target。

缺点：

- Step Demo 仍会绕过 grasp task 的最终任务状态。
- target lock / start consistency 更难保证。

推荐折中：

```text
grasp_preview_pipeline publishes rich preview event
        ↓
grasp_task_server consumes it and republishes /arachne/grasp_task/target_status
        ↓
step_cleanup_demo consumes only target_status
```

## 相机外参 / TF 根因排查

Step Demo 的动作是否自洽，依赖 `target_status.base_grasp_xyz` 是否可信。当前更可能的问题不是 Step Demo 如何解释坐标，而是系统是否正确知道深度相机在哪里：`camera_depth_optical_frame` 到 `base_link` 的 TF 链路一旦错，所有点云投影后的坐标都会系统性偏移。

当前源码中需要重点核对的链路是：

```text
base_link
  -> aubo_base_link
  -> Aubo joints / aubo_wrist3_Link
  -> ee_camera_support_link
  -> ee_camera_link
  -> camera_depth_optical_frame
```

风险点：

- `src/arachne_sensors/launch/gemini335.launch.py` 当前用 `camera_parent_frame=ee_camera_link` 和 hardcoded `camera_optical_*` 发布 `ee_camera_link -> camera_depth_optical_frame`。
- `src/arachne_description/urdf/arachne.urdf.xacro` / `ee_camera.xacro` 也定义了 `ee_camera_support_link -> ee_camera_link` 的固定安装位姿。
- `src/arachne_description/config/physical_parameters.yaml` 中存在历史 hand-eye 标定 `tool0 -> camera_color_optical_frame`，但状态是 `archived_hand_eye_not_used_for_rviz_camera_tf`，当前并未作为运行时 TF source of truth。
- 如果真实相机安装位置、optical frame 方向、URDF 支架位姿、hand-eye 标定和 launch 静态 TF 之间任一处不一致，点云坐标就会“看起来能用但抓不准”。
- 2026-06-28 离线验证发现：启动现有 `arachne_description display.launch.py` 后，`/arachne/display/joint_states` 能接收变化的 Aubo 关节角，`ee_camera_link -> camera_depth_optical_frame` static TF 也存在，但 `tf2_echo base_link camera_depth_optical_frame` 报告两者不在同一棵 TF 树中。这说明当前运行路径下实时模型 TF 到相机 optical frame 的连接尚未验证通过。

重点不要在 Step Demo 内部做补偿，而应检查和收敛这些上游环节：

- 先验证 `camera_depth_optical_frame` 在 RViz/TF 中的位置和方向，而不是先调 Step Demo 阈值。
- 先确认 `robot_state_publisher` 实际发布 `base_link -> aubo_* -> aubo_wrist3_Link -> ee_camera_support_link -> ee_camera_link`，并且它和 `ee_camera_link -> camera_depth_optical_frame` static TF 属于同一棵 TF 树。
- 明确运行时 TF 的 source of truth：要么使用经过验证的 URDF + static TF，要么接入新的 hand-eye 标定结果；不要同时存在互相矛盾的 camera TF。
- `src/arachne_sensors/arachne_sensors/depth_to_pointcloud_node.py` 的 `CameraInfo`、`depth_scale`、`projection_flip_x/y`、stride 后像素网格、`target_frame` TF 转换必须和真实相机一致。
- `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` 的 `_pixel_to_xyz()`、ROI depth 采样、`_roi_points()`、`_base_from_depth_transform()`、`_make_base_path()` 必须使用同一套投影和 TF 约定。
- `camera_optical_*`、`ee_camera_xyz/rpy`、`grasp_base_offset_xyz`、`depth_projection_flip_*` 等实机调过的参数必须保留为 source of truth，不应被 Step Demo 默认值覆盖。
- `target_status` 应暴露坐标质量信号，例如 `coordinate_quality`、`coordinate_error_hint_m`、`depth_valid`、`roi_points`、`pointcloud_grasp_shape.point_count`、`depth_projection`。
- 如果坐标质量不足，grasp task 应发布 `state=coordinate_suspect` 或 `depth_invalid`；Step Demo 只能重新 observe / search step / fail safe，不应直接发抓取。

建议的验证方式：

- 先运行只读 TF 检查，确认 `ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame` 的平移和姿态与真实安装大体一致。
- 移动或注入两组不同的 Aubo joint state 后，重复 `tf2_echo base_link camera_depth_optical_frame`；如果平移/姿态不变，说明相机 TF 没有跟随机械臂运动。
- 如果 `tf2_echo` 报告 `base_link` 和 `camera_depth_optical_frame` 不在同一棵树，必须先修 TF 发布链路，再讨论点云精度。
- 在 RViz 中显示机器人模型、`camera_depth_optical_frame`、`/arachne/debug/depth_points`，确认点云地面落在真实地面附近，而不是整体旋转/漂移到错误位置。
- 用固定高度地面、已知尺寸物体或标定板采样，比较 `grasp_camera_xyz` 和 `base_grasp_xyz` 的 x/y/z 偏差。
- 对同一静止目标重复采样，记录 `base_grasp_xyz` 抖动量，区分系统偏差和随机噪声。
- 检查彩色 ROI、深度 ROI、mask ROI 是否对齐，特别是反光地面和目标边缘处的深度 percentile 是否稳定。
- 在 RViz 同时查看 `/arachne/debug/depth_points`、目标 marker、规划 waypoint，确认点云落地位置和机械臂可达判断一致。
- 相机 TF 没有收敛前，不把 Step Demo 的 ready/too_far 判断当成最终真机抓取验证。

## 抓取策略不应长期固定为 Topdown

当前真机链路倾向于 topdown 抓取，部分 launch 参数还会锁定抓取姿态，例如 `--lock-grasp-orientation`、`--grasp-topdown-max-tilt-deg`、`--topdown-flange-rpy-rad`。这适合作为当前阶段的保守默认，但它不是通用抓取策略。

后续应把“怎么抓”也纳入 grasp task / preview 的 evaluated target，而不是让 Step Demo 固定调用一种姿态。

建议先支持这些策略枚举：

- `topdown`: 目标上表面可见，点云高度稳定，IK 和碰撞检查通过。
- `side_pinch`: 目标竖直或侧面轮廓更可靠，夹爪应沿点云主轴/副轴选择夹持方向。
- `angled_topdown`: topdown 可行但需要小角度倾斜，适合边缘遮挡或瓶罐倾斜。
- `defer`: 形态/点云/可达性不足，不执行抓取，只重新观察或安全失败。

策略选择应优先由这些信号决定：

- label / class hint，例如 bottle、can、box、cup、bag。
- mask 轮廓长宽比和朝向。
- ROI 点云 PCA 主轴、副轴、高度范围和 axis confidence。
- 目标是否贴地、是否倾斜、是否只有上半部分可见。
- 当前夹爪类型、开口方向、gripper clearance。
- IK、碰撞、地面 clearance、basket/rear-rack keepout。

推荐输出方式：

```text
grasp_preview_pipeline estimates shape / orientation candidates
        ↓
grasp_task_server chooses grasp_strategy and validates IK/collision
        ↓
target_status publishes grasp_strategy + reasons + lock_token
        ↓
step_cleanup_demo only starts locked target when strategy is accepted
```

Step Demo 的责任是尊重 `target_status.grasp_strategy` 和 `state`：

- `ready + topdown/side_pinch/angled_topdown`：可以进入 locked grasp。
- `grasp_strategy_unavailable/defer`：不硬抓，重新 observe 或失败。
- debug 日志记录策略、原因、候选姿态数和最终锁定姿态。

实施时不要一次性改掉当前 topdown 真机默认。推荐顺序是：

1. 先在 `target_status` 中记录当前实际使用的 topdown 策略和原因。
2. 再让 preview 输出候选姿态和 shape hints，不改变执行。
3. 最后才允许 grasp task 在明确安全门槛下选择非 topdown 策略。

## 参数策略

Step Demo 应尽量减少自己持有的几何阈值。

保留在 Step Demo 的参数：

- `approach_step_m`
- `max_approach_steps`
- `search_step_m`
- `max_search_steps`
- `max_grasps`
- `observe_timeout_sec`
- `base_step_timeout_sec`
- `return_home_on_finish`
- `move_to_search_pose_before_start`

迁出或只作为 fallback 的参数：

- `grasp_min_base_x_m`
- `grasp_max_base_x_m`
- `target_base_x_m`
- `confidence`

这些应由 grasp task target evaluator 给出结果，Step Demo 不再自己解释原始 detection。

## 分阶段实施

### Phase S1: 文档与观测对齐

- 保持现有行为不动。
- 给 `step_cleanup_demo` 增加日志字段，记录当前用的是 raw candidate 还是 evaluated target。
- 明确当前 raw candidate 路径为 legacy/temporary。

### Phase S2: 发布 target_status

- 在 `grasp_task_server` 增加 `/arachne/grasp_task/target_status`。
- 从 preview event 中整理出 best target、rejection reasons、suggested step。
- target_status 先记录当前默认 topdown 策略、候选姿态数量和选择原因。
- 不改变 grasp execution。

### Phase S2.5: 先收敛相机外参 / TF

- 确认运行时 `base_link -> camera_depth_optical_frame` 真实可信。
- 确认机械臂关节变化时，`base_link -> camera_depth_optical_frame` 会随 URDF/FK 实时变化。
- 修复或确认 `robot_state_publisher`、`/joint_states`、`/tf`、`/tf_static`、`ee_camera_link -> camera_depth_optical_frame` 之间处于同一 TF 树。
- 对齐 URDF、`gemini335.launch.py` 静态 TF、历史/新 hand-eye 标定和真实安装位姿。
- 对齐 `depth_to_pointcloud_node` 和 `grasp_preview_pipeline` 的投影、flip、scale、TF 语义。
- 保留远端实机实验验证过的 grasp 坐标、offset 和阈值，不用本地默认参数替代。
- 在 target_status 中加入坐标质量字段和 rejection reason。
- 只有坐标质量达到 usable 时，Step Demo 才能把 `ready/too_far` 用于动作决策。

### Phase S3: Step Demo 消费 target_status

- Step Demo 不再订阅 `/arachne/perception/taco_instances` 做决策。
- Step Demo 等待 `target_status`，按 state 决策 approach/backtrack/grasp。
- raw detection 只保留为 debug fallback，默认关闭。
- `target_status.state=no_target` 时执行 bounded search step，而不是直接失败。
- `target_status.state=too_far` 时执行 target-guided approach。
- `target_status.state=grasp_strategy_unavailable` 时不使用 Step Demo 默认姿态硬抓。

### Phase S4: 目标锁定

- 增加 lock token 或 start_locked 服务。
- Step Demo 调用 grasp 时传递 token。
- Grasp Task 保证执行同一个 target，否则失败并返回 reason。

### Phase S4.5: 多抓取策略

- 在相机 TF、点云坐标和 target lock 稳定后，再引入非 topdown 策略。
- 先只做离线/preview 策略评估，不改变真机执行。
- 再在低速低风险验证中启用 `side_pinch` / `angled_topdown`。
- topdown 继续保留为默认安全策略和 fallback，但不再被当作所有目标的唯一策略。

### Phase S5: 真实硬件验证

- 先 dry-run / headless。
- 再只读硬件。
- 再 current-state hold。
- 最后才做低速小步 base + locked target grasp。

## 验收标准

离线：

```bash
ros2 run arachne_operator step_cleanup_demo --dry-run-check
ros2 run arachne_operator grasp_task_server --dry-run-check
ros2 run arachne_operator arachne check entrypoints
```

ROS graph smoke：

- `step_cleanup_demo` 启动后服务存在。
- `/arachne/grasp_task/target_status` 存在。
- Step Demo status 中能看到最近 target state。
- 不调用真实 start 时不发 base command。
- target_status 能报告 depth / coordinate quality，不把 `coordinate_suspect` 当成 `ready`。
- target_status 能报告当前 grasp_strategy；`grasp_strategy_unavailable` 不会触发 grasp start。

真实硬件前置：

- Aubo readonly check 通过。
- `/joint_states` 正常。
- `/arachne/aubo/move_joint` dry-run graph 正常。
- `base_link -> camera_depth_optical_frame` TF 与真实安装方向一致，且没有互相矛盾的重复 camera TF source。
- grasp task target_status 能稳定报告 ready/too_far/no_target。
- 静止目标的 `base_grasp_xyz` 重复采样误差在可接受范围内，且 RViz 点云/marker/waypoint 方向一致。
- `no_target` 能触发 bounded search step，并在最大步数后明确失败。
- `too_far` 能触发 target-guided approach，且每步后重新 observe。

真实低风险：

- 只允许小步 base approach / search step。
- 每步后必须重新 observe。
- 不允许 Step Demo 自己覆盖 grasp task 的目标选择。

## 不做的事

- 不把 Step Demo 做成第二个 road cleanup state machine。
- 不复制 grasp pipeline 的 reach/IK/collision 逻辑。
- 不改变实机验证过的抓取坐标阈值。
- 不在 Step Demo 内硬编码临时深度坐标补偿。
- 不在 Step Demo 内硬编码 topdown 为所有物体的唯一抓取方式。
- 不把 `/arachne/perception/taco_instances` 当成最终任务合同。
- 不默认开启任何新的真实运动路径。

## 期望结果

修改后，Step Demo 应该变成：

```text
look
  ask grasp task what it can actually grasp
step
  if target is too far, move a target-guided small step
  if no target is visible, move a bounded search step
look again
  never reuse stale target
grasp
  execute the same locked target and grasp strategy that were evaluated
return
  backtrack only the measured step-demo progress
```

这样它的动作会更连贯，目标选择和真实抓取执行会来自同一个 task-level 判断，避免“看的是 A、抓的是 B”或“Step Demo 觉得可抓、grasp task 觉得不可抓”的分裂。
