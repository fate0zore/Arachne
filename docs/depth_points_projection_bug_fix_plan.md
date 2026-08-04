# `/arachne/debug/depth_points` 投影偏移修复路线

> 来源:2026-07-22 只读排查结论(未改代码)。本文件只整理修复路线,给出具体的指令。
> 症状回顾:只有在 home 位置才能正确投影到地面,机械臂移动后映射偏移,静止后仍偏;垂直于地面的物体在 RViz 中投影歪斜。

## 一、根因总结

### 根因 A(主):点云首帧冻结,不再随机械臂运动更新

- 实测 `arachne_depth_to_pointcloud` 节点实参:`continuous:=False`、`exit_after_publish:=False`、`republish_period_sec:=1.0`。
- `src/arachne_sensors/arachne_sensors/depth_to_pointcloud_node.py`:
  - L107-110:`if self.published and not continuous: return` —— 首帧成功发布后 `published=True`,之后所有新 depth 帧被直接丢弃。
  - L183-199:首帧发布成功后启动 1.0s 定时器,反复重发同一个 `snapshot_points`。
- 结果:点云坐标在第一次采集后被冻结,不再随 `base_link←camera_depth_optical_frame` 的实时 TF 更新。
  - 机械臂静止后 TF 已是新位姿,但重发的点云坐标仍是首帧时刻算出的固定值 → "静止后仍偏"。
  - 若首帧采集于 home,移动后自然只对 home → "只有 home 对、动后偏"。

### 根因 B(次):相机光学外参硬编码默认,未经真机标定验证

- 实测 `/tf_static` 中 `ee_camera_link → camera_depth_optical_frame`:
  - `t = (0.0201, 0, 0.2196)`
  - `RPY = (11.23°, -1.375°, -90°)`
- 该 `11.23° roll` / `-1.375° pitch` 来自 `src/arachne_sensors/launch/gemini335.launch.py` L189-191 硬编码默认 `camera_optical_roll:=0.196` / `camera_optical_pitch:=-0.024` / `camera_optical_yaw:=-1.570796327`。
- `src/arachne_description/config/physical_parameters.yaml` 存在真实手眼标定:
  - `hand_eye_tool0_to_camera_color_optical`(19 样本,重投影 1.47 px,DANIILIDIS)
  - `status: archived_hand_eye_not_used_for_rviz_camera_tf` —— 标定做了未接入运行时 TF。
- `docs/step_demo_plan.md` L301-309/419 已自警:"真实相机安装、optical frame、URDF、手眼标定、launch 静态 TF 间任一处不一致就会'看起来能用但抓不准'。"
- 结果:约 11° 的 roll 倾斜让整片点云相对真实地面/物体倾斜;home 时相机近乎垂直俯视地面不明显,移动后视点变化被放大 → "垂直物体投影歪"。

### Packaging 疑点:实际运行的节点并非 teach_panel 命令串所配

- 当前 `arachne_depth_to_pointcloud` 实参与 `teach_panel.launch.py` 的 `depth_pointcloud_command` 命令串不一致:

  | 参数 | teach_panel 命令串 | 节点实参 |
  |------|------|------|
  | continuous | `true` (L625) | **False** |
  | stride | `4` (L623) | **1** |
  | projection_flip_x/y | `false` (L624) | **True** |
  | max_depth_m | `3.0` | **5.0** |

- `ps` 显示节点以 `--params-file /tmp/launch_params_4sad5yrr`(临时文件已删除)启动,而非 `--ros-args -p` 命令串。
- 推断:当前 RViz 看到的节点可能是另一套 launch(如 standalone `depth_to_pointcloud.launch.py` 默认值)启动的,而非 teach_panel 拉起 —— 这直接导致根因 A。

### 已排除项

- **projection_flip_x/y 不是根因**:`flip=false` 是 `depth_to_pointcloud_node`、`grasp_preview_pipeline`、standalone `depth_to_pointcloud.launch.py` 的统一有意约定;相机驱动的 `flip=true` 只作用于它"被 `publish_pointcloud:=false` 关闭的内部点云",与 `/arachne/debug/depth_points` 无关。flip 是镜像选项,不解释"歪"。
- **stride 像素网格不是根因**:stride>1 时源码会 `*stride` 还原全分辨率像素再与全分辨率 K 对齐(`depth_to_pointcloud_node.py` L117-129/L143);实测 `stride=1` 不触发降采样。
- **TF 时间戳错位非主因**:`tf2_echo` 用 time=0 能成功查到 TF,stamp 查询 + time=0 回退路径可行。
- **"两棵 TF 树"假设已被否定**:全链 `base_link→aubo_*→aubo_wrist3_Link→ee_camera_support_link→ee_camera_link→camera_depth_optical_frame` 单树连通;TF 随关节变化(注入测试:真实关节 `[0.883,-0.122,0.433]` → home 注入后 `[0.820,-0.087,0.495]`)。
- **`step_demo_plan.md:302` "不在同一棵 TF 树"结论已过时**:当前运行栈不复现。

---

## 二、修复路线(按优先级)

### 路线 1:消除首帧冻结 —— 修正 `continuous` 生效路径

> 目标:让点云随每帧 depth + 实时 TF 持续重算,而非首帧冻结重发。

**1.1 先定位实际启动来源(不改代码,只读验证)**

```bash
# 确认当前节点是谁起的、用什么参数
ps -ef | grep arachne_depth_to_pointcloud | grep -v grep
ros2 param get /arachne_depth_to_pointcloud continuous
ros2 param get /arachne_depth_to_pointcloud stride
ros2 param get /arachne_depth_to_pointcloud projection_flip_x
ros2 param get /arachne_depth_to_pointcloud max_depth_m
```

若实参与 teach_panel 命令串不一致,说明节点不是 teach_panel 拉起的。逐个 launch 文件排查谁在默认以 `continuous=false` 启动该节点:

```bash
grep -rn "continuous" src/arachne_sensors/launch/ src/arachne_operator/launch/
```

**1.2 修正参数透传(需改代码)**

- 检查 `src/arachne_sensors/launch/depth_to_pointcloud.launch.py` 的 `depth_to_pointcloud_command` 是否真的把 `continuous:=true` / `stride:=4` / `projection_flip_x/y:=false` 通过 `--ros-args -p` 正确透传给节点(当前疑似用临时 params-file 启动导致默认值不生效)。
- 确认 `src/arachne_sensors/arachne_sensors/depth_to_pointcloud_node.py` 的 `continuous` 参数默认值是否应从 `False` 改为 `True`(若系统默认就该连续投影,应改默认)。
- 确保 teach_panel 与 standalone 两条启动路径行为一致。

**1.3 验证**

```bash
# 修改后重启,确认 continuous 实际生效
ros2 param get /arachne_depth_to_pointcloud continuous   # 应为 True
# 移动机械臂,观察 RViz 中点云是否跟随地面 / 不再停在 home
ros2 topic hz /arachne/debug/depth_points                # 应接近 depth 帧率
```

### 路线 2:统一相机光学外参 source of truth

> 目标:消除 11.23° 未验证倾斜,让 `ee_camera_link→camera_depth_optical_frame` 对应真实相机安装。

**2.1 二选一(优先用标定结果)**

- **方案 A(推荐):接入已有手眼标定**
  - `physical_parameters.yaml` 的 `hand_eye_tool0_to_camera_color_optical`(`rpy=[0,0,3.128380]`、`xyz=[-0.239470,0.181459,0.190102]`)是 `tool0→camera_color_optical_frame`,需要换算成 `ee_camera_link→camera_depth_optical_frame` 的等效变换(经 `aubo_wrist3_Link→ee_camera_support_link→ee_camera_link` 链路组合)。
  - 换算后用真实标定值替换 `gemini335.launch.py` L186-191 的 `camera_optical_xyz/rpy` 硬编码默认。
- **方案 B:重新做真机手眼标定**
  - 用现有 hand-eye 流程采集新数据,直接输出 `ee_camera_link→camera_depth_optical_frame` 的 xyz/rpy,覆盖硬编码默认。
  - 标定后将结果回写 `physical_parameters.yaml`,并将该条 status 从 `archived` 改为 `current_rviz_camera_tf`。

**2.2 显式 source of truth 约定**

- 在 `gemini335.launch.py` 注释明确:`ee_camera_link→camera_depth_optical_frame` 的唯一 source of truth 是 `physical_parameters.yaml` 的手眼标定,而非硬编码默认。
- 移除/废弃硬编码 `camera_optical_roll:=0.196` 等占位默认,改为从标定结果读取。

**2.3 验证**

```bash
# 修改后重启
ros2 topic echo /tf_static  # 确认 ee_camera_link->camera_depth_optical_frame 的 RPY 已变
# 再用 rqt_image_view 看 depth,放一块垂直于地面的标定板,
# 在 RViz 里看点云是否与标定板真实方向一致(不再歪)
# home 与至少两个非 home 关节位姿下分别验证地面投影与垂直物体姿态
```

### 路线 3:修复 packaging 疑点(保障参数真实性)

> 目标:确保 teach_panel 声明的 `depth_pointcloud_command` 参数在节点上真实生效,避免"命令串写了 true 但节点跑 false"。

- 排查 `depth_pointcloud_command` 的 `parameters=[...]` / `--ros-args -p` 写法是否在 launch 系统里被正确解析(ROS2 launch 中 `ExecuteProcess` 的 `parameters` 与 `Node` 的 `parameters` 语义不同,`ExecuteProcess` 不自动转 params-file)。
- 若当前实现确实使用临时 params-file 启动,确认其内容是从 LaunchConfiguration 动态生成的,且与期望命令串一致。
- 在 `arachne check entrypoints` 或 dry-run 检查中加一条:启动后立即校验 `continuous`/`stride`/`flip`/`max_depth_m` 是否等于声明值。

---

## 三、验证顺序建议

1. 先做**路线 1**(改 continuous 生效路径),重测移动后是否仍偏 —— 预计能消除"静止后仍偏",验证首帧冻结是否为主因。
2. 再做**路线 2**(外参标定),用垂直标定板验证"歪"是否消除。
3. 最后做**路线 3**(packaging),确保长期不会因参数未生效而复发。

## 四、安全约束(沿用 AGENTS.md)

- 所有修改经 `./scripts/build/build_workspace.sh` 重建。
- dry-run/headless 路径不触碰真实硬件;真机手眼标定与运动验证需用户显式确认机械臂就绪。
- 不新增 shell wrapper 作为稳定运行入口;不改动 `/joint_states` 链与 Aubo 控制权网关。
- `teach_panel.launch.py` 仍为正常操作入口。