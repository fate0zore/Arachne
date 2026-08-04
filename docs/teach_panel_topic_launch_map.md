# Teach Panel Topic 启动链路梳理

本文基于 [`scripts/operator/start_teach_panel.sh`](../scripts/operator/start_teach_panel.sh) 追踪 [`teach_panel.launch.py`](../src/arachne_operator/launch/teach_panel.launch.py) 启动的所有 ROS topic、服务与 action 的来源。

> 入口脚本 `start_teach_panel.sh` 的核心是最后一行 `ros2 launch arachne_operator teach_panel.launch.py "$@"`。它在启停前后只做环境 source、旧栈清理与日志重定向,真正的 topic 启动全在顶层 launch 及其 include 的子 launch + demo_orchestrator 的 subprocess 托管机制里。

## 启动总链路

顶层 `teach_panel.launch.py` 通过 5 个并行组件启动全部 topic 和服务:

| 组件 | 条件参数 | 默认 | 作用 |
|---|---|---|---|
| `teach_visualization.launch.py` | `with_visualization` | **true** | 可视化 / 机器人模型 / joint_states 桥接 |
| `gemini335.launch.py` | `with_camera` | **false** | 顶层直接相机(默认不开) |
| `real_bringup.launch.py` | `with_real_bringup` | **true** | 真实底盘 / 夹爪 / Aubo 驱动 |
| `demo_orchestrator.launch.py` | `with_demo_orchestrator` | **true** | demo 编排 + 托管子进程 |
| `teach_panel` 节点 | 无条件 | — | 示教面板主控 |

> 注意:顶层 launch 里的 `gemini335.launch.py` 默认是关的(`with_camera:=false`);相机一般由下面的 demo_orchestrator 托管进程启动。

---

## 一、teach_visualization.launch.py —— 可视化与 joint_states 桥接

文件:`src/arachne_operator/launch/teach_visualization.launch.py`。启动 3 个节点(L38-82):

1. **`teach_visualization_joint_states`**(`arachne_operator`,L39-60)
   - 订阅 `/joint_states`(默认 `input_joint_states_topic`,L103)
   - 发布 `/arachne/teach_visualization/joint_states`(默认 `visualization_joint_states_topic`,L105-107)
   - 当真实 `/joint_states` 无发布者时,可用 Aubo SDK fallback(`aubo_sdk_fallback_enabled` 默认 true,L131)

2. **`arachne_teach_robot_state_publisher`**(`robot_state_publisher`,L61-74)
   - 订阅 `/arachne/teach_visualization/joint_states`(remap `joint_states`,L67)
   - 发布 `/arachne/display/robot_description`(remap,L69-71)与 TF

3. **`rviz2`**(`arachne_teach_rviz`,L75-82,`with_rviz` 默认 true)

可选:`visualization_with_lslidar_driver` 默认 false,开启时会额外 include `lslidar_cx_launch.py`(L84-96),发布 `/scan` 等激光数据。

---

## 二、real_bringup.launch.py —— 真实硬件驱动(默认全开)

文件:`src/arachne_hardware/launch/real_bringup.launch.py`。用 `OpaqueFunction` 按开关动态组装节点。默认 `use_scout=true`、`use_ms42dc=true`、`use_aubo=true`、`use_aubo_ros2_driver=false`(L358-361)。

### 底盘(scout,默认 `scout_driver=waveshare`)

- **`scout_waveshare_serial_driver`**(`arachne_hardware`,L60-83)
  - 订阅 **`/cmd_vel`**(L75)
  - 发布 **`/odom`**(L76)、odom→base_link TF(L77-78)
- 若选 `official` 则改为 `scout_base.launch.py` + `scout_official_status_bridge`(L36-58),同样把 `odom_topic_name` 设为 `/odom`。

### 夹爪(ms42dc,默认 `ms42dc_driver=direct`)

- **`ms42dc_direct_serial_driver`**(`arachne_hardware`,L142-174)
  - 订阅 **`/arachne/gripper/command`**(teach_panel 传入的 `gripper_command_topic`)
  - 发布 `/arachne/gripper/status`
- 或 vendor 模式:`step_motor/motor_node` + `ms42dc_official_bridge`(L93-139)

### 机械臂(aubo)

- **`aubo_official_status_probe`**(`arachne_hardware`,L202-217)
  - 发布 **`/joint_states`**(Aubo 关节状态,供 `teach_visualization_joint_states` 订阅)
- **`aubo_teach_command_bridge`**(`arachne_hardware`,L220-242)
  - 订阅 **`/arachne/aubo/teach_command`**(L226)→ 切换 Aubo 示教/拖动模式
- **`aubo_sdk_velocity_bridge`**(`arachne_hardware`,L245-275)
  - 订阅 **`/arachne/aubo/joint_velocity_command`**(L251,来自顶层 `arm_velocity_command_topic` 默认值)
  - 通过 SDK 实时下发关节速度,带 watchdog
- **`aubo_move_joint_action_server`**(`arachne_hardware`,L303-343)
  - 提供 action **`/arachne/aubo/move_joint`**(L309,`aubo_move_joint_action_name` 默认值)

---

## 三、demo_orchestrator.launch.py —— 编排节点 + 托管子进程

文件:`src/arachne_operator/launch/demo_orchestrator.launch.py`。L118-168 只启动一个节点 **`demo_orchestrator`**,但它通过 `subprocess.Popen` 拉起多个 managed processes。

### demo_orchestrator 节点本身

- 订阅 `joint_states_topic`、`odom_topic`、`gripper_status_topic`、`aubo_teach_command_topic`、`camera_color_topic`、`camera_depth_topic`(L125-135)
- 提供服务:`/arachne/demo/start_visual_grasp`、`/arachne/demo/start_road_cleanup`、`/arachne/demo/pause_road_cleanup`、`/arachne/demo/return_home`、`/arachne/demo/stop`、`/arachne/demo/preflight`、`/arachne/demo/status`
- 发布 **`/arachne/demo/state`**

### 托管进程机制

源码 `src/arachne_operator/arachne_operator/demo_orchestrator.py:28-32` 定义映射名→命令参数:

```
camera          -> camera_command
depth_pointcloud -> depth_pointcloud_command
viewer          -> camera_view_command
grasp_server    -> grasp_server_command
cleanup_server  -> cleanup_server_command
```

`_start_process`(L384-447)用 `subprocess.Popen(["bash", "-lc", shell_command])`(L421-422)执行命令字符串,日志写入 `runtime_log_root`。顶层 launch 的 `autostart_managed_processes` 默认值 `camera,depth_pointcloud,viewer,grasp_server,cleanup_server`(`teach_panel.launch.py`:L583-585)决定哪些进程在启动时自动拉起。

每个命令字符串实际启动的内容(来自 `teach_panel.launch.py:588-692` 的权威默认值):

| managed 进程 | 命令 | 最终启动 | 产生 / 消费的 topic |
|---|---|---|---|
| **camera** | `ros2 launch arachne_sensors gemini335.launch.py ... with_tf:=true camera_parent_frame:=ee_camera_link ...` | 相机驱动 + TF | 发布 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`、ee_camera TF |
| **depth_pointcloud** | `ros2 run arachne_sensors depth_to_pointcloud --ros-args -p pointcloud_topic:=/arachne/debug/depth_points ... target_frame:=base_link ...` | depth_to_pointcloud 节点 | 订阅 `/camera/depth/image_raw` + camera_info,发布 **`/arachne/debug/depth_points`** |
| **viewer** | `ros2 run arachne_operator raw_image_viewer --topic /camera/color/image_raw ...`(`teach_panel.launch.py`:L602-608) | 图像查看窗口 | 订阅 `/camera/color/image_raw`(纯显示,无 topic 输出) |
| **grasp_server** | `ros2 launch arachne_operator grasp_task_server.launch.py execute_real:=true ...` | grasp_task_server 节点 | 服务 `/arachne/grasp_task/start\|stop\|preflight\|status`,话题 `/arachne/grasp_task/state`;内部调用 `/arachne/aubo/move_joint` action、`/arachne/gripper/command`、订阅相机 / 点云 |
| **cleanup_server** | `ros2 launch arachne_operator road_cleanup_task_server.launch.py patrol_pattern:=line ...` | road_cleanup_task_server 节点 | 服务 `/arachne/road_cleanup/start\|pause\|return_home\|stop\|status\|preflight`,话题 `/arachne/road_cleanup/state` |
| **slam**(可选,`slam_command` 默认未 autostart) | `ros2 launch arachne_nav nav2_lidar.launch.py with_lslidar_driver:=true with_pointcloud_to_scan:=true ...` | lidar 驱动 + pointcloud_to_laserscan + nav2 | 发布 `/scan`、lidar 点云、`/tf` |
| **yolo_view**(可选,`yolo_view_command`) | `ros2 run arachne_operator raw_image_viewer --yolo-model ...` | YOLO 分割查看器 | 订阅 `/camera/color/image_raw`,叠加分割结果 |

---

## 四、teach_panel 节点 —— 主控

文件:`src/arachne_operator/arachne_operator/teach_panel.py:277` 的 `TeachPanelNode` 是所有 topic 的中央枢纽,参数从顶层 launch 全量注入(`teach_panel.launch.py:131-393`)。它:

- **发布 `/cmd_vel`** → 驱动 `scout_*_driver`
- **订阅 `/odom`** ← 来自底盘
- **订阅 `/arachne/teach_visualization/joint_states`**(默认 `joint_states_topic`)← 来自 `teach_visualization_joint_states`
- **发布 `/arachne/gripper/command`**(L458 默认)→ 驱动夹爪
- **发布 `/arachne/aubo/teach_command`**(L460-461)→ 切换 Aubo 示教
- **发布 `/arachne/aubo/joint_velocity_command`**(L478-479 默认)→ `aubo_sdk_velocity_bridge` 速度控制
- **调用 action `/arachne/aubo/move_joint`**(L521-522)→ 运动到目标关节
- **调用 demo 服务**(`/arachne/demo/start_visual_grasp` 等)→ 触发 grasp/cleanup 服务器和托管进程
- 订阅 `/arachne/demo/state` 由 demo_orchestrator 产生,teach_panel 用来更新 UI

---

## 核心 topic × 启动来源 映射表

| topic / action / service | 产生者(节点) | 消费者 | 启动来源 |
|---|---|---|---|
| `/cmd_vel` | teach_panel | scout_waveshare_serial_driver | teach_panel 节点 / real_bringup |
| `/odom` | scout_waveshare_serial_driver | teach_panel、demo_orchestrator | real_bringup |
| `/joint_states` | aubo_official_status_probe | teach_visualization_joint_states | real_bringup |
| `/arachne/teach_visualization/joint_states` | teach_visualization_joint_states | robot_state_publisher、teach_panel | teach_visualization |
| `/arachne/display/robot_description` + TF | robot_state_publisher | rviz2 | teach_visualization |
| `/arachne/gripper/command` | teach_panel | ms42dc 驱动 | teach_panel 节点 / real_bringup |
| `/arachne/gripper/status` | ms42dc 驱动 | demo_orchestrator、teach_panel | real_bringup |
| `/arachne/aubo/teach_command` | teach_panel | aubo_teach_command_bridge | teach_panel / real_bringup |
| `/arachne/aubo/joint_velocity_command` | teach_panel | aubo_sdk_velocity_bridge | teach_panel / real_bringup |
| `/arachne/aubo/move_joint`(action) | aubo_move_joint_action_server | teach_panel、grasp_task_server | real_bringup |
| `/camera/color/image_raw`、`/camera/depth/image_raw`、camera_info | gemini335 相机驱动 | depth_to_pointcloud、viewer、YOLO viewer、grasp_task_server | demo_orchestrator 托管的 `camera` 进程(默认 autostart) |
| `/arachne/debug/depth_points` | depth_to_pointcloud | 抓取 / 清理任务服务器、RViz | demo_orchestrator 托管的 `depth_pointcloud` 进程(默认 autostart) |
| `/arachne/grasp_task/{start,stop,preflight,status}`(service) | grasp_task_server | teach_panel、demo_orchestrator | demo_orchestrator 托管的 `grasp_server` 进程(默认 autostart) |
| `/arachne/grasp_task/state` | grasp_task_server | teach_panel | 同上 |
| `/arachne/road_cleanup/{start,pause,return_home,stop,status,preflight}`(service) | road_cleanup_task_server | teach_panel、demo_orchestrator | demo_orchestrator 托管的 `cleanup_server` 进程(默认 autostart) |
| `/arachne/road_cleanup/state` | road_cleanup_task_server | teach_panel | 同上 |
| `/arachne/demo/*`(服务) | demo_orchestrator | teach_panel | demo_orchestrator.launch.py |
| `/arachne/demo/state` | demo_orchestrator | teach_panel | 同上 |
| `/scan`、lidar 点云 | lslidar_driver / pointcloud_to_laserscan | nav2、RViz | `slam_command`(默认**不**在 autostart 列表,需手动启动) |

---

## 关键结论

1. **真实运动 topic 的安全门**:所有真实运动接口(`/cmd_vel`、`/arachne/aubo/teach_command`、`/arachne/aubo/joint_velocity_command`、`/arachne/aubo/move_joint`、`/arachne/gripper/command`)只来自 `real_bringup.launch.py` 启动的驱动节点,而 `real_bringup` 受 `with_real_bringup` 控制(默认 true,但驱动本身又被 `use_scout/use_aubo/use_ms42dc` 子开关约束)。Aubo 路径还通过 `teach_flag_path` 和 `control_owner_path` 做示教 / 所有权门控。

2. **默认即启动**(`autostart_managed_processes`):相机、深度点云、viewer、grasp_server、cleanup_server;**默认不启动**:顶层 `gemini335.launch.py`(`with_camera:=false`)、SLAM / lidar(`slam_command` 不在 autostart 列表)、`yolo_view_command`。

3. **两处相机启动路径的区别**:顶层 `with_camera:=true` 直接 include `gemini335.launch.py`(独立,不复用 TF);demo_orchestrator 的 `camera` 托管进程也运行同一个 `gemini335.launch.py`,但带了 ee_camera 专用参数(`camera_parent_frame:=ee_camera_link`、`with_tf:=true`)。生产链路用的是后者。

4. **`start_teach_panel.sh` 本身的职责**:它只是 source 环境 → 确保干净 → `ros2 launch`;退出时 `cleanup` trap 调 `stop_real_stack.sh` 收尾。topic 启动逻辑全部来自 `teach_panel.launch.py` 及其 include 的子 launch + demo_orchestrator 的 subprocess 托管机制。