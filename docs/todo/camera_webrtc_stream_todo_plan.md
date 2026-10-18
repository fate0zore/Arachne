# 摄像头 WebRTC/WHEP 实时画面外发任务规划：MediaMTX + x264 软编 + info/snapshot HTTP 面（camera_webrtc_stream_todo_plan）

> 状态：**已回滚（2026-08-28：Phase 0-3 实施 + Phase 4 部分验收后，因相机 USB 重枚举异常导致在线验收受阻，用户要求回滚全部代码修改——详见文末 §8 回滚记录；本档保留完整调研/设计/实测结论供将来重启该任务时直接续用）**
> 创建：2026-08-28
> 来源：用户需求（外部 agent 需要机械臂摄像头实时画面接口：WHEP 视频流 + `/arachne/camera/info` + `/arachne/camera/snapshot.jpg`；明确要求"不要把视频帧塞进 MCP、SSE 或 FastAPI JSON"，推荐 MediaMTX 架构）+ 2026-08-28 只读调研（代码 + 本机硬件/软件栈 + MediaMTX 官方文档核实）。
> 决策记录：4 项设计决策已由用户确认（§0.2）。实施时按 §5 阶段执行并回写进度；验收对照 §6。
> 目标读者：后续接手实现的开发者；文档应可独立执行。

---

## 0. 任务总结

### 0.1 需求原文（用户，归纳）

1. 机械臂摄像头实时画面通过 **WebRTC/WHEP** 提供给外部 agent（Vue 前端 `<video>`），架构：
   `摄像头 → GStreamer H.264 → MediaMTX → WebRTC/WHEP`，播放地址形如 `http://192.168.1.132:8889/arm-camera/whep`。
2. 提供辅助 HTTP 接口（同一外部 agent 消费）：
   - `GET /arachne/camera/info`：在线状态、WHEP 地址、编码、分辨率、帧率、时间戳；
   - `GET /arachne/camera/snapshot.jpg`：断流占位、截图、报告用；
   - `WHEP /arm-camera/whep`：真正的视频流。
3. 明确禁止：把视频帧塞进 MCP、SSE 或 FastAPI JSON。
4. 现阶段只调研与规划，审核后执行。

### 0.2 已确认设计决策（用户 confirm，2026-08-28）

| 决策项 | 结论 |
| --- | --- |
| 视频源 | **Gemini 335 color**（`/camera/color/image_raw`，640×480@30 bgr8，现有 `gemini335_v4l2_node` 产出）；不新增 V4L2 设备占用 |
| 编码方案 | **x264 软编 640×480**（本机为 Jetson Orin Nano Super，无 NVENC 硬编，§1.2 实测）；目标 ~25-30fps、bitrate ~1.5Mbps，参数化可降档 |
| 鉴权 | **info/snapshot 免 token**（定位同 `/health`：只读探测面，不含能力/数据控制；WHEP 本身在 LAN 即可播放，snapshot 不高于流本身暴露面）；RTSP 推流仅绑 `127.0.0.1`，LAN 上不可见 |
| 生命周期 | **与 MCP 同模式**：teach_panel 受管服务（GUI Start/Stop + 状态行）+ `teach_panel.launch.py enable_camera_stream:=false`（默认关，opt-in）+ 独立 launch 支持无 GUI 场景 |

### 0.3 与用户推荐架构的三处偏差（均为调研实证后修正，需知悉）

| # | 用户原始设想 | 本机实证 | 修正方案 |
| --- | --- | --- | --- |
| 1 | GStreamer **硬件编码** H.264 | 本机是 **Jetson Orin Nano Super**（`/proc/device-tree/model` 实证），Orin Nano/NX 系列无 NVENC 硬核：`nvh264enc`/`nvv4l2h264enc`/`v4l2h264enc` 均不存在，`/dev/nvhost-nvenc` 不存在（§1.2） | **x264 软编**（ffmpeg `libx264`，已随系统安装）。640×480@30 ultrafast/zerolatency 约占 1~2 核（6 核 CPU），Phase 0 实测回写。Phase 0 顺带验证 `h264_omx`/`h264_v4l2m2m` 是否在本机可用（可用则作为 CPU 减负可选项，不作为主路径依赖） |
| 2 | GStreamer 直接从摄像头取流 | `/dev/video6` 被 `gemini335_v4l2_node` 以 `v4l2-ctl --stream-mmap=8` **独占**（`gemini335_v4l2_node.py:437-439`），第二个进程无法再打开同设备 | 取流源改为 **ROS topic** `/camera/color/image_raw`：流节点订阅后经 stdin 喂给编码进程（§2.2）。零 V4L2 冲突，且天然与现有相机进程解耦 |
| 3 | MediaMTX 提供 snapshot | MediaMTX **无内建 snapshot 端点**（官方文档的 snapshot 方案是 `runOnReady` 里起 ffmpeg 循环抓 RTSP，重且滞后，§调研来源） | `/arachne/camera/snapshot.jpg` 由流节点**直接对最近一帧 BGR 做 `cv2.imencode`**（§3.3）：实现最薄、断流前最后画面可用、不依赖编码器/MediaMTX 存活 |

### 0.4 风险项

- ⚠️ **风险项 1（CPU/RAM 余量）**：`free -h` 当前 available 仅 ~1.2GB（7.4GB 总量，5.9GB 已用）。x264 ultrafast 640×480@30 预估 +150~250MB、1~2 核；MediaMTX（Go 单进程）+~30-50MB。Phase 0 必须实测编码进程 CPU% 与 RSS 并回写 §5；超标则降 `preset`/`fps`/`bitrate`（全部参数化）。编码进程不得无限吃帧：节点侧**只喂最新帧、阻塞即丢帧**（§3.4），防止把系统拖垮。
- ⚠️ **风险项 2（暴露面）**：用户已确认 LAN 可信、info/snapshot/WHEP 免 token。等价语义：**LAN 内任何主机都能看到相机画面**。这与现有 MCP LAN-open 决策一致，但需在文档中显式声明（将来要收紧时：MediaMTX 侧加 publish/read 鉴权 + HTTP 面加 bearer，均为配置级改动，不需重构）。
- ⚠️ **风险项 3（Jetson IP 耦合）**：WHEP 的 ICE 候选需要 `webrtcAdditionalHosts: [192.168.1.132]`（MediaMTX 官方要求，否则浏览器拿到的候选不可达）；IP 变化需同步改配置。mediamtx.yml 中加注释标注此耦合；`whep_url` 同样参数化（§3.2）。
- ⚠️ **风险项 4（相机依赖）**：流依赖 `camera` 受管进程（gemini335 节点）在线发布 `/camera/color/image_raw`。相机离线时：info 返回 `online:false`、snapshot 返回 503、编码进程因 stdin 断供自然无产出（节点 watchdog 管理）。**不新增对 debug 工具（depth_to_pointcloud）的任何依赖**（AGENTS.md 红线）。
- ⚠️ **风险项 5（新增进程崩溃循环）**：编码子进程 / MediaMTX 子进程 watchdog 必须用**有上限指数退避**（对齐 `gemini335_v4l2_node._color_capture_worker` 的 backoff 模式：`0.2s → ×2 → 上限 ~30s`），节点退出即全部终止。杜绝 unbounded retries（AGENTS.md 评审标准）。
- ⚠️ **风险项 6（teach_panel 回归面）**：teach_panel 改动 = 受管服务注册 + GUI 一行 Runtime Services 条目 + launch 参数注入，全部复用 `enable_mcp`/`mcp_planner_server` 的既有模式（`teach_panel.py:135` `MANAGED_SERVICE_COMMAND_PARAMS`、`teach_panel.launch.py:458-465,934`），不动采图/运动/规划逻辑。默认 `enable_camera_stream:=false`，不改变现有启动行为。

---

## 1. 现状调研（2026-08-28 只读调研结论）

### 1.1 相机链路

- `src/arachne_sensors/arachne_sensors/gemini335_v4l2_node.py`：color 640×480@30 YUYV（`/dev/video6`，v4l2-ctl mmap 独占采集，`:437-460`）→ 发布 `/camera/color/image_raw`（**bgr8**，`_image_from_array`）+ `/camera/color/camera_info`；depth/pointcloud 并行。
- 受管服务名 `camera`，随 teach_panel 自启（`autostart_managed_processes` 默认含 `camera`）。
- **结论**：推流节点的正确挂点是订阅 `/camera/color/image_raw`（sensor QoS，best-effort 语义可选参数），绝不二次打开 V4L2 设备。

### 1.2 本机硬件/软件栈（2026-08-28 实测）

| 项 | 实测值 |
| --- | --- |
| 平台 | NVIDIA Jetson Orin Nano Super（JetPack R36.4.7，aarch64，6 核，RAM 7.4GB） |
| NVENC 硬编 | **无**：`nvh264enc`/`nvv4l2h264enc`/`v4l2h264enc` 均"没有那样的组件或插件"；`/dev/nvhost-nvenc` 不存在（Orin Nano/NX 无编码硬核） |
| GStreamer | 1.20.3（`gst-launch-1.0` 可用）；`nvv4l2camerasrc`/`nvv4l2decoder` 在但编码器不在；`rtspclientsink` **未装**（`gstreamer1.0-rtsp` apt 包有候选 1.20.1，未装） |
| PyGObject Gst | `python3-gi` 3.42.1 + `gir1.2-gstreamer-1.0` 已装，`Gst.init` 冒烟通过 |
| ffmpeg | 4.4.2，含 `libx264`（+ `h264_omx`/`h264_v4l2m2m` 存在但在 Orin Nano 上大概率不可用，Phase 0 顺带验证） |
| MediaMTX | 未安装；端口 8889/8189/8554 当前空闲 |
| 端口占用现状 | 仅 `:8123`（mcp_servers）监听 |

### 1.3 对外 HTTP 面现状

- `arachne_mcp` 单端口 `:8123`：bearer token 按路径分能力域 + `GET /health` 免 token（`common.with_health_check`，guard 之外）；自研 ASGI 分发 + uvicorn；Host 白名单默认关（LAN open）。
- **决策**：`/arachne/camera/info` + `/snapshot.jpg` **不挂到 8123**。理由：camera 面是独立能力域、独立 opt-in 服务（与 MCP enable 状态无关）；不跨包耦合。端口默认 **8124**。
- **实施修正（见 §5 Phase 2 / §5.1 偏差①）**：`arachne_env.sh` 导出 `PYTHONNOUSERSITE=1`，uvicorn 在文档化入口下不可导入——HTTP 面最终用 **stdlib `http.server.ThreadingHTTPServer`** 实现，零依赖。HTTP 面保持"免 token + GET-only + 无 CORS"的最小面。

### 1.4 MediaMTX 能力核实（官方文档/源，2026-08-28）

- WebRTC/WHEP：`webrtc: true`，HTTP(S) 监听 `webrtcAddress: :8889`（默认），`webrtcEncryption: false` 默认明文 HTTP——`http://<ip>:8889/<path>/whep` 直接可用；LAN 客户端可达性需 `webrtcAdditionalHosts: [<LAN IP>]`（官方 WebRTC-specific features 文档明确要求）；ICE 媒体走 UDP mux（默认 `:8189`）。
- RTSP 收流：`rtspAddress: :8554`（默认）、`rtspEncryption: "no"` 默认；**可绑 `127.0.0.1:8554`** 只收本机推流。
- 无内建 snapshot 端点（§0.3 偏差 3）。
- 实施时以**实际安装版本的 `mediamtx.yml` 模板**为准核对键名（版本间拼写有差异，如 `webrtcEncryption` 的 `false/no` 取值），安装脚本落地后一次性校准并回写 §5。

---

## 2. 整体设计

### 2.1 架构与端口

```
Gemini335 color（gemini335_v4l2_node，/dev/video6 独占）
        ↓  ROS topic（bgr8 640×480@30）
/camera/color/image_raw
        ↓  camera_stream 节点（新增，arachne_sensors）
        ↓  stdin 喂最新帧（阻塞即丢帧）
ffmpeg 子进程（libx264 ultrafast zerolatency，x264 软编 H.264）
        ↓  RTSP push（ANNOUNCE/RECORD）
MediaMTX（子进程，本机托管）
  ├─ RTSP  server  127.0.0.1:8554   ← 仅本机可推/可读
  ├─ WHEP  HTTP    0.0.0.0:8889     → http://192.168.1.132:8889/arm-camera/whep
  └─ ICE   UDP     :8189            （webrtcAdditionalHosts: [192.168.1.132]）
        ↓  WebRTC/WHEP
Vue 前端 <video autoplay muted playsinline>（WHEP reader）

camera_stream 节点同时托管（token-free，GET-only）：
  GET http://192.168.1.132:8124/arachne/camera/info          → JSON（§2.3）
  GET http://192.168.1.132:8124/arachne/camera/snapshot.jpg  → JPEG（最近一帧）
  （HLS/RTMP/SRT/录制/API 在 mediamtx.yml 全部关闭）
```

- 一个节点（`camera_stream`）持有整个 camera 外发面的策略：topic 消费、编码子进程、MediaMTX 子进程、HTTP 面。MediaMTX 是它的**子进程**（非独立受管服务），生命周期随之启停、崩溃由 watchdog 有界重拉——GUI 上只需一个 `camera_stream` 受管服务条目，启停顺序天然正确。
- AGENTS.md 对齐：不新增 `scripts/` 稳定 runtime wrapper；MediaMTX 二进制由 `scripts/setup/install_mediamtx.sh`（setup/bootstrap 类，允许）安装到 **repo 外** `~/mediamtx/`；配置文件随包发布（§4）。

### 2.2 编码与推流进程（进程模型选型）

| 方案 | 做法 | 结论 |
| --- | --- | --- |
| **A（选定）ffmpeg 子进程 + stdin 喂帧** | `ffmpeg -f rawvideo -pix_fmt bgr24 -s 640x480 -r 30 -i - -c:v libx264 -preset ultrafast -tune zerolatency -b:v 1500k -g 60 -f rtsp -rtsp_transport tcp rtsp://127.0.0.1:8554/arm-camera`；节点把最新帧 bytes 写入 stdin | **零新增依赖**（ffmpeg+libx264 已装）；与 `gemini335_v4l2_node` 的 subprocess 管道模式同风格；RTSP push 由 ffmpeg 完成，无需 `gstreamer1.0-rtsp` |
| B gst-launch fdsrc（需 `apt install gstreamer1.0-rtsp` 得到 rtspclientsink） | `gst-launch fdsrc do-timestamp=true ! rawvideoparse format=bgr ... ! x264enc ! rtspclientsink` | 备选；多一个 apt 依赖，无额外收益 |
| C PyGObject appsrc 全控制 | Python 内嵌 GStreamer | 代码量最大；A 失败再启用 |

- 帧时间戳：ffmpeg `-r` 固定帧率打时间戳，直播观看场景足够（非测量用途，不与 ROS 时间对齐——见 §3.5 已知限制）。

### 2.3 `/arachne/camera/info` 返回结构

用户要求字段全保留，另加三个只读运维字段（超集，向后兼容）：

```json
{
  "camera_id": "arm_camera",
  "online": true,
  "protocol": "webrtc",
  "whep_url": "http://192.168.1.132:8889/arm-camera/whep",
  "codec": "H264",
  "width": 640,
  "height": 480,
  "fps": 30,
  "timestamp": "2026-08-28T14:30:00+08:00",
  "snapshot_url": "http://192.168.1.132:8124/arachne/camera/snapshot.jpg",
  "rtsp_url": "rtsp://127.0.0.1:8554/arm-camera",
  "last_frame_age_ms": 120
}
```

- `online` = 最近一帧时间 < `online_stale_sec`（默认 3.0s）且编码子进程存活。`fps` 为配置目标值（参数 `stream_fps`）。
- `timestamp` 本地时区 ISO8601。HTTP 响应带 `cache-control: no-store`。

### 2.4 安全与暴露面汇总（评审重点）

| 面 | 绑定 | 鉴权 | 暴露内容 |
| --- | --- | --- | --- |
| RTSP :8554 | 127.0.0.1 | 无（本机可达性即边界） | 原始 H.264 流 |
| WHEP/HTTP :8889 | 0.0.0.0 | 无（用户确认 LAN 可信） | 视频画面 |
| HTTP :8124 info/snapshot | 0.0.0.0 | 无（同上，定位同 /health） | 状态 JSON + 最近一帧 JPEG |
| ROS topic | DDS 域内 | 现状不变 | 无新暴露 |

- 全部 GET-only、无写路径、无运动路径、无 ROS 能力外泄；不与 8123 的能力/执行面交叉。
- 不把视频帧放进任何 JSON/MCP/SSE（红线遵守）：snapshot 是独立二进制 JPEG 端点。

---

## 3. 组件设计

### 3.1 新文件清单（全部新增，零改既有逻辑文件）

| 文件 | 作用 |
| --- | --- |
| `src/arachne_sensors/arachne_sensors/camera_stream_node.py` | 流节点（console_script `camera_stream`）：订阅 → ffmpeg → MediaMTX → HTTP 面 |
| `src/arachne_sensors/config/mediamtx.yml` | MediaMTX 配置（setup.py `data_files` 随包安装到 share） |
| `src/arachne_sensors/launch/camera_stream.launch.py` | 独立无 GUI 启动（camera 可选带入 + camera_stream） |
| `src/arachne_sensors/test/test_camera_stream.py` | 离线单测（§5 Phase 2） |
| `scripts/setup/install_mediamtx.sh` | MediaMTX 安装脚本（pin 版本 + sha256 校验，装到 `~/mediamtx/`，repo 外） |

teach_panel 侧最小改动（复用 MCP 既有模式，`teach_panel.py:135` + `teach_panel.launch.py:458-465,934`）：
`MANAGED_SERVICE_COMMAND_PARAMS` 注册 `camera_stream`（命令参数 `camera_stream_server_command`，默认 `ros2 run arachne_sensors camera_stream --ros-args ...`）；GUI Runtime Services 增加一行；launch 增加 `enable_camera_stream`（默认 `false`）+ autostart 注入。

### 3.2 `camera_stream_node.py` 参数（全部参数化，无硬编码）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `image_topic` | `/camera/color/image_raw` | 订阅 topic（QoS：sensor data best-effort + 可靠 fallback 参数） |
| `stream_name` / `rtsp_push_url` | `arm-camera` / `rtsp://127.0.0.1:8554/arm-camera` | MediaMTX path 与推流地址 |
| `http_bind_host` / `http_port` | `0.0.0.0` / `8124` | info/snapshot HTTP 面 |
| `whep_public_url_base` | `http://192.168.1.132:8889` | 拼 `whep_url`；launch 默认值集中管理 |
| `stream_width/height/fps` | `640/480/30.0` | 喂 ffmpeg 的原始帧规格（须与相机发布一致，不一致时按实际帧 reformat 并告警） |
| `x264_bitrate_kbps` / `x264_preset` / `keyframe_interval` | `1500` / `ultrafast` / `60` | 编码规格（降档旋钮） |
| `online_stale_sec` | `3.0` | online 判定阈值 |
| `snapshot_max_age_sec` / `jpeg_quality` | `10.0` / `85` | 超龄拒绝（503）/ JPEG 质量 |
| `mediamtx_binary` / `mediamtx_config` | `~/mediamtx/mediamtx` / `<share>/config/mediamtx.yml` | MediaMTX 托管；binary 缺失→降级：仅 HTTP 面，`online:false` + 明确日志 |
| `mediamtx_restart_backoff_*` | 对齐 gemini backoff 模式 | 有界指数退避（上限 ~30s） |
| `encode_restart_backoff_*` | 同上 | ffmpeg 崩溃重拉退避 |

### 3.3 HTTP 面（stdlib `http.server.ThreadingHTTPServer` 后台线程；规划时拟用 uvicorn，实施时因 PYTHONNOUSERSITE 环境约束改为 stdlib，见 §5.1 偏差①）

- `GET /arachne/camera/info`：§2.3 JSON；节点未收到过帧→`online:false`，`last_frame_age_ms:null`。
- `GET /arachne/camera/snapshot.jpg`：最近一帧 `cv2.imencode('.jpg', ...)`；无帧或超 `snapshot_max_age_sec` → `503` + `text/plain` 原因；响应 `image/jpeg` + `no-store`；支持 `?ts=` 防缓存参数（忽略其值）。
- 方法/路径之外一律 404；非 GET 405（HEAD 无 body）；`Cache-Control: no-store`。`ThreadingHTTPServer` 在独立 daemon 线程，与 rclpy 主线程互不阻塞。
- **不做** bearer/CORS/Host 检查（用户决策）；端口冲突（8124 被占）→ 节点启动失败并明确报错（不自作聪明换端口）。

### 3.4 帧管道与背压（关键正确性点）

1. 回调线程：`Image` → `(header.stamp, np array view)` 存入**单槽 latest**（锁保护），永远覆盖旧帧。回调内不做编码/拷贝以外的工作。
2. 写线程（~fps 节奏或事件驱动）：取 latest → `proc.stdin.write(frame_bytes)`（921,600B/帧）。**write 前先检查上一次 write 是否已返回**；未返回（下游阻塞=编码慢或 mediamtx 未起）→ 直接跳过本帧（丢帧不排队，延迟恒定），计数器 `dropped_frames` 进入 info 运维日志。
3. ffmpeg 子进程 stderr → 节流日志（对齐 `_warn_color` 的 2s 节流模式）；退出码非 0/非 SIGTERM → 有界退避重拉。
4. MediaMTX 子进程：节点 init 启动（binary 存在时）；`destroy_node` 时 terminate→wait(2s)→kill（对齐 gemini 清理模式）；崩溃同款有界退避。
5. stdin 管道断开（ffmpeg 死）→ 捕获 BrokenPipeError → 触发重拉路径。

### 3.5 已知限制（如实记录，不掩盖）

- 时间戳非 ROS 时间（ffmpeg 固定 `-r` 打戳）：仅直播观看用途，不用于任何测量/投影（测量仍走 `/camera/color/camera_info` + 原始 topic 链路）。
- 帧率自适应：相机 30fps 且回调稳定时达 30；CPU 吃紧时 ffmpeg 输入端自然丢帧（节点主动丢最新帧策略），画面流畅性优先于完整性。
- MediaMTX path `arm-camera` 的读者是无限流（一个 path 多 WHEP reader 由 MediaMTX 扇出，与推流端解耦）。

---

## 4. MediaMTX 安装与配置

### 4.1 安装（`scripts/setup/install_mediamtx.sh`）

- 从 GitHub releases 下载 **pin 版本**的 `mediamtx_*_linux_arm64v8.tar.gz` + 校验 sha256（版本号与哈希以脚本变量落地，Phase 0 确定最新稳定版后 pin 死）。
- 安装到 `~/mediamtx/`（repo 外，不进 git）；脚本可重复执行（幂等）；不写 systemd（用户已选"与 MCP 同模式"，由节点托管子进程）。

### 4.2 `mediamtx.yml`（随包发布，Phase 0 按实装版本校准键名）

```yaml
# 核心（其余键全部维持模板默认/显式关闭）
rtsp: yes
rtspEncryption: no
rtspAddress: 127.0.0.1:8554        # 仅本机推流/读取

rtmp: no
srt: no
hls: no
webrtc: yes
webrtcEncryption: false
webrtcAddress: :8889
# ICE 候选可达性（官方要求）：Jetson IP 变化时同步改这里和 whep_public_url_base
webrtcAdditionalHosts: [192.168.1.132]
api: no
playback: no
pathDefaults:
  source: publisher
paths:
  arm-camera:
```

---

## 5. 实现阶段（规划；实施时按此执行并回写）

**Phase 0：MediaMTX 落地 + 编码冒烟（✅ 完成，2026-08-28）**

- [x] `scripts/setup/install_mediamtx.sh`：pin **v1.20.1**（`mediamtx_v1.20.1_linux_arm64.tar.gz`，sha256 `d1689f0b…` 校验），装至 `~/mediamtx/`（repo 外）。**网络适配**：本机 GitHub release 直连超时，脚本内置镜像链（ghfast.top → ghproxy.net → 直连）逐个尝试。幂等可重复执行。
- [x] `config/mediamtx.yml` 按 v1.20.1 校准：`rtspEncryption: "no"`、`webrtcEncryption: false`、`webrtcLocalUDPAddress`、`webrtcAdditionalHosts: [192.168.1.132]`；另在冒烟中发现默认配置会多开 **RTSP UDP :8000/:8001** 与 **MoQ :8892** 两个不必要 LAN 监听，已显式关闭（`rtspTransports: [tcp]`、`moq: false`）——暴露面比规划时更收敛。
- [x] 命令行冒烟通过：ffmpeg testsrc → RTSP push → MediaMTX path ready（`stream is available and online, 1 track (H264)`）→ RTSP 回读 30 帧成功；WHEP 端点存活（GET 405 / POST 空 SDP 400，符合 WHEP 语义）。
- [x] **资源实测回写**：ffmpeg libx264 640×480@30 ultrafast = **44% CPU（~0.44 核）/ 64MB RSS**；MediaMTX = **0.7% CPU / 43MB RSS**——远优于规划预估（1~2 核 / 150-250MB），无需降档，`h264_omx`/`h264_v4l2m2m` 无需验证。

**Phase 1：`camera_stream` 节点（✅ 完成，2026-08-28）**

- [x] `arachne_sensors/camera_stream_node.py`（console_script `camera_stream`）+ 参数表（§3.2 全参数化）+ `/camera/color/image_raw` 订阅（默认 sensor best-effort QoS，`reliable_qos` 可切）+ 单槽 latest。
- [x] ffmpeg 子进程管理（stdin 喂帧、阻塞即丢帧、stderr 节流日志、有界指数退避 0.2s→30s 封顶、健康运行 ≥10s 重置退避）。
- [x] MediaMTX 子进程托管（binary 缺失→降级模式仅 HTTP 面 + 明确日志；崩溃同款有界退避；节点退出 terminate→wait→kill）。
- [x] `--dry-run-check`：命令串构造、退避边界、FrameSlot 往返/超龄、info 合同、**真实 stdlib HTTP 服务于临时端口的路由自检**——纯离线零硬件。

**Phase 2：HTTP info/snapshot + 离线单测（✅ 完成，2026-08-28）**

- [x] **实现偏差（重要）**：HTTP 面改为 **stdlib `http.server.ThreadingHTTPServer`**，弃用规划中的 uvicorn。根因：`scripts/env/arachne_env.sh` 导出 `PYTHONNOUSERSITE=1`，uvicorn/starlette/mcp 均装在 `~/.local`（user site）被隐藏——按 AGENTS.md 文档化入口启动必然 `ModuleNotFoundError`。stdlib 零依赖、全环境可用（测试在 `PYTHONNOUSERSITE=1` 下跑通即证明）。GET-only + 404/405（HEAD 无 body）+ `Cache-Control: no-store` 语义与规划一致。
- [x] `test/test_camera_stream.py` **18 项离线单测**全过（`PYTHONNOUSERSITE=1` + 源码 PYTHONPATH 即可跑）：命令串形状、退避上限/重置、FrameSlot 最新帧制胜/超龄/JPEG 魔数、info 合同字段、真实 HTTP 路由（200/404/405/503/no-store）。
- [x] `py_compile`、`camera_stream --dry-run-check` 全过。

**Phase 3：launch + teach_panel 接入 + 构建（✅ 完成，2026-08-28）**

- [x] `camera_stream.launch.py` 独立通道（`with_camera:=true` 默认连带 gemini335.launch.py；`http_bind_host` 故意不设 launch 参数，注释说明通配绑定由节点 `WILDCARD_BIND` 常量承担，是 §2.4 既定 LAN-open 决策）。
- [x] teach_panel 四处最小接入：`MANAGED_SERVICE_COMMAND_PARAMS` 注册 `camera_stream`、`camera_stream_command` 参数（默认命令含 `whep_public_url_base:=http://192.168.1.132:8889`）、GUI 快捷按钮 + Runtime Services 状态行（`Camera Stream (:8889)`）+ 状态快照键。
- [x] `teach_panel.launch.py`：`enable_camera_stream`（默认 false）+ `camera_stream_command` 默认值 + autostart 注入表达式扩展为 mcp_servers/camera_stream 双条件链式追加。
- [x] demo_orchestrator **有意不接入**（其受管注册表本就不含 MCP/viewpoint_planner，同先例；面板是标准入口）。
- [x] 全量构建 18 包通过；`teach_panel --headless-check`、`arachne check entrypoints`、18 项单测、`--dry-run-check` 全部通过（含 `PYTHONNOUSERSITE=1` 严格环境）。

**Phase 4：端到端验收（◐ 进行中——降级模式自动化部分完成，完整在线验收转用户手动）**

- [x] **降级模式 E2E（无相机，自动化完成）**：`ros2 run arachne_sensors camera_stream` → HTTP 面 `:8124` 监听、info 200 `online:false`/`last_frame_age_ms:null`、snapshot 503、未知路径 404、非 GET 405、RTSP 仅 `127.0.0.1:8554` 监听、mediamtx 子进程稳定（启动 1 次，无重启循环）。
- [x] **watchdog 实证**：测试期间人为制造过 mediamtx 端口被孤儿进程占用 → 有界退避重拉按设计工作（0.2s→2×→30s 封顶，无 unbounded retry）；孤儿根因（下述偏差②）已修。
- [x] **干净退出实证**：SIGINT 节点 → mediamtx/ffmpeg 子进程消亡、8554/8889/8124 全部释放、无残留进程。
- [x] **实弹相机链路一次实测**：独立 launch 拉起 gemini335 + camera_stream，首帧到达、无编码/推流报错——但发现偏差③（bgr8 帧全丢 bug），修复后未及复测相机即不可用（见下）。
- [ ] **转用户手动验收**（§6.1 清单）：相机当前不可用——测试期间 `/dev/video6` 设备节点消失（仅剩 video0-3，疑似 USB 重枚举/掉线，先 `v4l2-ctl -d /dev/video6 --info` 确认）；浏览器 WHEP 播放、GUI 启停按钮、断流演练、Vue 前端接入均需人工。
- [x] `git status --short` 检查（无二进制/安装产物入库，见 §6）。

### 5.1 实施偏差（2026-08-28，全部实证驱动）

1. **HTTP 面弃用 uvicorn → stdlib `http.server`**（Phase 2 偏差，最重要）：规划时按 arachne_mcp 模式选 uvicorn；实机首跑即发现 `arachne_env.sh` 的 `PYTHONNOUSERSITE=1` 使 user-site 里的 uvicorn 不可见。改 stdlib 后反而更稳（零依赖）。⚠️ **遗留观察（未改动）**：mcp_servers 依赖的 `mcp`/uvicorn 同样在 user-site——若 teach_panel 经文档化入口（source arachne_env.sh）拉起，同样存在导入失败风险（测试时 :8123 即无监听）；属既有问题，建议后续单独核查 mcp_servers 的启动环境。
2. **初始化顺序加固**：HTTP 面先于 mediamtx/ffmpeg 子进程绑定——否则端口冲突（如 :8124 被占）会在子进程已拉起后才失败，留下孤儿 mediamtx。测试中实际发生过一次并借此发现。
3. **bgr8 帧全丢 bug（已修 + 回归单测）**：`_SUPPORTED_ENCODINGS` 用 `None` 表示“bgr8 无需转换”，但帧回调守卫把 `conversion is None` 误判为“编码不支持”，导致所有 bgr8 帧被丢。重构为纯函数 `convert_image_frame`（按成员关系判定）并新增 3 项回归单测。**该 bug 证明了“完整在线 E2E 不可省略”**。
4. MediaMTX 配置额外关闭 RTSP UDP 监听与 MoQ（Phase 0 冒烟发现的多余 LAN 面）。
5. `/dev/video6` 在测试中消失（设备节点仅剩 video0-3）：疑似 USB 重枚举/相机掉线，非代码问题；手动验收前需先确认设备在线。

---

## 6. 验收标准（2026-08-28 实施后回写；未勾选项 = 转用户手动验收，见 §6.1）

- [x] **架构红线**：视频帧只存在于 ROS topic → stdin → ffmpeg → RTSP → WebRTC 链路；JSON/MCP/SSE 响应体零帧数据（info 是状态 JSON、snapshot 是独立二进制 JPEG 端点；单测 + HTTP 自检覆盖）。
- [ ] **WHEP**：`http://<jetson>:8889/arm-camera/whep` 可被浏览器 `<video>`（WHEP reader）播放；分辨率/帧率与参数一致。（**待手动**：端点存活已验证——GET 405/POST 400；浏览器出图需相机在线 + 人工目视）
- [x] **info**：免 token 返回 §2.3 结构（含全部要求字段 + 3 个运维超集字段）；相机离线时 `online:false`、`last_frame_age_ms:null` 且不 500（降级 E2E 实测）。
- [x] **snapshot**：免 token 返回合法 JPEG（`\xff\xd8` 魔数单测）；无帧/超龄 503 实测。（真实画面 JPEG 需手动验收）
- [x] **RTSP 不外露**：实测 8554 仅 `127.0.0.1` 监听；8889/8124 为 `0.0.0.0`；另显式关闭了 MediaMTX 默认多开的 RTSP UDP :8000/:8001 与 MoQ :8892。
- [x] **生命周期**：默认不自启（`enable_camera_stream:=false`，默认 autostart 列表未动）；独立 launch 可无 GUI 启动（实测）；节点 SIGINT 退出后 mediamtx/ffmpeg 全部消亡、端口全释放（实测）；GUI 启停按钮已接入（**待手动点按**）。
- [x] **无回归**：现有 camera/MCP/采图链路零改动；teach_panel 仅受管服务注册 + GUI 一行 + launch 参数（`--headless-check` 过）；默认启动行为不变（`enable_mcp`/`enable_camera_stream` 均默认 false）；全量构建 18 包过。
- [x] **资源**：实测 ffmpeg ~0.44 核 / 64MB + mediamtx ~0.7% / 43MB，远低于规划阈值。
- [x] **测试**：18 项离线单测 + `--dry-run-check`（含临时端口真实 HTTP 自检）+ `--headless-check` + entrypoints 全过（`PYTHONNOUSERSITE=1` 严格环境）。

### 6.1 转用户手动验收清单（Phase 4 收尾）

前置：确认相机在线（`v4l2-ctl -d /dev/video6 --info`；本次测试中 video6 曾消失，若仍缺失需先恢复相机连接/上电）。

```bash
# 1) 拉起完整链路（独立通道，或用面板全家桶 + enable_camera_stream:=true）
ros2 launch arachne_sensors camera_stream.launch.py

# 2) info 应为 online:true，last_frame_age_ms 小于几百
curl -s http://127.0.0.1:8124/arachne/camera/info

# 3) snapshot 应为真实画面 JPEG
curl -s -o /tmp/s.jpg http://127.0.0.1:8124/arachne/camera/snapshot.jpg && file /tmp/s.jpg

# 4) Win 主机浏览器打开 WHEP 播放页（或 Vue 前端按 §7 接入）
#    http://192.168.1.132:8889/arm-camera/whep

# 5) 断流演练：面板 Stop "camera" → info online:false + snapshot 503；Start → 自愈

# 6) GUI Runtime Services 中 Camera Stream (:8889) Start/Stop 点按
#    停止后确认无 mediamtx/ffmpeg 残留：ps aux | grep -E 'mediamtx|ffmpeg'
```

---

## 7. 前端对接说明（交付给外部 agent / Vue 侧）

```html
<video ref="cameraVideo" autoplay muted playsinline></video>
```

```js
// 1) 拿地址（免 token）
const info = await (await fetch("http://192.168.1.132:8124/arachne/camera/info")).json();
// 2) WHEP 播放：可用 mediamtx 官方示例的 WHEP reader（fetch POST SDP offer → answer → RTCPeerConnection）
//    https://github.com/bluenviron/mediamtx 仓库内 mediamtx.js / 或自写 ~40 行 WHEP 客户端
const whep = new WHEPClient();
whep.play(info.whep_url, videoEl);        // 断流时轮询 info.online 做"信号丢失"占位
// 3) 截图/占位：直接 <img src="http://192.168.1.132:8124/arachne/camera/snapshot.jpg">
```

（视频播放地址与 info/snapshot 均免 token、跨端口直连；若前端走 HTTPS 站点，浏览器会拦混合内容——现状机器人侧为 LAN 明文 HTTP，前端也应 HTTP 部署，或将来统一上 TLS 时三处同步：8123/8889/8124。）

**易误判提示（2026-08-28 手动验收实测）**：直接用浏览器 GET `http://<ip>:8889/arm-camera/whep` 会返回 `{"status":"error","error":"method not allowed"}`——这是 **WHEP 协议端点的预期行为**（它只接受 POST SDP offer，GET 405 / 空 POST 400 均正常），不是故障。要看画面：① 浏览器打开 **`http://<ip>:8889/arm-camera`**（MediaMTX 内置 WHEP 播放页）；② Vue 前端按上文用 WHEP 客户端播放。

---

## 8. 回滚记录（2026-08-28，用户要求）

### 8.1 回滚范围与方式

| 项 | 处理 |
| --- | --- |
| `src/arachne_operator/arachne_operator/teach_panel.py` | `git checkout --` 恢复（含 2 处编辑期 lint 自动格式化） |
| `src/arachne_operator/launch/teach_panel.launch.py` | 同上 |
| `src/arachne_sensors/setup.py` | 同上 |
| `src/arachne_sensors/arachne_sensors/camera_stream_node.py` | 删除 |
| `src/arachne_sensors/config/mediamtx.yml` | 删除（目录为本次新建） |
| `src/arachne_sensors/launch/camera_stream.launch.py` | 删除 |
| `src/arachne_sensors/test/`（仅本次新建的 test_camera_stream.py） | 删除 |
| `scripts/setup/install_mediamtx.sh` | 删除（scripts/setup/ 目录为本次新建） |
| `install/`、`build/` 内的全部生成残留（executable/site-packages/config/launch/pycache） | 手动清除（colcon 不自动清理已删除文件） |
| 人工测试期拉起的 camera_stream/mediamtx 进程 | 已全部终止，端口 8554/8889/8124 释放 |
| `~/mediamtx/`（MediaMTX v1.20.1 二进制，repo 外） | **保留**（非代码，无害；将来重启任务可直接用） |
| 本文档 | 保留，状态改为"已回滚" |

### 8.2 回滚后验证

- `git status --short`：代码改动清零（仅余文档与用户既有 untracked 文件）。
- 全量构建 18 包通过；`teach_panel --headless-check`、`arachne check entrypoints` 通过。
- `ros2 pkg executables arachne_sensors` = `depth_to_pointcloud` + `gemini335_v4l2_node`（camera_stream 已消失）。
- 端口 8554/8889/8124 无监听，无 camera_stream/mediamtx 进程。

### 8.3 将来重启该任务的入口

- 方案设计与实测结论（编码选型、端口/安全面、资源实测、Python 陷阱与修正）全部在本文档 §0-§6，无需重新调研。
- 待办前置：① 相机 UVC 枚举异常（无 YUYV/Z16）需先恢复（§5 Phase 4）；② 建议改用 `/dev/v4l/by-id/usb-Orbbec_…-video-index*` 稳定设备路径（§6.1）。
- MediaMTX v1.20.1 已装于 `~/mediamtx/`，重装脚本内容可从本文档 §4.1/§5 Phase 0 还原。
