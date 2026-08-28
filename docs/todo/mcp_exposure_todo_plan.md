# MCP 对外接口暴露任务规划：视点规划 + 视点采图链（mcp_exposure_todo_plan）

> 状态：**已实施（2026-08-26 全量交付；2026-08-27 二审传输合流；2026-08-28 三审 Host 白名单 + 远程端到端验收 + 503 根因闭环，Phase 4/4.1 完成见 §5/§6；2026-08-28 四审 token-free 健康探针 /health，Phase 4.2）**——Phase 0-4.2 已按 §5 执行完成，见各阶段回写；验收对照 §6 已勾选项。
> 创建：2026-08
> 更新：2026-08（一审回写见上）→ 2026-08-26 实施完成：新增 `src/arachne_mcp` 包（两个 server + 独立 launch + 离线单测），teach_panel GUI/launch 双通道接入，`build_workspace.sh` 纳入新包并添加构建环境 token 清理；实施偏差见 §5。→ **2026-08-27 二审改造（传输层合流）**：按用户要求收敛端口占用与路径可读性，改为**单端口（默认 8123）+ 双可读路径**（`/arachne/viewpoint/plan`、`/arachne/viewpoint/acquire`）。实现：单进程 `mcp_servers` 聚合入口（宿主节点复用 `ExecutionMcpNode`，planner 能力以 `ViewpointPlannerApi` 绑定宿主，零新增 ROS 节点），两个 FastMCP 实例挂自研 ASGI 分发器（避免 Starlette Mount 的 307 斜杠重定向），路径级 bearer token（缺 token 拒启对应路径）；session manager 由 `serve_http_background` 在 uvicorn lifespan 之外显式 `run()`。**安全边界不变**：两个独立 tool 集、两个独立 token、规划只读/执行运动门控。两个旧独立入口（`viewpoint_planner_mcp`/`viewpoint_acquire_mcp`）保留为调试/dry-run 通道。真实冒烟：`curl` 401/404 + 官方 `mcp` 客户端双端 initialize/tools/list 全通。GUI 受管服务合并为单个 `mcp_servers`（状态行 `MCP Servers (:8123)`）。
> 来源：用户需求（把"视点规划 + 视点规划之后执行的链路"对外暴露给上游 agent）+ 2026-08 只读代码审查 + 用户确认的 4 项设计决策（§0.2）。
> **2026-08-27 JSON 端点暴露**：按用户要求把同五个 MCP 工具额外以纯 JSON HTTP 端点暴露（新增 `arachne_mcp/json_api.py`，单端口不变、复用同样的按能力分界 bearer token）：`/arachne/json/viewpoint/plan/{plan_viewpoints,list_reachable_viewpoints}`、`/arachne/json/viewpoint/acquire/{acquire_chain,status,cancel}`；请求体为工具参数 JSON，响应包 ok/result 或 ok/error；`acquire_chain` 运动门禁拒绝（success:false）映射为 HTTP 错误。新增离线单测 `test_json_api.py`（21 用例全过）+ HTTP 冒烟（401/400/503/404/405/200 全通）。
> **2026-08-27 运维修复 + 默认自启接入 + 面板真实端点端到端验证**：
> ① **托管 MCP 启动崩溃修复**——teach_panel 面板拉起的 `mcp_servers` 曾因 launch 默认命令生成空值参数规则 `-p autostart_managed_processes:=` 而在 rcl 参数解析阶段即崩溃（`RCLError: error not set`），`:8123` 永不监听；修复为从 `mcp_servers_server_command` 默认值删除该传参（冗余：`ExecutionMcpNode.__init__` 硬护栏本就无条件清空此参数）。重建后隔离域验证：节点 ready → 监听 :8123，无任何受管服务自启。
> ② **viewpoint_planner 纳入默认自启列表**——面板/launch 双侧默认值同步加 `viewpoint_planner`（`autostart_managed_processes = camera,depth_pointcloud,viewer,grasp_server,cleanup_server,viewpoint_planner`）：MCP 是 headless 只调服务不自启它（§2.4 约束），故规划服务提供方必须随面板常备，否则 `plan_viewpoints` 返回 "start the viewpoint planner first"。
> ③ **端到端实测（GUI 全家桶 + 受管 mcp_servers 在线）**：经 `:8123/arachne/viewpoint/plan` 标准流走 initialize→tools/list→`plan_viewpoints(target_xyz=[0.45,0,0.30],32)` 成功：plan_id `20260827_145411_063`，ik_reachable=9/32、collision_free=5/32、最终可达 5 点（含逐点 IK 关节角与碰撞间隙）；`list_reachable_viewpoints(plan_id)` 从共享目录回读一致。§6 验收新增两条已勾选项见下。
> **2026-08-28 三审改造（Host 白名单，折中方案）+ 远程端到端验收**：
> ① **问题**：LAN 远程 agent 调 MCP 被拒 **421 Invalid Host header**。根因：三处 `FastMCP(...)` 构造均未传 `host`，SDK 默认 `host="127.0.0.1"` 自动启用 DNS rebinding 防护（allowed_hosts 仅 `127.0.0.1:*/localhost:*/[::1]:*`，见 SDK `fastmcp/server.py` / `transport_security.py`），与 uvicorn 绑 `0.0.0.0` 的 LAN 服务设计意图冲突；本仓库代码无 CORS/Host 处理，拦截来自 SDK 传输层。
> ② **决策（用户确认折中方案）**：默认 **Host 校验关**（LAN 直连可用，`bind_host=0.0.0.0` 意图与服务行为一致；bearer token + 无 CORS 已封死浏览器路径，JSON overlay 本就无 Host 检查，全开后两面属性对齐）；可选 **`mcp_allowed_hosts` 白名单**（env `ARACHNE_MCP_ALLOWED_HOSTS`，逗号分隔）。SDK 匹配语义：`host` 或 `host:*`（通配仅覆盖端口，host 精确匹配到冒号，如 `192.168.1.132:*`）；**显式配置会替换 SDK 隐式 loopback 放行**——收紧时需自加 `127.0.0.1:*` 才能保住本机客户端。
> ③ **实现**：`common.py` 新增 `parse_allowed_hosts`/`transport_security_for`/`build_fastmcp`（单一构造路径，Host 策略不漂移）；三入口（聚合 + 两个 standalone）全部换用；参数：聚合 `mcp_allowed_hosts`、standalone `allowed_hosts`，launch 双侧（`mcp_servers.launch.py` + `teach_panel.launch.py` 命令串）透传 env。新增离线单测 `test_transport_security.py`（10 用例：解析边界、settings 开关、经真实 SDK 的 200/421 行为）。
> ④ **新增远程验收脚本 `scripts/remote/test_mcp_remote.py`**：纯 stdlib（Windows `py` 直接跑，零依赖），4 项只读检查（无 token 401 / 带 token JSON status 200 / correct-Host 200 / forged-Host 行为自动判定 + `--expect open|allowlist` 断言）；本地"真实 SDK + 真实中间件"双模式副本验证（收紧 4/4、全开 MCP 分支 2/2）。
> ⑤ **运维修复（`-p name:=` 空值崩溃第二例）**：`ARACHNE_MCP_ALLOWED_HOSTS` 未导出时命令串展开出 `-p mcp_allowed_hosts:=`（空值），rcl 拒绝（`error not set`）——与 2026-08-27 ① 同根因但彼时只修了单一参数。通用化修复：`common.sanitize_ros_args()` 在三个入口 `rclpy.init` 前过滤空值 `-p/--param` 规则（参数回落声明默认值 = 未导出语义）；修复过程中二次修正实现（丢弃空值规则时需连同其 `-p` 旗标一起丢，否则下一条规则旗标重复）。
> ⑥ **端到端实测（真机 + Win 主机）**：面板 `enable_mcp:=true` + token 导出重启后，Win 主机 `test_mcp_remote.py --robot-ip 192.168.1.132` **4/4 PASS**（无 token 401、带 token JSON status 200、correct-Host 200、forged-Host 报告 allowlist OFF）；机器人侧日志核对：监听 `0.0.0.0:8123`、`host allowlist: off (LAN open)`、`motion_enabled=False`/`confirm_agent_motion=False`、事件日志无任何 acquire/motion 记录（测试全程只读）。
> **2026-08-28 三审续：远程 503 根因闭环 + 请求日志 + 日志消减**：
> ① **请求日志**：`common.log_requests()` ASGI 中间件（三入口最外层，401 也留痕）。每请求记录 method/path/body/响应状态；`Authorization` 值强制脱敏（只留 scheme）。日志经 ROS logger → stderr → 面板 tee 落 `mcp_servers.log`。
> ② **503 根因闭环（真机日志实锤）**：两层原因先后确认——(a) 服务侧 `viewpoint_plan_service_timeout_sec` 45s 窗口偏紧（最坏 IK 规划 ~28s），放宽至 **60s**（teach_panel 默认 + standalone + launch 三处同步，GUI 面板调用同参受益）；(b) **客户端 HTTP read timeout（~12s 量级）< 规划时延（实测 12.5s）**——SSE 流响应头先行返回（协议语义），结果事件在流尾等待，客户端在静默期超时断开（DELETE 会话），服务器结果未能送达。客户端日志证据：header 全字段正确（accept/session-id/protocol-version），自研 httpx 封装（`python-httpx2/2.12.0`）；次要 bug：DELETE 后复用旧 session-id 重连 → 404 `Session has been terminated`。**修复全在客户端**：read timeout ≥60s、超时后新建会话、参数名纠正（`radius`→`radius_m`、`filter_by_ik`→`filter_ik_reachable`、`filter_by_collision`→`filter_collision_free`——pydantic 忽略未知字段曾静默落到默认值）。
> ③ **日志消减（防日志膨胀）**：默认模式收敛为每请求 2-3 行（request 行+body、response 状态行），**响应体仅非 2xx 记录**（错误可诊断，17KB 级 plan JSON/SSE 块不再刷屏）；`include_headers`/`include_response_bodies` 开关保留排深挖能力；截断 2048→1024B。远端验收依赖的请求头分析已完成使命，默认关闭。
> ④ **遗留运维瑕疵（未修，记录在案）**：面板自启对 `viewpoint_planner` 的 "already available" 判定可为 DDS 陈旧发现假阳性，导致规划服务空窗（11:28~13:05 的 503 窗口）；将来可加存活复验。
> 目标读者：后续接手实现 MCP 暴露的开发者；文档应可独立执行或续接。实施时按 §5 阶段执行并回写进度。
> **2026-08-28 四审新增：token-free 健康探针 `GET /health`**（Phase 4.2，见 §5）：`common.with_health_check` 三入口统一接入（聚合 `mcp_servers` service=`arachne_mcp`、两个 standalone service=`viewpoint-planner`/`viewpoint-acquire`），位于 bearer-guard **之外**——探针只报"监听进程存活"，不携带任何能力/数据，无鉴权不改变既有安全边界（受保护路径无 token 仍 401，单测证明）；用于 agent/脚本/监控 `curl http://<host>:8123/health` 判断 MCP 是否启动（免 MCP initialize 握手、免 token）。新增 `test_health.py` 13 用例，49 项离线单测全过，真实 uvicorn 监听冒烟通过；**远程验收脚本 `test_mcp_remote.py` 同步扩展 2 项 health 检查（总 6 项）**。

---

## 0. 任务总结

### 0.1 需求原文（用户）

> "视点规划部分的功能，需要对外暴露接口，我的上游是一个 agent。目前只暴露视点规划，以及视点规划之后执行的链路，后续可能会有其他的拓展。帮我分析：是使用一个 MCP 全部集成，还是多个 MCP 分别接入。"

需求归纳：

1. 把**视点规划**（`viewpoint_planner`）能力暴露给远程 agent。
2. 把**视点规划之后执行的链路**（视点采图链：选点 → 移动到视点 → 采图/落盘）也暴露给同一 agent。
3. 面向未来**可扩展**（后续会有 nav/感知/标定/其他任务等新能力域）。
4. 采用 **MCP（Model Context Protocol）** 作为对外接口协议。

### 0.2 已确认设计决策（用户 confirm，见上文问答摘要）

| 决策项 | 结论 |
| --- | --- |
| MCP server 组织方式（核心问题） | **推荐：按"能力域 + 安全边界"拆成多个 MCP server，而非一个全集成**；现阶段最小拆成 **2 个**（规划 + 执行）。详见 §2.1 |
| 部署拓扑（agent 位置） | **agent 远程、机器人（Jetson）本地** → MCP **必须走 HTTP/SSE（Streamable HTTP）传输**（非 stdio）；MCP server 跑在 ROS 侧，对外暴露带**鉴权**的 HTTP endpoint |
| 权限隔离 | **暂不确定**是否要按 agent 隔离权限 → 先把 **server 边界 = 权限边界**的设计做对（规划只读、执行可动分属两个 server），将来要隔离时无需重构；不为不存在的情况预建 RBAC 框架 |
| 执行链范围 | **仅视点采图链**（规划→选点→移动到视点→采图/落盘），**不含**下游 grasp/cleanup 任务 server |
| 实现底座 | **纯新增 MCP server，不动现有 `arachne_agent_bridge`**（其 JSON-topic 桥保持原样） |
| 新增包 | 新建独立包（建议 `src/arachne_mcp`），两个 server 各一个 console_script；不并入 `arachne_agent_bridge` |
| 视点数据流（一审新增） | **共享 plan 文件 + plan_id**：planner MCP 把每次规划落盘到**共享 plan 目录**（参数化，见 §2.3.1）并返回 `plan_id`；execution `acquire_chain(plan_id, ...)` 用 `_parse_viewpoint_plan_file`（`teach_panel.py:562`，纯函数）读取。两 server 无代码耦合、有审计痕迹；agent 不搬运关节角浮点数组 |
| headless 化（一审确认） | **坚持 §2.4 方案 A + 强制约束清单**：MCP 实例关 autostart、关 teach_config_autoload、节点改名、runtime 目录隔离、明确 MoveIt joint_states 喂源归属（§2.4/§4.2） |
| control-owner 身份（一审新增） | execution MCP **沿用 `teach_panel` owner 名**：与 GUI 同一身份互斥竞争 `/tmp/arachne_aubo_control_owner`，天然防双头运动；驱动层 owner 白名单（`real_bringup.launch.py`）零改动。GUI 运动中 agent 调用将被拒并返回明确原因 |
| 启动与暴露面（一审定案） | **双通道控制：GUI 按钮 + launch 启动参数并存**：<br>1. **GUI 按钮**：在 `MANAGED_SERVICE_COMMAND_PARAMS` 注册 `mcp_servers`，在 `teach_panel` GUI 的 "Runtime Services" 中提供 **Start / Stop 按钮** 与状态监控（与其他受管服务一致），随时手动启停。<br>2. **Launch 启动参数**：在 `teach_panel.launch.py` 中新增 `enable_mcp:=false`（或加入 `autostart_managed_processes` 联动），传 `enable_mcp:=true` 即可随 launch 一并自启动 MCP 进程。<br>3. **独立场景**：提供独立 `mcp_servers.launch.py` 支持完全无 GUI 部署。<br>默认值均为 `false`（保持 opt-in 默认安全红线）；`bind_host`/端口/token 全参数化（token 走 env）。 |

### 0.3 合理性评估 / 风险项

- ✅ **按能力域 + 安全边界拆分（2 个 server）合理**：视点规划只读（`PlanViewpoints` srv，只采点不移动）与视点采图链**真实移动机械臂**（`move_arm_to_joints_degrees`→速度流式），安全档位截然不同。合并到一个 server 等于"能规划的表面天然够得到运动路径"，违反仓库"motion 必须走显式 enable/确认"红线（AGENTS.md）。拆分由**安全 + 独立演进**两个与权限无关的理由支撑，即便权限隔离暂不需要也成立。此时拆只要几分钟，运行后再拆（重配 agent / 重接 endpoint）才贵。
- ✅ **不是更多 server**：执行链目前只有"视点采图链"一个域，超出一个即过度设计；后续每个新能力域再各自 +1。
- ⚠️ **风险项 1（真机安全，一审修正）**：execution MCP 一旦被 agent 调用即可能运动。需沿用现有 motion gate（`motion_enabled` / 确认开关，对齐 `agent_bridge` 的 `_require_motion_allowed`（`agent_bridge.py:323-327`）语义）；**默认关闭运动，需显式 enable 才能实际移动**。**运动主路径是 MoveIt 碰撞检查规划 + SDK moveJoint 逐点执行**（`_move_arm_to_joints_worker`，`teach_panel.py:4289`，默认 `moveit_plan_enabled=true` 且 `moveit_refuse_on_plan_failure=true`），速度流式 + `_arm_keepout_violation` 实时拦截仅是默认被拒的降级路径——因此 execution MCP 的运行依赖**还包括 `/plan_kinematic_path`（MoveIt）与 `/arachne/moveit/joint_states` 喂源**（后者由 TeachPanelNode 自发布，`teach_panel.py:1245`；GUI 不在时 MCP 实例自己喂）。运动还经过 `_sdk_enter_control_owner`（`:5404`，排他 owner 文件）+ `_sdk_enter_gate`（`:5426`，teach flag），MCP 路径**不得绕过**（AGENTS.md 红线）。
- ⚠️ **风险项 2（采图链 GUI 绑定）**：采图链当前的核心方法（`viewpoint_acquire_begin`/`viewpoint_acquire_point`/`move_arm_to_joints_degrees`）在 **`TeachPanelNode`（headless 节点类，`teach_panel.py:749`）**，但**整链编排在 GUI（`TeachPanelApp`）**：Tk `messagebox.askyesno` 确认、`root.after` 回 UI 线程、`_viewpoint_acquire_busy` 门禁（`teach_panel.py:7240-7405`）。MCP server 是 headless 远程进程，**不能直接复用 GUI 驱动**。头less化方式见 §2.4，是本次实施最关键的开放决策，需用户拍板。
- ⚠️ **风险项 3（远程鉴权）**：MCP 走 HTTP 暴露在网络上（agent 远程）。需传输层鉴权（token / 配置），且**不能**把真实运动放到任何默认开放路径。规划 server（只读）与执行 server（可动）**应各自独立端口 + 独立鉴权**，便于将来只给规划。**一审加严**：明文 HTTP + bearer 在 LAN 可被嗅探——`bind_host` 参数化、token 走环境变量（禁入库）、声明仅可信 LAN/生产反代 TLS；MCP 进程默认 opt-in 不随主 launch 启动（§5 Phase 3）。
- ⚠️ **风险项 4（新增依赖）**：MCP 是标准协议，实现它需引入 MCP SDK（**已定案：官方 `mcp` 1.x**）。这**不属于** AGENTS.md "为简单 stdlib/ROS 原生工作加依赖"的禁止情形（对外协议接口是实质复杂度，且非 ROS 原生能力）。已核实官方 `mcp` 1.x 全系兼容 Python 3.10（Humble），实施时**锁定 `mcp>=1.0,<2`**（防 2.x 行为变更）；若与 rclpy 同进程冒烟卡壳，可退化为极薄的自实现 HTTP-JSON 桥（不推荐，非标准 MCP）。
- ⚠️ **风险项 5（新进程/生命周期）**：新增 MCP 进程纳入 `teach_panel` 的**受管服务（Managed Services）体系**（`teach_panel.py:130`，`MANAGED_SERVICE_COMMAND_PARAMS`），支持**双通道启停**：① 在 GUI 上通过 Start/Stop 按钮动态启停；② 在 `teach_panel.launch.py` 中通过 `enable_mcp:=true` 参数（联动 `autostart_managed_processes` 注入 `mcp_servers`）随面板自启动。默认均为 `false`（不自启，保持 opt-in 默认安全）；不新增 `scripts/` 稳定 runtime wrapper（AGENTS.md）。规划 server 依赖 `viewpoint_planner` 服务在线；执行 server 依赖采图链所需节点（相机/**MoveIt**/运动链路）在线。
- ⚠️ **风险项 6（一审新增：双 TeachPanelNode 实例冲突）**：方案 A 下 execution MCP 与 GUI teach panel 是两个 `TeachPanelNode` 实例：`__init__` 副作用包括 80Hz 速度 timer、**`/arachne/moveit/joint_states` publisher（`:1245`，重复 publisher 属 AGENTS.md 评审标准中的 operator 回归项）**、5 路相机订阅、`_autostart_managed_processes_once` 自动拉受管进程、`teach_config_autoload` 默认 true、每次实例化新建 `runtime_log_dir` 并**改写 `latest` 符号链接（`:1428-1438`）**。必须以约束清单关闭/隔离（§2.4、§4.2）；MCP 采图数据落在 MCP 自己的 runtime 目录，tool 结果须返回绝对路径供操作员索引。
- ⚠️ **风险项 7（一审新增：`cancel_acquire` 是新开发）**：GUI 采图链当前**无取消路径**（busy flag + on_success 链一路跑完，`:7242-7405`）。cancel 需主动中断在途 MoveIt/SDK 运动（stopJoint / 停速度流）+ 链终止 + manifest 收尾，属安全敏感代码，工作量不可按"对齐 safe_stop"低估，需单独验收（§6）。

---

## 1. 现状调研（2026-08 只读代码审查结论，带 file:line）

### 1.1 现有对外接口：`arachne_agent_bridge`（JSON-topic 桥）

- 包 `src/arachne_agent_bridge/`：单节点 `AgentBridge`（`agent_bridge.py`），通过 ROS **topic/service**（非 HTTP）向外部 agent 暴露工具表面。
- `TOOL_SCHEMAS`（`agent_bridge.py:37-138`）：`get_robot_state` / `safe_stop` / `base_velocity` / `base_relative` / `base_turn` / `arm_cartesian_jog` / `arm_joint_jog` / `arm_stop` / `gripper` / `aubo_teach`，各带 `danger`（read_only/stop/motion/mode_change）与 JSON `schema`。
- 运动门禁：`_require_motion_allowed`（`agent_bridge.py:323-327`）要求 `motion_enabled:=true` **且** `confirm_agent_motion:=true`，否则 `PermissionError`——**这正是我们执行 MCP 要复用的安全语义**。
- 传输：`command_topic`/`status_topic`/`event_topic`/`tools_topic`（默认 `/arachne/agent/*`）+ 三个 Trigger 服务（`/arachne/agent/status|tools|safe_stop`）。**是 ROS 内/局域网桥，不是外网 HTTP，也不是 MCP。**
- **决策**：按用户确认，**该桥保持原样**，MCP 为全新实现。

### 1.2 视点规划：`viewpoint_planner`（只读、可复用）

- 服务 `/arachne/viewpoint/plan`（`PlanViewpoints.srv`，`src/viewpoint_planner_msgs/srv/PlanViewpoints.srv`）：请求 `target_xyz_base[3]/strategy/radius_m/num_viewpoints/filter_ik_reachable/filter_collision_free/up_vector_base[3]`；响应 `success/message/Viewpoint[]`。
- `Viewpoint.msg`：`stamp/camera_xyz_base/camera_quat_xyzw/tool0_xyz_base/tool0_quat_xyzw/ik_reachable/ik_joints[6]/collision_free/collision_clearance`——**已有完整可达性（IK+碰撞）逐点标记**。
- 节点 `viewpoint_planner_node.py`：`_plan_inner`（IK 过滤 → 碰撞过滤 → 构造 Viewpoint），**只规划不运动**；可被 teach_panel auto-start（`_call_viewpoint_plan_worker`）。
- **MCP 规划 server 只需薄包一层**：把 `PlanViewpoints` 服务调用序列化/反序列化为 MCP tool（可直接复用 `teach_panel._serialize_viewpoint_plan`（`teach_panel.py:497`，模块级纯函数）那套 JSON 结构）。零运动、零安全门槛。

### 1.3 视点采图链：核心在 headless 节点，编排在 GUI

- 采图**核心方法在 `TeachPanelNode`（headless 节点类，`teach_panel.py:749`）**（非 GUI）：
  - `viewpoint_acquire_begin`（`:1715`，建批次目录 + 初始 manifest）。
  - `viewpoint_acquire_point`（`:1760`，单点采 RGB/depth/cloud/camera_info + 位姿快照，落盘 `<session>/viewpoint_acquisition/<run_id>/<n>/`）。
  - `move_arm_to_joints_degrees(deg, on_success=...)`（`:4235`）→ `_move_arm_to_positions_velocity`（`:4479`，速度流式 + keepout 实时拦截），`on_success(success:bool)` 传递到位成败。
  - `_current_arm_vector`（`:5747`）。
- 采图链**整链编排在 GUI `TeachPanelApp`**（`_viewpoint_acquire_all`，`:7242`）：`_viewpoint_acquire_all`（确认框）、`_acquire_start_motion`/`_acquire_on_motion_done`（`on_success` 链）、`_viewpoint_acquire_busy` 门禁、`root.after` 回 UI。
- 落盘结构 `viewpoint_acquisition/<run_id>/{color_<ts>.png,depth_<ts>.png,cloud_<ts>.pcd}` + `manifest.json` + `events.jsonl`（`viewpoint_acquire_*` 事件带 run_id）。
- **结论**：采图链的"逐点运动 + 逐点采集落盘"原子能力已存在于 headless 节点类，可复用；缺的是**无 GUI 的链式驱动 + 确认 + 状态 + 取消**——即 §2.4 的决策点。（一审补：GUI 链当前**没有取消路径**，cancel 为新开发，见风险项 7。）

### 1.4 MCP 现状

- 仓库**当前无任何 MCP 依赖/实现**（`grep -rn mcp src` 无命中）。
- 无 HTTP server 基础设施（无 fastapi/starlette/aiohttp）。
- ROS 2 Humble → Python 3.10，MCP SDK 需兼容该版本（§0.3 风险项 4）。

---

## 2. 整体设计：MCP server 边界与拓扑

### 2.1 核心结论：**2 个 MCP server，按"安全边界 + 能力域"拆分**

- **`viewpoint-planner` MCP（只读，安全）**：暴露视点规划 tool。无运动、无 gate。
- **`execution` MCP（运动，gated）**：暴露视点采图链 tool。复用 motion gate，默认不可运动。

拆分依据（§0.3）：安全档位不同（只读 vs 真移动）+ 独立演进（未来扩展各 +1 server）+ 权限边界天然落在 server 上（将来要隔离时无需重构）。**不合并**成"一个 MCP 全集成"——那是把安全和演进耦合回一个 monolith。

目标拓扑：

```
Jetson（ROS 侧）                         远程 agent（上游）
┌───────────────────────────────┐      ┌──────────────┐
│ src/arachne_mcp（新包）        │      │              │
│                               │      │    LLM       │
│ viewpoint-planner MCP         │◄─token──┤   agent     │
│   (Streamable HTTP :8123)     │      │              │
│   plan_viewpoints        ◄──────►(调 /arachne/viewpoint/plan)│
│                               │      └──────────────┘
│ execution MCP                 │      ┌──────────────┐
│   (Streamable HTTP :8124)     │◄─token──┤   agent     │
│   acquire_chain / ...   ◄──────►(复用 TeachPanelNode 核心)│
└───────────────────────────────┘      └──────────────┘
```

- 两个 server = 两个 MCP 实例（tool 集/token 各自独立）。**一审为两个进程两端口；二审变更为单进程单端口 + 双路径（/arachne/viewpoint/plan、/arachne/viewpoint/acquire，默认 :8123），执行 server 另加 motion gate**。
- 均以受管进程纳入启动（§5 Phase 3，`enable_mcp` opt-in），不新增 `scripts/` wrapper。
- 命名说明（一审）：实现时建议执行 server 的进程/脚本名用 `viewpoint_acquire_mcp`（`execution` 过泛，未来 grasp/nav 各自 +1 server 时命名撞车）；本文档其余处沿用"execution MCP"指同一物。

### 2.2 传输、鉴权与 SDK 选型（决策点）

- **传输**：Streamable HTTP（MCP 标准远程传输，一对多、支持流式工具输出，替代旧的 HTTP+SSE）。两个 server 各自独立端口。
- **鉴权**：每个 server 一个 bearer token（只读与执行各一个，便于只给其一）；agent 侧 MCP 配置里带 token。默认**无 token 拒绝连接**。**token 走环境变量**（如 `ARACHNE_MCP_PLANNER_TOKEN` / `ARACHNE_MCP_EXECUTION_TOKEN`），禁止入库（AGENTS.md secrets 红线）。
- **网络暴露（一审新增）**：`bind_host` 参数化（默认 `0.0.0.0`，单机调试可收 `127.0.0.1`）；明文 HTTP 仅用于可信 LAN，生产建议反代 TLS；可动真机的 endpoint 不依赖网络隐蔽性做安全。
- **SDK 选型（已定案：官方 `mcp` Python SDK 1.x，用户确认）**：
  - 首选：**官方 `mcp` SDK 1.x**（`pip install "mcp>=1.0,<2"`），用其内置 FastMCP 高层 API（`from mcp.server.fastmcp import FastMCP`），Streamable HTTP 原生支持。选它理由：官方 MCP 组织维护、协议演进同步；py3.10 全系兼容（已查 PyPI 元数据，1.0~1.29 均 `>=3.10`）；内置 OAuth/bearer 鉴权；高层 API 开发体验与 fastmcp 相当。
  - 进程模型：MCP server 与 ROS 节点同进程——rclpy 主线程 `spin()`，ASGI（uvicorn）在独立后台线程跑自己的 asyncio 事件循环；两套事件循环互不阻塞（Phase 0 冒烟核心验证点）。
  - 备选：极薄自实现 JSON-RPC over HTTP（不推荐，非标准 MCP）。
  - 实施 Phase 1 做最小冒烟（`mcp` server 能起来、能上 `tools/list`），再进入业务 tool。

### 2.3 各 server 的 tool 清单（对齐"仅视点采图链"范围）

**viewpoint-planner MCP（只读，无 gate）**

| tool | 说明 |
| --- | --- |
| `plan_viewpoints(target_xyz, strategy, radius, count, filter_ik, filter_collision, up_vector)` | 调 `/arachne/viewpoint/plan`，序列化结果**落盘共享 plan 目录**为 `viewpoint_plan_<plan_id>.json`，返回 `plan_id` + `Viewpoint[]`（含 `ik_reachable`/`collision_free` 逐点标记） |
| `list_reachable_viewpoints(plan_id)` | 从共享目录读指定 plan（`_parse_viewpoint_plan_file` 纯函数），列出 `ik_reachable && collision_free` 点（index + tool0 + joints 摘要）。**按 plan_id 寻址而非进程内存缓存**（一审修正：缓存重启即失、与 GUI 规划互不感知） |

**execution MCP（运动，默认拒绝，需 motion gate）**

| tool | 说明 |
| --- | --- |
| `acquire_chain(plan_id, indices | "all")` | 触发视点采图链：**从共享 plan 目录按 `plan_id` 读视点表**（一审定案的数据流，§2.3.1），过滤可达点后按 indices/全部 顺序移动→采集→落盘；返回每点成败与落盘路径 |
| `get_acquire_status()` | 轮询采图链进度/状态/上一轮结果（对齐 `acquire_chain` 异步执行） |
| `cancel_acquire()` | 安全停止当前采图链：**主动中断在途运动**（stopJoint / 停速度流）+ 终止链 + manifest 收尾（**新开发**，GUI 现无取消路径，见风险项 7） |

> **motion gate**：execution server 侧复用 `agent_bridge` 的语义——`motion_enabled` / `confirm_agent_motion` 两个参数缺一拒绝运动（`agent_bridge.py:323-327`）。agent 触发 `acquire_chain` 时若未 enable 则返回明确拒绝，默认值必须**关闭**。

#### 2.3.1 共享 plan 目录约定（一审定案）

- 新增参数 `viewpoint_plan_share_dir`（两 server 同值；建议默认为固定受管路径如 `<runtime_log_root>/viewpoint_plans/`——**不**用各自 runtime_log_dir，否则两进程互相不可见）。
- planner MCP 写入、execution MCP 只读；文件即审计痕迹，沿用既有 `viewpoint_plan_<stamp>.json` 格式 + `_parse_viewpoint_plan_file` 解析（`teach_panel.py:562`）。
- `plan_id` = 文件时间戳 stem；`acquire_chain`/`list_reachable_viewpoints` 对不存在/解析失败的 plan_id 返回明确错误。
- 清理策略：保留最近 N 份（参数，默认 20），启动时惰性清理；不起后台线程。
- GUI 链路不变（仍写自己 session 的 plan 文件）；若未来要让 MCP 执行 GUI 发起的规划，再让 GUI 也写一份到共享目录（本阶段不做）。

### 2.4 执行 server 的头less化方式（**最关键的开放决策**，需用户拍板）

采图链整链编排在 GUI（§1.3），MCP 是 headless 远程进程，需一种 headless 驱动方式。三案对比：

| 方案 | 做法 | 优点 | 缺点 |
| --- | --- | --- | --- |
| **A（推荐）MCP 复用节点核心，自持采图策略** | execution MCP server 作为一个 headless ROS 节点，同进程复用 `TeachPanelNode` 的 `move_arm_to_joints_degrees`/`viewpoint_acquire_point`/`viewpoint_acquire_begin`，自己实现链式编排 + 状态 + motion gate | 零/最小改动 teach_panel；符合 AGENTS.md"一个节点持有一个策略"；最大化复用已验收的逐点能力；不动 GUI | 需实例化一个含相机订阅/参数的大节点（复用成本）；GUI 的 `messagebox` 确认改由 MCP 的 enable+参数显式确认代替 |
| **B 抽独立 headless 采集 task server** | 参照仓库既有 task-server 模式（`grasp_task_server`/`step_cleanup`/`road_cleanup` 的 status+start 服务 + `state` topic，`teach_panel.py:141-159`），把采图链下沉为 `/arachne/viewpoint_acquire/*` 服务，GUI 与 MCP 都调它 | 与该仓库模式高度一致；解耦 GUI；MCP 只薄包一层服务；天然支持状态轮询/取消 | **代码量最大**：要把 teach_panel 里已验收的采图逻辑迁移到新节点并动 teach_panel 的调用方；有回归风险 |
| **C 最薄：不暴露整链，暴露逐点 tool** | MCP 只暴露 `move_to_viewpoint(index)` / `capture_frame()` 两个离散 tool，由 agent 自行编排"选点→移动→采图"多轮 | 实现最省；不碰 teach_panel | 违背"暴露整条执行链路"的意图；把链式编排/成败处理丢给 agent，可观测性和安全性更弱 |

**推荐 A**：贴合"纯新增、不动 bridge"和 AGENTS.md 的"单节点单策略"，复用已实机验收的采图核心，把 GUI 确认平移为 motion gate + 显式 enable 参数。若评审希望更贴近现有 task-server 模式、或希望 GUI 与 MCP 共用同一执行入口，则选 **B**。C 仅作了解。

**一审定案：方案 A + 强制约束清单**（针对风险项 6 的双实例冲突，全部为启动参数/命名层约束，不改 `TeachPanelNode` 代码）：

- `autostart_managed_processes:=[]`（MCP 实例**不**拉起任何受管进程，避免与 GUI 重复拉起 viewpoint_planner 等）。
- `teach_config_autoload:=false`（不加载 GUI 示教配置）。
- 节点改名（如 `arachne_execution_mcp`），不与 GUI 的 `arachne_teach_panel` 同名。
- MCP 实例用**独立 `runtime_log_root`**（或接受 `latest` 符号链接与 GUI 分离）；tool 结果返回**绝对落盘路径**供 agent/操作员索引。
- `/arachne/moveit/joint_states` 喂源归属：GUI 在时由 GUI 喂；GUI 不在时 MCP 实例自喂（同进程 TeachPanelNode 本就会发，`:1245`）——**验收用 `ros2 topic info` 确认 publisher 数符合预期**（§6）。
- 两实例并存期间 GUI 手动运动与 MCP 链式运动由 control-owner 文件天然互斥（§4.1），不新增应用层锁。

---

## 3. 需求一：viewpoint-planner MCP（只读，安全）

### 3.1 实现

- 新包 `src/arachne_mcp`（建议名），console_script `viewpoint_planner_mcp` / `execution_mcp`。
- 节点内嵌 MCP server（Streamable HTTP，端口参数化，默认 8123），同一 ROS 进程内创建 `PlanViewpoints` service client（`create_client`，服务名参数化默认 `/arachne/viewpoint/plan`）。
- `plan_viewpoints` tool：校验/规约入参 → 调服务（超时参数，对齐 teach_panel `viewpoint_plan_service_timeout_sec` 默认 45s）→ `_serialize_viewpoint_plan` 序列化 → **落盘共享 plan 目录**（§2.3.1）→ 返回 `plan_id` + 序列化字典。
- `list_reachable_viewpoints(plan_id)`：从共享目录读文件，返回 `ik_reachable && collision_free` 的点（index + tool0 + joints 摘要）；plan_id 缺失/解析失败返回明确错误。

### 3.2 实现要点

- 纯只读，**无任何 motion 路径**，不需要 gate。
- 依赖 `viewpoint_planner` 服务在线；服务未启动时返回明确错误（可提示 auto-start 方式，但不依赖 debug 工具——规划服务是主链路）。
- 独立 token 鉴权（只读 token）。

---

## 4. 需求二：execution MCP（运动，gated）

### 4.1 交互与门禁

- **motion gate（默认关闭运动）**：复用 `agent_bridge._require_motion_allowed` 语义：`motion_enabled:=true` 且 `confirm_agent_motion:=true` 才能实际触发运动；否则 tool 返回明确拒绝。默认值必须关闭（真机安全红线）。
- **control-owner / teach 门控（一审新增，AGENTS.md 红线）**：运动复用 `move_arm_to_joints_degrees` 链路，自然经过 `_sdk_enter_control_owner`（排他 owner 文件）+ `_sdk_enter_gate`（teach flag）。**owner 名沿用 `teach_panel`**（一审定案）：与 GUI 同一身份互斥——GUI 运动中 agent 触发将被拒并返回明确原因，反之亦然；驱动层 owner 白名单零改动。不新增任何绕过 owner/teach-gate 的入口。
- **异步执行 + 状态**：`acquire_chain` 异步启动（worker 线程），返回立即受理；`get_acquire_status` 轮询；`cancel_acquire` 触发安全停止（**主动中断在途运动**：MoveIt/SDK moveJoint 路径走 stopJoint，速度路径停流清零；链终止后 manifest 收尾——GUI 现无此能力，为新开发，见风险项 7）。
- **路径安全（一审修正）**：主路径为 MoveIt 碰撞检查规划（`/plan_kinematic_path` + `/arachne/moveit/joint_states` 喂源），规划失败默认拒动；`_arm_keepout_violation` 实时拦截仅作用于速度流式降级路径（`on_success(False)` 的点跳过并记录）。不新增绕过 MoveIt/keepout 的入口。

### 4.2 实现要点（按 §2.4 选定方案）

- 方案 A（一审定案 + 约束清单，§2.4）：execution MCP server 实例化/复用 `TeachPanelNode` 的采图核心方法，自持链式编排（逐点 `move_arm_to_joints_degrees(..., on_success=采集下一环)`）+ 独立 busy 门禁 + 每点事件落盘（`viewpoint_acquire_point` 已写）。
- 视点来源：`acquire_chain(plan_id, ...)` 从共享 plan 目录读表（§2.3.1），过滤 `ik_reachable && collision_free` 后按 indices/全部执行；**不接受 agent 直接传入关节角数组**（避免 LLM 搬运浮点出错）。
- 每点落盘沿用既有批次结构（`viewpoint_acquisition/<run_id>/`，位于 MCP 实例自己的 runtime 目录），返回逐点成败与**绝对落盘路径**。
- 状态/事件通过 MCP 工具结果 + ROS `events.jsonl` 双记录；`_write_event`（`teach_panel.py:1440`）可复用。
- 真机运动仍走显式 enable + confirm + control-owner/teach-gate，不放到默认路径。

---

## 5. 实现阶段（规划；实施时按此执行并回写）

> 说明：以下为**规划草案**，具体拆分/范围以用户审核后的修订为准。

**Phase 0：SDK 与依赖冒烟（✅ 完成）**

- ✅ 安装 `mcp==1.29.1`（`mcp>=1.0,<2`，Py3.10 兼容已核实）；冒烟：FastMCP Streamable HTTP + uvicorn 后台线程 + rclpy 主线程共存，`initialize`/`tools/list`/`tools/call` 端到端通过（httpx 客户端实测）。
- ✅ 新增包 `src/arachne_mcp`（`setup.py` + `setup.cfg`（`install_scripts=$base/lib/<pkg>`，ros2pkg 可发现）+ `package.xml` + `resource/`）；两个 console_script：`viewpoint_planner_mcp` / `viewpoint_acquire_mcp`（执行 server 进程名按 §2.1 命名建议用 `viewpoint_acquire_mcp`，避免未来 grasp/nav +1 时撞名）。

**Phase 1：viewpoint-planner MCP（只读）（✅ 完成）**

- ✅ `arachne_mcp/viewpoint_planner_mcp.py`：节点 `arachne_viewpoint_planner_mcp`，内嵌 FastMCP（Streamable HTTP，端口参数化默认 8123）+ `PlanViewpoints` client（服务名参数化默认 `/arachne/viewpoint/plan`）。
- ✅ `plan_viewpoints`：入参校验 → 调服务（超时参数默认 45s）→ `_serialize_viewpoint_plan` 序列化 → 落盘共享 plan 目录 → 返回 `plan_id` + 完整逐点数据 + 绝对路径。
- ✅ `list_reachable_viewpoints(plan_id)`：`_parse_viewpoint_plan_file` 解析，过滤 `ik_reachable && collision_free`，返回 index/tool0/joints 摘要；缺失/解析失败明确报错。
- ✅ 只读 token 鉴权：bearer 中间件；**token 未配置则拒绝提供服务**（`assert_token_configured`，对齐"无 token 拒绝连接"验收）；`bind_host` 参数化（默认 0.0.0.0，经模块常量 + env 覆盖）。

**Phase 2：execution MCP（运动，gated）（✅ 完成）**

- ✅ 方案 A：`arachne_mcp/execution_mcp.py` 中 `ExecutionMcpNode(TeachPanelNode)`（节点名 `arachne_execution_mcp`，`TeachPanelNode.__init__` 增加可选的 `node_name` 参数，默认不变）。
- ✅ 约束清单：launch 传入 `autostart_managed_processes:=[]`/`teach_config_autoload:=false`/独立 `runtime_log_root`（`log/arachne_execution_mcp`）；**并在 `ExecutionMcpNode.__init__` 加硬性护栏**——无论启动参数如何，强制 `autostart_managed_processes` 为空（实测：不带约束参数直接 ros2 run 也不会拉起任何受管服务；实施中发现裸启动会拉起 grasp_server 等，已封堵）。
- ✅ 三 tool：`acquire_chain(plan_id, indices|all)`（motion gate 默认关 + busy 门禁 + 共享目录读表 + 过滤可达点 + 异步 worker 链式逐点 `_move_arm_to_joints_worker`→`viewpoint_acquire_point`→manifest/events 落盘，返回逐点成败与绝对路径）、`get_acquire_status()`、`cancel_acquire()`（置 cancel 标志 + `stop_arm_velocity_hold` 停流 + SDK `stopJoint` 主动中断 + join 收尾 manifest，新开发能力）。
- ✅ motion gate 语义复用 `agent_bridge._require_motion_allowed`（`motion_enabled` + `confirm_agent_motion` 双开才放行，默认关）；control-owner 沿用 `teach_panel` 名（`aubo_sdk_control_owner_name` 默认值，未改驱动层）。

**Phase 3：启动接入 + 测试与验收（✅ 完成）**

- ✅ **GUI 按钮通道**：`teach_panel.py` 的 `MANAGED_SERVICE_COMMAND_PARAMS` 注册 `mcp_planner_server`/`mcp_execution_server`（命令参数化，默认空串）；Home Tab "Runtime Services" 快捷按钮（MCP Planner/MCP Exec）+ Details 列表行（状态 + Start/Stop）与 snapshot 状态接入。
- ✅ **Launch 参数通道**：`teach_panel.launch.py` 新增 `enable_mcp`（默认 `false`）+ `mcp_planner_server_command`/`mcp_execution_server_command`（默认命令含 token env 展开 `${ARACHNE_MCP_*_TOKEN:-}`）；`enable_mcp:=true` 时经 `PythonExpression` 把 `mcp_planner_server,mcp_execution_server` 注入 `autostart_managed_processes`（已实测求值）。
- ✅ **独立 Launch 通道**：`src/arachne_mcp/launch/mcp_servers.launch.py`（real_bringup + moveit_planning + camera + 两 MCP 节点；motion gate 默认关，`mcp_motion_enabled:=true mcp_confirm_agent_motion:=true` 显式开启）。
- ✅ 测试：`py_compile`、两 server `--dry-run-check`（离线吐 tool 清单）、`teach_panel --headless-check`、`arachne check entrypoints`、全量 `build_workspace.sh`（18 包）全部通过；离线单测 13 项全过（`src/arachne_mcp/test/`：plan 写/读 round-trip、`_parse_viewpoint_plan_file` 复用、plan_id 错误/路径穿越、prune 保留 N、gate 拒绝路径（motion_enabled/confirm 双关/单开）、indices 解析、cancel 无活动链 no-op）；端到端 HTTP 冒烟：两 server 带 token 起服务 → 401 无 token → `tools/list` → `plan_viewpoints` 缺规划服务返回明确错误 → `acquire_chain` gate 拒绝。
- ✅ 构建集成：`build_workspace.sh` PACKAGES 列表加入 `arachne_mcp`；**新增构建环境 token 清理**（unset `VSCODE_CLI_REQUIRE_TOKEN`/`ARACHNE_MCP_*_TOKEN`，防止 colcon env dump 把凭据烘进 `build/**/colcon_command_prefix_*.sh.env` 与构建日志——gitleaks 曾因此命中历史构建产物，已清理并恢复 0 findings）。

**实施偏差（2026-08-26）**：

- teach_panel 最小改动超出"仅复用导入"：`TeachPanelNode.__init__` 增加 `node_name` 参数（默认 `arachne_teach_panel`，GUI 行为不变）；`_status_callback` 两处 `is False/is True` 改为 `==`（历史 S3403 级联，语义等价）；`viewpoint_planner_msgs` 导入分组归位（ruff I001）。
- token 语义：配置了 token 时无/错 token → 401；**token 未配置时 server 拒绝启动**（比"默认拒绝连接"更硬，属实施加严，防止裸奔暴露）。
- `mcp_servers` 拆为 `mcp_planner_server` + `mcp_execution_server` 两个受管服务（独立启停/独立状态），而非单一 `mcp_servers`。
- 新增 `pyrightconfig.json`（extraPaths 指向 src + ROS + user site）与项目级 `ruff.toml`（镜像 pi-lens 规则 + 对 legacy 文件 per-file-ignores 冻结存量 lint）——解决新包/历史文件的 Pyright-ROS 环境性误报与历史 lint 级联，非功能改动。

**Phase 4：Host 白名单折中方案 + 远程端到端验收（✅ 完成，2026-08-28）**

- ✅ **Host 校验策略**：`common.py` 新增 `parse_allowed_hosts` / `transport_security_for` / `build_fastmcp` 单一构造路径；三入口（聚合 `_build_app` 两实例 + 两个 standalone）全部换用；默认关（LAN open），可选 `mcp_allowed_hosts`（聚合）/`allowed_hosts`（standalone）+ env `ARACHNE_MCP_ALLOWED_HOSTS` 收紧。SDK 语义：`host`/`host:*` 通配仅覆盖端口；显式配置替换隐式 loopback 放行。
- ✅ **launch 透传**：`mcp_servers.launch.py`（DeclareLaunchArgument + env 默认 + 节点参数）、`teach_panel.launch.py`（`mcp_servers_server_command` 追加 `-p mcp_allowed_hosts:=${ARACHNE_MCP_ALLOWED_HOSTS:-}`）。
- ✅ **测试**：新增 `src/arachne_mcp/test/test_transport_security.py`（10 用例：解析边界、settings 开关/Origin 派生、经真实 SDK ASGI 的 200/421 行为——收紧模式白名单与 loopback 过、同网段其他主机 421）；`SanitizeRosArgsTest` 2 用例（空值 `-p name:=` 回归）；31 项离线单测全过。
- ✅ **远程验收脚本**：`scripts/remote/test_mcp_remote.py`（纯 stdlib，Windows 零依赖），4 项只读检查 + `--expect open|allowlist` 断言；本地双模式副本（真实 SDK + 真实 `require_token_by_path`）验证。
- ✅ **真机端到端**：见头部 ⑥——Win 主机 4/4 PASS，机器人日志状态互证。

**实施偏差（2026-08-28）**：

- `-p name:=` 空值崩溃为**第二例**（2026-08-27 ①只修了 `autostart_managed_processes` 单点），本次以 `sanitize_ros_args` 通用化修复并接入三入口；实现曾引入"旗标残留"bug（丢弃空值规则未连同 `-p` 旗标），被精确复现脚本抓出后重写为索引式实现。
- 远程测试脚本落于 `scripts/remote/`（既有远程联调工具目录，非新 wrapper 入口）。

**Phase 4.1：远程 503 闭环 + 请求日志（✅ 完成，2026-08-28 续）**

- ✅ `log_requests()`：全请求日志（含脱敏 header 开关）→ 面板 `mcp_servers.log`；靠它拿到客户端 `user-agent=python-httpx2/2.12.0` 与完整时序，定位客户端 read timeout ~12s < 规划时延 12.5s。
- ✅ 服务超时 45s→60s 三处同步（真机实测 12.5s 规划在 45s 窗口内本应成功，客户端侧才是最终瓶颈；放宽服务窗为将来更慢的规划留裕量）。
- ✅ 日志消减：默认 2-3 行/请求，响应体仅非 2xx；verbose 开关保留。
- ✅ 客户端侧修复清单已交付（超时/会话重建/参数名），机器人侧无需再动。
- ⚠️ 遗留：面板自启 "already available" 假阳性可致 planner 空窗（未修，记录于头部 ④）。

**Phase 4.2：token-free 健康探针 `/health`（✅ 完成，2026-08-28）**

- ✅ `common.with_health_check(app, service_name)`：ASGI 包装器，`GET/HEAD /health` 返回 200 + `{"status":"ok","service":<名>}`（`content-type: application/json`、`cache-control: no-store`、HEAD 空 body），其余路径（含 POST /health）原样透传。常量 `HEALTH_PATH = "/health"`。
- ✅ **三入口统一接入**（与 `build_fastmcp` 同款"单一构造路径"防漂移思路）：
  - 聚合 `mcp_servers._build_app`：`log_requests(with_health_check(require_token_by_path(_dispatch, ...), "arachne_mcp"), ...)`；
  - standalone `viewpoint_planner_mcp` / `viewpoint_acquire_mcp`：`log_requests(with_health_check(require_token_by_path(mcp_app, {"/": token}), "viewpoint-planner"/"viewpoint-acquire"), ...)`。
  - **关键点**：standalone 的 bearer-guard 规则是 `{"/": token}`（全路径拦截），`/health` 若在其内侧会被 401——因此 health 必须置于 guard **之外**；聚合 guard 是具体前缀，`/health` 本就不匹配，外置同样成立且语义一致。
- ✅ **安全边界不变**：探针只报"监听进程存活 + service 名"，不携带任何能力/状态/数据；受保护路径无 token/wrong token 仍 401（单测覆盖两种组合）。
- ✅ 用途：agent/脚本/监控 `curl http://<host>:8123/health` 判断 MCP 是否启动，免 MCP initialize 握手、免 token；`mcp_servers` 模块 docstring 与 `--dry-run-check` 输出均已标注。
- ✅ 测试：新增 `test_health.py` 13 用例（GET/HEAD/POST 透传/带 query、standalone 与聚合两种鉴权组合下 health 无 token 200 + 受保护路径 401）；全套 49 项离线单测通过；真实 uvicorn 监听冒烟（health 200、无 token 401、请求日志 request/response 两行）。**远程验收脚本 `scripts/remote/test_mcp_remote.py` 同步扩展**：新增 2 项 health 检查（token-free GET：200 + body `{"status":"ok","service":"arachne_mcp"}`；HEAD：200），总检查数 4→6（docstring 编号 1-5 相应更新）；本地真实中间件模拟聚合监听器复验 6/6 PASS，`--expect allowlist` 断言分支（forged 项 FAIL、exit=1）符合预期。

---

## 6. 验收标准（规划草案，待审核）

- [x] 新增 `src/arachne_mcp` 包，两个 console_script 可独立启动，各自暴露 HTTP（Streamable HTTP）MCP endpoint，独立端口 + 独立 token 鉴权；无 token 拒绝连接。（2026-08-26 实测：`ros2 run` 带 token 起服务，无 token 401；token 未配置拒绝启动）
- [x] **viewpoint-planner MCP**：`plan_viewpoints` 返回与 `_serialize_viewpoint_plan` 一致的序列化结果（含 `ik_reachable`/`collision_free` 逐点标记）；`list_reachable_viewpoints` 正确过滤；全程只读、无任何运动路径。（单测 round-trip + HTTP 冒烟）
- [ ] **execution MCP**：`acquire_chain` 触发采图链并逐点落盘（复用既有 `viewpoint_acquisition/<run_id>/` 结构）；`get_acquire_status` 轮询；`cancel_acquire` 安全停止。（gate/indices/cancel 状态机已离线单测；**在途运动实机中断需真机验收**）
- [x] **motion gate**：未 `motion_enabled`/`confirm_agent_motion` 时 `acquire_chain` 明确拒绝、绝不运动；默认关闭。（单测 + HTTP 冒烟双验证）
- [x] **数据流**：`plan_viewpoints` 返回 `plan_id` 且共享目录出现对应 plan 文件；`acquire_chain(plan_id)` 能读到同一份视点表；错误/过期 plan_id 明确报错。（round-trip 单测 + 共享目录默认 `log/viewpoint_plans`）
- [ ] **共存**：GUI teach panel 与 execution MCP 同时运行——`ros2 topic info /arachne/moveit/joint_states` publisher 数符合预期；MCP 实例不拉起受管进程、不加载 teach config；双方同时触发运动时 control-owner 互斥生效、失败方收到明确原因。（**已单测：MCP 实例不拉起受管进程（硬护栏）；GUI+MCP 双实例 publisher 数、control-owner 互斥需真机验收**）
- [ ] **cancel**：采图链在途运动时 `cancel_acquire` 能主动中断运动（stopJoint/停流）并收尾 manifest；链路不留置位 busy。（状态机单测已覆盖 no-op/拒绝路径；**在途中断需真机验收**）
- [x] **暴露面与启停双通道**：
  - **按钮控制**：`teach_panel` 面板 Runtime Services 列表中有 MCP 服务的状态显示与 Start/Stop 按钮（已接入，点击启停路径与既有受管服务一致；GUI 实机点按需真机/显示环境验证）；
  - **Launch 参数**：`teach_panel.launch.py enable_mcp:=true` 能在面板启动时自动拉起 MCP 进程（PythonExpression 注入已实测求值），缺省 `enable_mcp:=false` 时默认不起；
  - **独立 Launch**：`ros2 launch arachne_mcp mcp_servers.launch.py` 在无 GUI 场景下独立拉起（launch 解析通过；完整拉起需真机环境）；
  - **鉴权与网络**：bind/token 参数生效，无 token 拒绝连接已验证（HTTP 冒烟 401）。
- [x] **托管启动健壮性（2026-08-27）**：teach_panel 面板拉起受管 `mcp_servers` 成功监听 `:8123`；修复 launch 默认命令空值参数导致的 rcl 解析崩溃，并验证无任何受管服务自启（headless 硬护栏生效）。
- [x] **面板链路端到端规划（2026-08-27）**：MCP tool → `/arachne/viewpoint/plan` ROS 服务全链实测成功（GUI 全家桶在线），plan 落盘共享目录且 `list_reachable_viewpoints` 按 plan_id 回读一致；依赖的 `viewpoint_planner` 已纳入面板默认自启列表。
- [x] **无回归**：`arachne_agent_bridge` 零改动；`viewpoint_planner` 零改动；`teach_panel` 仅最小改动（`node_name` 参数 + 受管服务注册 + GUI 行/按钮 + snapshot + 两处 `is False/is True`→`==`）；`--dry-run-check`/`--headless-check`/全量构建/离线单测通过。
- [x] **远程连通**：本地 HTTP 冒烟 `tools/list`/`tools/call` 通过；**2026-08-28 已在真实跨主机网络实测**——Win 主机 `test_mcp_remote.py --robot-ip 192.168.1.132` 4/4 PASS（401/200/Host 检查/allowlist 模式判定），机器人侧日志互证（监听 0.0.0.0:8123、allowlist off、motion gate 双关、无运动事件）。
- [x] `git status --short` 已检查。
- [x] **Host 白名单（2026-08-28 三审）**：SDK 隐式 localhost DNS-rebinding 防护与 LAN 服务意图的冲突已解决——默认关（LAN IP 直连可用），可选 `ARACHNE_MCP_ALLOWED_HOSTS`（`host`/`host:*`）收紧为 421；离线单测 10 项 + 本地双模式副本验证 + 真机 Win 主机 4/4 PASS；JSON overlay 无 Host 检查（设计如此），收紧时 MCP 面只会更严。
- [x] **托管启动健壮性通用化（2026-08-28）**：`sanitize_ros_args` 修复任意 `-p name:=` 空值（未导出 env 变量）导致的 rcl 解析崩溃（第二例，含旗标残留 bug 回归测试）；31 项离线单测全过。
- [x] **健康探针（2026-08-28 四审）**：`GET /health` 免 token 请求返回 200 + `{"status":"ok","service":...}`（三入口统一，聚合 service=`arachne_mcp`、standalone=`viewpoint-planner`/`viewpoint-acquire`）；HEAD 亦支持；探针位于 bearer-guard 之外且只报存活、无数据，受保护路径无 token/wrong token 仍 401（安全边界不变）；`test_health.py` 13 用例 + 全套 49 项离线单测 + 真实 uvicorn 监听冒烟通过；**远程验收脚本 `test_mcp_remote.py` 已含 health 两项检查**（本地模拟监听 6/6 PASS）。

---

## 7. 可选后续（本阶段不做）

- 按 agent 身份/权限隔离（多个上游 agent，各自只被授予部分 server/tool）——因"暂不确定"未立项，结构上已预留（server 即边界）。
- 暴露下游任务 server（grasp/cleanup）到执行 MCP（当前范围仅采图链）。
- 采图结果回灌 / 重规划循环（采集→感知→更新视点）。
- MCP 资源/提示（resources/prompts）补充，用于向 agent 提供世界模型/标定/机器人状态上下文。

---

## 8. 安全与流程约束（对齐 AGENTS.md）

- **真机运动红线**：execution MCP 触发运动必须走显式 `motion_enabled` + `confirm_agent_motion`，默认关闭；**control-owner（沿用 `teach_panel` 名）+ teach-gate 链路不得绕过**；运动主路径 MoveIt 碰撞检查保留，速度降级路径 keepout 实时拦截保留；不新增绕过任何一层 gate 的直接运动入口。
- 规划 server 是只读，永不接触运动。
- 不依赖 debug 工具（如 `depth_to_pointcloud`）；采图沿用主链路相机/运动入口。
- 不新增 `scripts/` 稳定 runtime wrapper；MCP 进程以 `teach_panel` 受管服务纳入体系，支持 **GUI Start/Stop 按钮** 与 **Launch 参数 `enable_mcp`** 双通道启停，默认不自启。
- 新增 MCP SDK 依赖（**已定案：官方 `mcp` 1.x**）属"对外协议接口"的实质复杂度（非简单 stdlib/ROS 原生任务），在 AGENTS.md 依赖规则内；版本锁定 `mcp>=1.0,<2`，与 rclpy 同进程共存须在 Phase 0 冒烟确认。
- 遵循"最小改动"：不动 `arachne_agent_bridge`，尽量不改 teach_panel 运动/采图核心（§2.4 方案 A）。
- `git status --short` 检查；硬件标定/端口/超时等走参数，不硬编码仓库相对路径。
