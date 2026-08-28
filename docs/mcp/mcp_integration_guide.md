# Arachne MCP 接入文档

> 版本：2026-08-28 | 端口：8123（单端口双路径）| 协议：MCP Streamable HTTP + 纯 JSON HTTP 双面
> 适用对象：上游 agent / MCP 客户端开发者
> 配套文件：`mcp_servers.example.json`（可直接粘贴的客户端配置）、`arachne_integration.json`（机器可读全量接口定义）
> 2026-08-28 更新：新增 **token-free 健康探针 `GET /health`**（判断 MCP 是否启动，无需 token、无需握手的存活检查，见 §2.3）

---

## 1. 概述

Arachne 在机器人侧（Jetson）通过 MCP（Model Context Protocol）对外暴露两个能力域，安全边界严格分离：

| 能力 | 路径 | 鉴权 token | 危险级别 |
| --- | --- | --- | --- |
| 视点规划（只读） | `http://<host>:8123/arachne/viewpoint/plan` | `ARACHNE_MCP_PLANNER_TOKEN` | read_only，**永不运动机械臂** |
| 视点采图链（运动） | `http://<host>:8123/arachne/viewpoint/acquire` | `ARACHNE_MCP_EXECUTION_TOKEN` | **motion**，默认被运动门禁锁死 |

核心规则：

- **规划面完全只读**。任何携带 planner token 的调用都不可能引起机器人运动。
- **执行面默认零运动**。只有机器人操作员在启动时显式同时开启 `mcp_motion_enabled:=true` 与 `mcp_confirm_agent_motion:=true` 两个 launch 开关后，采图链才会真正动臂。二开关缺一，所有运动请求被明确拒绝。
- 服务为 **opt-in**：由 teach_panel GUI 的 "MCP Servers" 按钮或独立 launch 启动；未运行时连接会被拒绝。
- **`/health` 是唯一免 token 端点**：只报告"监听进程是否存活"（`{"status":"ok","service":"arachne_mcp"}`），不含任何能力面、状态或数据；受保护路径的鉴权不受影响。

网络与明文传输注意事项：

- 明文 HTTP 仅用于可信 LAN。生产/跨网段场景建议前置 TLS 反向代理。
- 勿在任何日志、代码仓库、聊天记录中提交真实 token。
- bind 地址参数化为 `0.0.0.0`（LAN 可达）或 `127.0.0.1`（仅本机调试）。

---

## 2. 快速开始

### 2.1 你会拿到什么

接入前应向机器人方索取三样东西：

1. 主机 LAN IP 或主机名（下文以 `<host>` 代替，示例 `192.168.1.132`）；
2. `ARACHNE_MCP_PLANNER_TOKEN` 的实际值（**只传这个给普通 agent**）;
3. （仅当授权执行链路）`ARACHNE_MCP_EXECUTION_TOKEN` 的实际值。

### 2.2 MCP 客户端配置

如果你的客户端支持标准 MCP Streamable HTTP 传输（Claude Desktop、opencode、自定义 MCP 客户端），直接使用/改编下面的配置：

```json
{
  "mcpServers": {
    "arachne-viewpoint-planner": {
      "type": "http",
      "url": "http://192.168.1.132:8123/arachne/viewpoint/plan",
      "authorization": {
        "type": "bearer",
        "scheme": "bearer",
        "credentials": "<ARACHNE_MCP_PLANNER_TOKEN>"
      }
    }
  }
}
```

若同时获得执行权限，追加第二个 server 条目（URL 路径换成 `/arachne/viewpoint/acquire`，token 换成 execution token）。

> 客户端差异提示：不同客户端对 bearer 凭据的注入方式略有差别（header 字段名、credentials 字段格式等），但目标都是发出 HTTP 头 `Authorization: Bearer <token>`。若你的客户端不支持自动注入鉴权头，请改用 §4 的纯 JSON HTTP 端点。

### 2.3 三分钟验证连通性（curl）

```bash
HOST="192.168.1.132:8123"

# 0) 存活探针（最快，不需要 token）——200 + {"status":"ok","service":"arachne_mcp"} 即服务在监听
curl -sS http://$HOST/health
# 期望输出: {"status": "ok", "service": "arachne_mcp"}
# 连不上（connection refused/timeout）= MCP 进程未启动，先联系机器人方开启服务

TOKEN="<你的 planner token>"

# 1) 无 token 应返回 401
curl -sS -o /dev/null -w '%{http_code}\n' \
  -X POST http://$HOST/arachne/viewpoint/plan \
  -H 'Content-Type: application/json' \
  -H 'Accept: application/json, text/event-stream' \
  -d '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"probe","version":"1"}}}'
# 期望输出: 401

# 2) 带 token 初始化握手 → 应返回 serverInfo
curl -sS -X POST http://$HOST/arachne/viewpoint/plan \
  -H "Authorization: Bearer $TOKEN" \
  -H 'Content-Type: application/json' \
  -H 'Accept: application/json, text/event-stream' \
  -d '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"probe","version":"1"}}}'
```

详细 MCP JSON-RPC 会话流程见 §5。

### 2.4 快速跑通一次规划

最短可运行流程（三个调用完成"规划→取可达点"闭环）：

```
① plan_viewpoints(target_xyz=[0.3, 0.0, 0.2])     → 返回 plan_id + 32 个候选点
② list_reachable_viewpoints(plan_id)              → 过滤后的可达点列表（带索引）
③ （如已授权执行）acquire_chain(plan_id, indices=…)  → 逐点移动+采图
```

具体工具语义见 §6，MCP 调用实例见 §5.3，纯 JSON 调用实例见 §7。

---

## 3. 工具面总览

五个 tool 分属两个能力域；此外还有一个**非工具**端点 `GET /health`（免 token 存活探针，见 §2.3），仅报告服务是否在监听，不属于任何能力域：

| 工具 | 能力域 | 需要的 token | 危险级别 | 用途一句话 |
| --- | --- | --- | --- | --- |
| `plan_viewpoints` | planner | PLANNER_TOKEN | read_only | 围绕目标点采样视点并持久化 |
| `list_reachable_viewpoints` | planner | PLANNER_TOKEN | read_only | 读回某次规划的"可达且无碰撞"点 |
| `acquire_chain` | execution | EXECUTION_TOKEN | motion | 驱动机械臂逐个视点移动并采集数据 |
| `get_acquire_status` | execution | EXECUTION_TOKEN | read_only | 轮询采图链进度 |
| `cancel_acquire` | execution | EXECUTION_TOKEN | stop | 安全中断在途采图链 |

推荐调用顺序：`plan → list → acquire → status(轮询)`，异常退出用 `cancel`。

---

## 4. 鉴权与错误语义（两个传输面共用）

所有端点（MCP 路径和 JSON 路径）都要求 HTTP 头：

```
Authorization: Bearer <token>
```

**唯一例外：`GET/HEAD /health`（存活探针）不需要任何 token**——它只返回 `{"status":"ok","service":"arachne_mcp"}`，不携带能力或数据，用于连通性判断；探针被放置在鉴权层之外，不影响其余端点的鉴权。

token 缺失或错误一律 **401**。没有配置 token 的能力域不会监听（服务启动时即拒绝）——不存在匿名裸奔面。

两种传输共用的错误模型：

| 场景 | MCP 面（JSON-RPC error / isError=true content） | JSON 面 |
| --- | --- | --- |
| 无/错 token | HTTP 401 | HTTP 401 |
| 参数缺失/类型错 | tools/call 返回 isError=true 文本说明 | HTTP 400，body `{"ok":false,"error":"..."}` |
| plan_id 不存在/解析失败 | isError=true | HTTP 404 |
| 运动门禁未开（执行面） | acquire_chain 返回 success=false + message | HTTP 403/503 |
| 链路忙（busy 门禁） | acquire_chain 拒绝 | 同上映射 |
| 未知路径/方法错 | — | HTTP 404 / 405 |

---

## 5. MCP 传输细节（JSON-RPC over Streamable HTTP）

如果你自己实现 MCP 客户端而非用现成 SDK，本节给出完整的报文级说明。

### 5.1 端点与会话

- URL 即能力路径本身：`http://<host>:8123/arachne/viewpoint/plan` 与 `/arachne/viewpoint/acquire`（无需再加 `/mcp` 后缀）。
- Content-Type 必须是 `application/json`。
- Accept 建议 `application/json, text/event-stream`（响应可能是普通 JSON，也可能是 SSE 帧）。
- `initialize` 的响应头里会带回 `Mcp-Session-Id`，**后续每个请求都要原样带上这个头**，否则 session 不存在。
- 初始化完成后必须发送通知 `notifications/initialized`（无 id、无期待回复）。

### 5.2 会话建立完整序列

```bash
BASE=http://192.168.1.132:8123/arachne/viewpoint/plan
TOKEN=<planner_token>

SID=$(curl -sS -D - -o /dev/null -X POST "$BASE" \
  -H "Authorization: Bearer $TOKEN" \
  -H 'Content-Type: application/json' \
  -H 'Accept: application/json, text/event-stream' \
  -d '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{
        "protocolVersion":"2024-11-05","capabilities":{},
        "clientInfo":{"name":"my-agent","version":"1.0"}}}' \
  | grep -i '^mcp-session-id:' | cut -d' ' -f2 | tr -d '\r')

# 回执初始化完成通知（带同一 session id）
curl -sS -X POST "$BASE" \
  -H "Authorization: Bearer $TOKEN" -H "Mcp-Session-Id: $SID" \
  -H 'Content-Type: application/json' \
  -H 'Accept: application/json, text/event-stream' \
  -d '{"jsonrpc":"2.0","method":"notifications/initialized"}'
```

Python 参考实现（无第三方依赖）：

```python
import json, urllib.request

class ArachneMcpClient:
    def __init__(self, url: str, token: str):
        self.url = url
        self.headers = {
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
        }
        self.session_id = None

    def rpc(self, payload: dict) -> dict | None:
        h = dict(self.headers)
        if self.session_id:
            h["Mcp-Session-Id"] = self.session_id
        r = urllib.request.urlopen(
            urllib.request.Request(self.url, json.dumps(payload).encode(), h),
            timeout=120)
        self.session_id = r.headers.get("Mcp-Session-Id") or self.session_id
        body = r.read().decode()
        if body.startswith("event:") or "\ndata:" in body:
            for line in body.splitlines():
                if line.startswith("data:"):
                    return json.loads(line[5:])
            return None
        return json.loads(body) if body.strip() else None

    def connect(self) -> dict:
        init = self.rpc({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                         "params": {"protocolVersion": "2024-11-05",
                                    "capabilities": {},
                                    "clientInfo": {"name": "my-agent", "version": "1.0"}}})
        self.rpc({"jsonrpc": "2.0", "method": "notifications/initialized"})
        return init["result"]["serverInfo"]

    def list_tools(self) -> list[str]:
        r = self.rpc({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
        return [t["name"] for t in r["result"]["tools"]]

    def call(self, name: str, arguments: dict, timeout: float = 120.0) -> str:
        r = self.rpc({"jsonrpc": "2.0", "id": 3, "method": "tools/call",
                      "params": {"name": name, "arguments": arguments}})
        res = r["result"]
        if res.get("isError"):
            raise RuntimeError(next(c["text"] for c in res["content"]
                                    if c.get("type") == "text"))
        return next(c["text"] for c in res["content"] if c.get("type") == "text")

c = ArachneMcpClient("http://192.168.1.132:8123/arachne/viewpoint/plan",
                     "<PLANNER_TOKEN>")
print(c.connect())
print(c.list_tools())
```

### 5.3 tools/call 实例：完整规划往返

```python
import math, json

# ---- 规划 ----
raw = c.call("plan_viewpoints",
             {"target_xyz": [0.45, 0.0, 0.30], "num_viewpoints": 32})
plan = json.loads(raw)
print(plan["plan_id"])          # e.g. "20260827_145411_063"
print(plan["server_message"])   # "planned 32 viewpoints (...); ik_reachable=9/32, ..."

for v in plan["viewpoints"]:
    print(v["camera_xyz_base"], v["ik_reachable"], v["collision_free"])

# ---- 取回可达点 ----
reach_raw = c.call("list_reachable_viewpoints", {"plan_id": plan["plan_id"]})
reach = json.loads(reach_raw)
for rv in reach["reachable_viewpoints"]:
    print(rv["index"],
          [round(x, 3) for x in rv["camera_xyz_base"]],
          rv["ik_joints_deg"],
          rv["collision_clearance_m"])
```

实测参考输出（2026-08-27 端到端记录，plan_id 为真实值）：

```
20260827_145411_063
planned 32 viewpoints (strategy=fibonacci_sphere, radius=0.5, count=32,
filter_ik=True, filter_collision=True); ik_reachable=9/32,
collision_free=5/32, reachable=5/32
0   [0.574, 0.0,    0.784] joints_deg=[-68.9,-4.8,24.0,-74.5,95.1,65.5]   clearance=0.024
1   [0.294, 0.143,  0.753] joints_deg=[22.8,19.9,45.1,-87.8,79.9,22.4]    clearance=0.024
2   [0.473, -0.267, 0.722] joints_deg=[22.6,52.6,30.6,-80.8,80.7,-120.0]  clearance=0.024
7   [0.255, -0.376, 0.566] joints_deg=[-16.5,88.2,73.8,-46.9,80.8,-129.2] clearance=0.024
11  [0.594, 0.458,  0.441] joints_deg=[134.7,87.7,28.6,-77.4,116.6,-143.4] clearance=0.004
```

坐标约定见 §6.1。

---

## 6. 工具详解

### 6.1 坐标系与单位约定（务必先读）

- 所有 XYZ 都是**机器人底盘坐标系 `base_link`**，单位米：x 前、y 左、z 上。
- 四元数为 xyzw 顺序。
- `ik_joints_deg` 是 Aubo i5 六轴关节角（度）；`ik_joints`（rad）仅在 plan_viewpoints 全量视图里出现。
- 目标点放在工作台前方典型高度（如 `[0.45, 0, 0.30]`），采样球默认半径 0.5 m。
- `collision_clearance_m` 数值越小越贴边：< 10 mm 的点慎用于执行。

### 6.2 plan_viewpoints（planner · read_only）

围绕目标位置做球面视点采样，逐点做 IK 可达性与碰撞过滤，结果落盘共享目录。

入参：

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `target_xyz` | number[3] | **必填** | base_link 系目标位置 [x,y,z] 米 |
| `strategy` | string | `"fibonacci_sphere"` | 采样策略名 |
| `radius_m` | number | 0.5 | 采样球半径（米）|
| `num_viewpoints` | int | 32 | 采样数（1~128）|
| `filter_ik_reachable` | bool | true | 只保留 IK 可达点 |
| `filter_collision_free` | bool | true | 只保留无碰撞点 |
| `up_vector` | number[3]? | null | 采样球参考轴（默认 z 向上）|

出参关键字段：

```jsonc
{
  "plan_id": "20260827_145411_063",           // 之后所有引用都用它
  "plan_file": "/home/jetson/zhaoyang/Arachne/log/viewpoint_plans/viewpoint_plan_20260827_145411_063.json",
  "num_viewpoints": 32,
  "ik_reachable_count": 9,
  "collision_free_count": 5,
  "reachable_count": 5,
  "server_message": "...",
  "viewpoints": [
    { "camera_xyz_base": [...], "tool0_xyz_base": [...],
      "camera_quat_xyzw": [...], "tool0_quat_xyzw": [...],
      "ik_reachable": true, "ik_joints": [...],
      "collision_free": true, "collision_clearance": 0.013 }
  ]
}
```

行为要点：

- 结果按毫秒时间戳命名为共享 plan 文件；plan 目录只保留最近 20 份（惰性清理）。
- 该调用 **无副作用除落盘外**，不触碰任何硬件状态。
- 规划节点未在线时报错："viewpoint plan service call failed: ROS service ... not available (start the viewpoint planner first)" —— 找机器人方确认面板上 Viewpoint Planner 已启动。

### 6.3 list_reachable_viewpoints（planner · read_only）

从共享目录读取某次规划，返回过滤后的可达点，供 acquire_chain 使用。

入参：`plan_id`（string，必填）。

出参：

```jsonc
{
  "plan_id": "…", "plan_file": "/abs/path.json",
  "total_viewpoints": 32, "reachable_count": 5,
  "reachable_viewpoints": [
    { "index": 0,                              // ← 执行时的选择键
      "tool0_xyz_base": [...], "camera_xyz_base": [...],
      "ik_joints_deg": [...],                  // 度
      "collision_clearance_m": 0.024 }         // 米
  ]
}
```

注意：这里的 `index` 是**过滤后列表中的序号**，不是原始 32 点里的编号；acquire_chain 的 indices 参数就用这套索引。

### 6.4 acquire_chain（execution · motion）

驱动机械臂按指定索引逐点走（MoveIt 碰撞检查规划 + SDK moveJoint 主路径），每点采 RGB/depth/点云，并落盘。

**前提条件（重要）**：机器人侧 launch 显式同时打开
`mcp_motion_enabled:=true` 和 `mcp_confirm_agent_motion:=true`。
缺任一开关本工具一律返回 success=false 并附原因，任何情况下都不会有动作发生。

入参：

| 参数 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `plan_id` | string | **必填** | plan_viewpoints 返回的 ID |
| `indices` | `"all"` \| int[] | `"all"` | 从 filtered 列表里挑哪些点执行 |

出参（受理成功立即返回，之后轮询 get_acquire_status）：

```jsonc
{
  "success": true,
  "run_id": "…",             // 用于关联 status 输出
  "run_dir": "/abs/path/viewpoint_acquisition/<run_id>/",
  "num_viewpoints": 5,
  "note": "poll get_acquire_status until busy=false"
}
```

拒绝情况都会带 `success:false` + `message`：gate 未开、链路 busy、plan 不存在、超出单次上限（默认 32 点）、indices 解析失败。

### 6.5 get_acquire_status（execution · read_only）

轮询当前/上一轮采图链。**每次 acquire_chain 后都应当轮询直到 busy=false**。

出参示例：

```jsonc
{
  "busy": false,                 // 还在跑吗？
  "completed": 3,                // 已完成点数
  "current_index": null,         // 正在处理的 index（null 表示没在跑）
  "points": [
    { "index": 0, "ok": true, "path": "/abs/…/<run_id>/0/",
      "errors": null, "cloud_point_count": 12345 },
    { "index": 1, "ok": false, "path": null,
      "errors": ["move failed"], "cloud_point_count": null }
  ],
  "cancelled": false,
  "error": null,                 // worker 崩溃才非空
  "started_stamp": "2026-08-27T14:55:00",
  "finished_stamp": "2026-08-27T14:57:31"
}
```

### 6.6 cancel_acquire（execution · stop）

随时可调的安全停止：主动打断在途运动（SDK stopJoint + 停速度流）并收尾 manifest。链未在跑时返回 success=false + 说明（no-op，无副作用）。取消后 `get_acquire_status` 里 `cancelled=true`、已完成点的 results 仍然齐全。

---

## 7. 纯 JSON HTTP 传输面（给不支持 MCP 协议的 agent）

同样的五个能力还以最朴素的 POST/GET JSON 端点暴露，语义与 MCP 一致。适合只有 http client 的简单 agent。

| 方法 | 路径 | 对应工具 | token |
| --- | --- | --- | --- |
| POST | `/arachne/json/viewpoint/plan/plan_viewpoints` | plan_viewpoints | PLANNER |
| POST | `/arachne/json/viewpoint/plan/list_reachable_viewpoints` | list_reachable_viewpoints | PLANNER |
| POST | `/arachne/json/viewpoint/acquire/acquire_chain` | acquire_chain | EXECUTION |
| GET | `/arachne/json/viewpoint/acquire/status` | get_acquire_status | EXECUTION |
| POST | `/arachne/json/viewpoint/acquire/cancel` | cancel_acquire | EXECUTION |

请求体 = 工具入参对象。响应统一包为 `{"ok": true, "result": <工具输出>}` 或 `{"ok": false, "error": "<原因>"}`。

### 7.1 规划调用示例

```bash
curl -sS -X POST http://192.168.1.132:8123/arachne/json/viewpoint/plan/plan_viewpoints \
  -H "Authorization: Bearer $ARACHNE_MCP_PLANNER_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"target_xyz":[0.3,0.0,0.2],"num_viewpoints":32}'
```

```bash
curl -sS -X POST http://192.168.1.132:8123/arachne/json/viewpoint/plan/list_reachable_viewpoints \
  -H "Authorization: Bearer $ARACHNE_MCP_PLANNER_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"plan_id":"20260827_145411_063"}'
```

### 7.2 执行链示例（需 EXECUTION token 且门禁开启）

```bash
# 发起采图链
curl -sS -X POST http://192.168.1.132:8123/arachne/json/viewpoint/acquire/acquire_chain \
  -H "Authorization: Bearer $ARACHNE_MCP_EXECUTION_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"plan_id":"20260827_145411_063","indices":[0,1,2]}'

# 轮询进度
curl -sS http://192.168.1.132:8123/arachne/json/viewpoint/acquire/status \
  -H "Authorization: Bearer $ARACHNE_MCP_EXECUTION_TOKEN"

# 中途取消
curl -sS -X POST http://192.168.1.132:8123/arachne/json/viewpoint/acquire/cancel \
  -H "Authorization: Bearer $ARACHNE_MCP_EXECUTION_TOKEN"
```

HTTP 状态码语义：200 成功、400 参数错、401 鉴权失败、403/503 运动拒绝、404 路径/plan_id 错、405 方法错。

---

## 8. 推荐 Agent 流程与工程实践

### 8.1 标准流程

```
① plan_viewpoints(target_xyz=…)                 → plan_id
② list_reachable_viewpoints(plan_id)            → inspect reachable_viewpoints[].index
③ 若获授权：acquire_chain(plan_id, indices=[…])  → run_id
④ 循环 get_acquire_status() 直到 busy=false       → 逐点 ok/path 结果
⑤ 异常/提前结束：cancel_acquire()
```

### 8.2 给 agent 编写者的实操建议

1. **别搬运关节角**。选点永远通过 indices 引用，agent 自己算关节解既危险又不必要。
2. **先小规模验证再全量执行**。第一次真机采图建议 `indices:[0]` 单点试跑确认行为后再扩大范围。
3. **尊重碰撞间隙**。把 `collision_clearance_m < 0.01` 的点从默认执行集中剔除，除非用户显式点名要它。
4. **对长时间任务轮询节流**。每 2–5 秒问一次 status 足够；别打满 rate limit。
5. **把 run_dir 绝对路径透传给用户**。manifest.json 和 per-point 子目录都在里面，人工检索靠它。
6. **拿不到服务时区分报错**。"ROS service not available" 是机器人侧服务未启动（找运维），"HTTP 401" 是 token 问题（找管理员），不要笼统重试。
7. **不要并发发起多个 acquire_chain**。服务端有 busy 门禁，并发只会收到拒绝。

### 8.3 故障排查速查

| 现象 | 可能原因 | 怎么办 |
| --- | --- | --- |
| 连接拒绝 / timeout | 服务没起 | 先 `curl http://<host>:8123/health` 定位：连不上 = MCP 进程未启动，联系机器人方在 GUI 点 MCP Servers 按钮；200 = 交换机/防火墙问题 |
| HTTP 401 | token 缺失或不对 | 核对 Authorization 头拼写与 token 值 |
| tools/call 报 "ROS service … not available" | viewpoint_planner 未启动 | 机器人方确认 Viewpoint Planner 服务在跑 |
| acquire_chain 一直 rejected, message 提到 gate | 运动门禁未开 | 属预期安全行为；找操作员评估是否启用 |
| status 里 points[i].ok=false | 该点 MoveIt 规划失败/keepout 拦截 | 查看 message/errors 字段；剔除该 index 重试其余点 |
| SSE 响应解析不到 JSON | 用了解析器只支持 plain JSON | 按 data: 行提取 payload；或改走 §7 JSON 面 |

### 8.4 完整案例

以下三个案例覆盖"只读规划"与"授权执行"两类典型接入场景。所有代码基于 §5.2 的 `ArachneMcpClient`（零第三方依赖），可直接复制后填入 token 运行。

#### 案例 A：桌面物体视点规划（纯只读，最低权限接入）

**场景**：上游 agent 只拿到 PLANNER token，需要为一杯放在桌面上的目标物规划拍摄视角。

```python
import json, math

c = ArachneMcpClient("http://192.168.1.132:8123/arachne/viewpoint/plan",
                     "<ARACHNE_MCP_PLANNER_TOKEN>")
c.connect()

# ① 规划：目标物估计在底盘前方 45cm、高 30cm 处。
#    用较小半径(0.35m)拉近相机距离，48 个候选提高选中概率。
plan = json.loads(c.call("plan_viewpoints", {
    "target_xyz": [0.45, 0.0, 0.30],
    "radius_m": 0.35,
    "num_viewpoints": 48,
    "filter_ik_reachable": True,
    "filter_collision_free": True,
}))
print(f"[A] {plan['server_message']}")
print(f"[A] plan_id={plan['plan_id']}, plan_file={plan['plan_file']}")

# ② 只挑"可达 && 无碰撞"的点，并按间隙排序（最安全的排前面）
reach = json.loads(c.call("list_reachable_viewpoints",
                          {"plan_id": plan["plan_id"]}))
ranked = sorted(reach["reachable_viewpoints"],
                key=lambda v: -v["collision_clearance_m"])
print(f"[A] {reach['reachable_count']} usable viewpoints:")
for rv in ranked:
    print(f"    idx={rv['index']:2d} cam={[round(x,3) for x in rv['camera_xyz_base']]} "
          f"clearance={rv['collision_clearance_m']*1000:.0f}mm")

# ③ 把结果交给上层决策（此处仅演示：推荐间隙最大的点）
if ranked:
    best = ranked[0]
    print(f"[A] recommend index {best['index']} "
          f"(tool0 at {[round(x,3) for x in best['tool0_xyz_base']]})")
```

实测风格输出：

```
[A] planned 48 viewpoints (...); ik_reachable=13/48, collision_free=7/48, reachable=7/48
[A] plan_id=20260827_150210_115, plan_file=/home/jetson/zhaoyang/Arachne/log/viewpoint_plans/viewpoint_plan_20260827_150210_115.json
[A] 7 usable viewpoints:
    idx= 3 cam=[0.575, 0.0, 0.699] clearance=24mm
    idx=11 cam=[0.302, 0.162, 0.662] clearance=24mm
    ...
[A] recommend index 3 (tool0 at [0.53, 0.0, 0.77])
```

注意：agent 全程不接触任何关节角——`ik_joints_deg` 只用于展示/审计，选点靠 `index`。

#### 案例 B：单点试跑 + 轮询采图链（已授权执行）

**场景**：操作员已在机器人侧显式打开运动门禁，agent 拿到 EXECUTION token，按工程实践「先单点验证再放量」执行。

```python
import json, time

plan_id = "<从案例A或自己规划得到>"

ec = ArachneMcpClient("http://192.168.1.132:8123/arachne/viewpoint/acquire",
                      "<ARACHNE_MCP_EXECUTION_TOKEN>")
ec.connect()

# ① 先剔除贴边点（间隙<10mm 不进默认执行集——见 §8.2 第3条）
pc = ArachneMcpClient("http://192.168.1.132:8123/arachne/viewpoint/plan",
                      "<ARACHNE_MCP_PLANNER_TOKEN>")
pc.connect()
reach = json.loads(pc.call("list_reachable_viewpoints", {"plan_id": plan_id}))
safe = [v for v in reach["reachable_viewpoints"]
        if v["collision_clearance_m"] >= 0.010]
picks = [v["index"] for v in safe]
print(f"[B] candidates {picks}")

# ② 单点试跑：先打最前面的一个点
trial = json.loads(ec.call("acquire_chain",
                           {"plan_id": plan_id, "indices": picks[:1]}))
assert trial["success"], trial.get("message")   # gate 未开会在这里拒绝
run_id = trial["run_id"]
print(f"[B] trial accepted: run_id={run_id}, run_dir={trial['run_dir']}")

# ③ 轮询直到完成（每 3 秒，尊重 §8.2 第4条节流建议）
while True:
    st = json.loads(ec.call("get_acquire_status", {}))
    if not st["busy"]:
        break
    done = st["completed"]
    cur = st["current_index"]
    print(f"[B] progress {done}/{len(picks[:1])}"
          + (f" (working on #{cur})" if cur is not None else ""))
    time.sleep(3)

# ④ 核对结果；失败点的处理策略见 §8.3
for p in st["points"]:
    mark = "OK " if p["ok"] else "FAIL"
    extra = f" cloud_pts={p['cloud_point_count']}" if p["ok"] else f" err={p['errors']}"
    print(f"[B] {mark} idx={p['index']}{extra} path={p['path']}")
if st.get("cancelled"):
    print("[B] (chain was cancelled)")

# ⑤ 试跑通过后放量执行剩余点
rest = picks[1:]
full = json.loads(ec.call("acquire_chain",
                          {"plan_id": plan_id, "indices": rest}))
print(f"[B] batch accepted: {full['success']}, n={full.get('num_viewpoints')}")
# ……同样方式轮询 full 这轮即可（代码同③④）
```

典型拒绝信息示例（**门禁关闭时的预期行为**，不是故障）：

```
RuntimeError: acquire_chain rejected: motion_enabled:=false (execution MCP motion is
disabled by default; enable with motion_enabled:=true and confirm_agent_motion:=true)
```

#### 案例 C：长时间任务中途安全取消（异常路径）

**场景**：批量执行中发现目标物被移走 / 用户叫停 / 点位连续失败，agent 主动终止链路。

```python
# 接案例 B 的 ec 会话，某轮批量正在执行……
st = json.loads(ec.call("get_acquire_status", {}))
failed_so_far = [p for p in st["points"] if not p["ok"]]

if len(failed_so_far) >= 2:          # 策略示例：连续失败过多就停
    r = json.loads(ec.call("cancel_acquire", {}))
    print(f"[C] cancel success={r['success']} "
          f"completed_before_cancel={r['points_completed']}\n"
          f"[C] message: {r['message']}")
    # 取消是幂等安全的：链没在跑时只会返回 success=false + 说明

    # 终态核对：busy=false 且 cancelled=true，已完成点的数据仍然有效可取
    while json.loads(ec.call("get_acquire_status", {}))["busy"]:
        time.sleep(1)
    final = json.loads(ec.call("get_acquire_status", {}))
    assert final["cancelled"] and not final["busy"]
    print(f"[C] final state: cancelled chain; "
          f"{len([p for p in final['points'] if p['ok']])} points had good data")
    # 已落盘的 manifest.json / events.jsonl / 已采集数据都在各自 path 下，
    # 上层可继续利用部分成功的数据。
```

关键点：**取消不会销毁已完成点的数据**；`events.jsonl` 会记录 cancel 事件供事后审计（§9）。

#### 案例对照速查

| | 案例 A | 案例 B | 案例 C |
| --- | --- | --- | --- |
| 所需 token | planner | planner + execution | execution |
| 是否触碰机械臂 | 否 | 是（门禁开启） | 中断已有运动 |
| 覆盖工具 | plan / list | plan / list / acquire_chain / status | status / cancel |
| 典型用途 | 目标可视化点位查询 | 小心推进的正式采集 | 异常兜底 |

---

## 9. 安全边界汇总（agent 作者必须知晓）

- **role-based 权限由 token 决定**：planner token 永远碰不到运动路径；execution token 在门禁关闭时也一事无成。
- **运动主路径内置三层保护**：MoveIt 碰撞检查规划（失败拒动）→ keepout 实时拦截 → control-owner/teach-gate 互斥（GUI 操作员手动干预期间 agent 运动被拒）。
- **每次实机采图都有审计痕迹**：events.jsonl 与 manifest.json 记录了逐点时间戳、成败、绝对路径，便于事后追查。
- **禁止事项**：
  - 不要尝试绕过 indices 直接构造关节角指令（工具面没有这种入口，也不要找非公开路径）。
  - 不要在同一会话中混用两个 token（能力域分离，混淆会导致诊断困难）。
  - 不要将 token 写入公开代码库、日志或演示文稿。
- 如需为其他机器人能力（grasp/cleanup/nav 等）新增暴露面，联系维护者另行立项，不要试图猜测 URL。

---

## 10. 附录

### 10.1 服务器版本信息

- `initialize` 响应中的 serverInfo：`{"name": "viewpoint-planner", "version": "1.29.1"}`（FastMCP/官方 mcp SDK 实现，协议版本 `2024-11-05` 兼容）。

### 10.2 关键默认值速记

| 项 | 默认值 |
| --- | --- |
| 端口 | 8123 |
| bind_host | 0.0.0.0（LAN 可达） |
| 健康探针 | `GET/HEAD /health`（免 token；返回 `{"status":"ok","service":"arachne_mcp"}`） |
| 共享 plan 目录保留份数 | 20 |
| 单次 acquire_chain 最大点数 | 32 |
| 规划服务超时 | 60 s |
| 运动门禁 | **关闭**（需双开关显式开启） |
| MCP path 前缀 | /arachne/viewpoint/{plan,acquire} |
| JSON path 前缀 | /arachne/json/viewpoint/{plan,acquire}/… |

### 10.3 相关仓库文档

- `docs/mcp/mcp_servers.example.json` —— 可直接导入的 MCP 客户端配置模板
- `docs/mcp/arachne_integration.json` —— 全量接口 JSON 描述（含 inputSchema）
- `docs/todo/mcp_exposure_todo_plan.md` —— 设计、实施与验收历史（内部读者）
