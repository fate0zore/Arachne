"""Arachne MCP exposure layer: ONE port, two readable paths, two capabilities.

Single process hosting both MCP server instances (`FactMCP`) behind one
Streamable HTTP listener:

    http://<host>:8123/arachne/viewpoint/plan     -> read-only planner tools
    http://<host>:8123/arachne/viewpoint/acquire  -> motion-gated acquire tools

A token-free liveness probe answers "is the MCP up?" with a plain GET (no
bearer token, no MCP handshake — it carries no capability or data):

    http://<host>:8123/health   -> {"status":"ok","service":"arachne_mcp"}

Each path keeps its own bearer token (hence its own privilege boundary):
the planner token can be handed to any agent, the execution token must be
kept restricted. The *capabilities* stay fully separated (two FastMCP
instances, two tool sets, two ROS-facing roles) — only the transport port is
shared, per the mcp_exposure_todo_plan.md redesign decision.

The same capabilities are also exposed as plain-JSON HTTP endpoints (see
``json_api.py``) under the same per-capability bearer tokens, so a non-MCP
agent can drive them with nothing more than an HTTP client:

    http://<host>:8123/arachne/json/viewpoint/plan/plan_viewpoints
    http://<host>:8123/arachne/json/viewpoint/plan/list_reachable_viewpoints
    http://<host>:8123/arachne/json/viewpoint/acquire/acquire_chain
    http://<host>:8123/arachne/json/viewpoint/acquire/status
    http://<host>:8123/arachne/json/viewpoint/acquire/cancel

The execution host node (`ExecutionMcpNode`, same parameter surface as the
old standalone entrypoint) owns the process: it is the ROS parameter host
(launch passes every parameter to node `arachne_execution_mcp`) and the
planner capability binds to it via `ViewpointPlannerApi`.
"""

# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# ROS packages and user-site MCP SDK are outside the workspace LSP's scan
# path; imports resolve at runtime (verified via py_compile + smoke tests).
from __future__ import annotations

# The MCP SDK stack (mcp/starlette/uvicorn) lives in the user site-packages
# (~/.local), which the workspace PYTHONPATH shadows; make it importable
# before pulling in arachne_mcp.common (which imports starlette at module
# level).
import os
import site as _site
import sys as _sys

_user_site = os.path.expanduser("~/.local/lib/python3.10/site-packages")
if os.path.isdir(_user_site) and _user_site not in _sys.path:
    _site.addsitedir(_user_site)

import argparse
import threading
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor

from arachne_mcp.common import (
    ALLOWED_HOSTS_ENV,
    HEALTH_PATH,
    assert_tokens_configured,
    build_fastmcp,
    log_requests,
    parse_allowed_hosts,
    require_token_by_path,
    sanitize_ros_args,
    serve_http_background,
    to_int,
    wait_http_ready,
    with_health_check,
)
from arachne_mcp.execution_mcp import (
    ExecutionMcpNode,
    _register_tools as _register_execution_tools,
)
from arachne_mcp.json_api import (
    ACQUIRE_JSON_PREFIX,
    JsonApi,
    PLAN_JSON_PREFIX,
)
from arachne_mcp.viewpoint_planner_mcp import (
    create_planner_api,
    _register_tools as _register_planner_tools,
)

# Single transport port; paths are the externally visible endpoints.
DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_PORT = 8123
PLAN_PATH = "/arachne/viewpoint/plan"
ACQUIRE_PATH = "/arachne/viewpoint/acquire"

# JSON overlay path prefixes (see json_api.py). They carry the same
# per-capability bearer token as their MCP peers, so the privilege boundary
# (planner read-only vs execution motion) is preserved on the JSON surface.
PLAN_JSON_PATH = PLAN_JSON_PREFIX
ACQUIRE_JSON_PATH = ACQUIRE_JSON_PREFIX

# Planner-side params are declared by TeachPanelNode (host base class); these
# are the aggregate listen/token params the launch file sets on the same node.
AGGREGATE_PARAMS = ("mcp_bind_host", "mcp_port", "mcp_planner_token", "mcp_execution_token", "mcp_allowed_hosts")


class McpAggregateNode(ExecutionMcpNode):
    """Execution host + parameter host for the aggregated MCP process.

    Inherits :class:`ExecutionMcpNode` (TeachPanelNode core, motion gate,
    acquire chain) so the execution MCP tools bind directly to this node,
    exactly like the old standalone entrypoint. On top it declares the
    planner-side params and the aggregate listen/token params so the launch
    file can set them on this node by name.
    """

    def __init__(self) -> None:
        super().__init__()
        # Aggregate listen + token params (single port, per-path tokens).
        # Planner-side params (viewpoint_plan_service / timeout / share dir /
        # keep) are already declared by TeachPanelNode, which the host
        # inherits — no re-declaration needed.
        self.declare_parameter("mcp_bind_host", os.environ.get("ARACHNE_MCP_BIND_HOST", DEFAULT_BIND_HOST))
        self.declare_parameter("mcp_port", to_int(os.environ.get("ARACHNE_MCP_PORT", DEFAULT_PORT), "ARACHNE_MCP_PORT"))
        self.declare_parameter("mcp_planner_token", os.environ.get("ARACHNE_MCP_PLANNER_TOKEN", ""))
        self.declare_parameter("mcp_execution_token", os.environ.get("ARACHNE_MCP_EXECUTION_TOKEN", ""))
        self.declare_parameter("mcp_allowed_hosts", os.environ.get(ALLOWED_HOSTS_ENV, ""))


def _build_app(host: Any, planner_token: str, execution_token: str, allowed_hosts: list[str]) -> Any:
    """Build the single-port ASGI app dispatching the MCP + JSON endpoints.

    Starlette ``Mount`` is deliberately NOT used: on exact matches it emits
    307 -> trailing-slash redirects, which MCP clients dislike. Instead a
    tiny ASGI dispatcher routes by path prefix and rewrites the child scope
    path to "/" (matching the SDK's ``streamable_http_path="/"`` route),
    while the outer bearer-token middleware keeps the original path. The
    plain-JSON overlay (``JsonApi``) is tried first on exact JSON paths and
    never rewrites the scope path.
    """
    from mcp.server.fastmcp import FastMCP

    # Planner capability: bound to the host node (no separate ROS node).
    planner_api = create_planner_api(host)
    planner_mcp = build_fastmcp(
        "viewpoint-planner", allowed_hosts, log_level="ERROR", streamable_http_path="/"
    )
    _register_planner_tools(planner_mcp, planner_api)

    # Execution capability: the host node itself (TeachPanelNode core).
    execution_mcp = build_fastmcp(
        "viewpoint-acquire", allowed_hosts, log_level="ERROR", streamable_http_path="/"
    )
    _register_execution_tools(execution_mcp, host)

    planner_http = planner_mcp.streamable_http_app()
    execution_http = execution_mcp.streamable_http_app()
    # Keep both FastMCP session managers alive for the whole serving lifetime
    # (see serve_http_background).
    managers = (planner_mcp._session_manager, execution_mcp._session_manager)

    # Plain-JSON overlay bound to the same planner API + execution node.
    json_api = JsonApi(planner_api, host)

    async def _not_found(scope, receive, send) -> None:
        body = b"not found"
        await send(
            {
                "type": "http.response.start",
                "status": 404,
                "headers": [
                    (b"content-type", b"text/plain; charset=utf-8"),
                    (b"content-length", str(len(body)).encode()),
                ],
            }
        )
        await send({"type": "http.response.body", "body": body})

    async def _dispatch(scope, receive, send) -> None:
        if scope.get("type") != "http":
            await _not_found(scope, receive, send)
            return
        # Plain-JSON overlay first (exact-path match; consumes the body itself).
        if await json_api.route(scope, receive, send):
            return
        path = scope.get("path", "")
        target = None
        if path == PLAN_PATH or path.startswith(PLAN_PATH + "/"):
            target = planner_http
        elif path == ACQUIRE_PATH or path.startswith(ACQUIRE_PATH + "/"):
            target = execution_http
        if target is None:
            await _not_found(scope, receive, send)
            return
        child = dict(scope)
        child["path"] = "/"
        child["root_path"] = scope.get("root_path", "") + (
            PLAN_PATH if target is planner_http else ACQUIRE_PATH
        )
        await target(child, receive, send)

    # log_requests outermost so 401s and dispatch failures are logged too;
    # /health sits outside the bearer guard (liveness probe, no data).
    return (
        log_requests(
            with_health_check(
                require_token_by_path(
                    _dispatch,
                    {
                        PLAN_PATH: planner_token,
                        ACQUIRE_PATH: execution_token,
                        PLAN_JSON_PATH: planner_token,
                        ACQUIRE_JSON_PATH: execution_token,
                    },
                ),
                "arachne_mcp",
            ),
            host.get_logger().info,
        ),
        managers,
    )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Arachne MCP aggregate server (one port, two paths + JSON overlay)"
    )
    parser.add_argument(
        "--dry-run-check",
        action="store_true",
        help="validate imports and print the tool manifest without starting ROS/HTTP",
    )
    args, ros_args = parser.parse_known_args(argv)

    if args.dry_run_check:
        from mcp.server.fastmcp import FastMCP

        planner_mcp = FastMCP("viewpoint-planner", log_level="ERROR")
        _register_planner_tools(planner_mcp, None)
        execution_mcp = FastMCP("viewpoint-acquire", log_level="ERROR")
        _register_execution_tools(execution_mcp, None)
        planner_tools = [t.name for t in planner_mcp._tool_manager.list_tools()]
        execution_tools = [t.name for t in execution_mcp._tool_manager.list_tools()]
        from arachne_mcp.json_api import _ENDPOINTS

        json_tools = ", ".join(sorted(handler for _, handler in _ENDPOINTS.values()))
        print(
            f"mcp_servers dry-run check ok; planner tools: {', '.join(planner_tools)}; "
            f"execution tools: {', '.join(execution_tools)}; "
            f"json endpoints: {json_tools}; "
            f"health: GET {HEALTH_PATH} (token-free liveness probe)"
        )
        return

    rclpy.init(args=sanitize_ros_args(ros_args))
    host = McpAggregateNode()

    bind_host = str(host.get_parameter("mcp_bind_host").value)
    port = to_int(host.get_parameter("mcp_port").value, "mcp_port")
    planner_token = str(host.get_parameter("mcp_planner_token").value)
    execution_token = str(host.get_parameter("mcp_execution_token").value)
    allowed_hosts = parse_allowed_hosts(host.get_parameter("mcp_allowed_hosts").value)
    assert_tokens_configured(
        {
            PLAN_PATH: planner_token,
            ACQUIRE_PATH: execution_token,
            PLAN_JSON_PATH: planner_token,
            ACQUIRE_JSON_PATH: execution_token,
        },
        {
            PLAN_PATH: "viewpoint-planning MCP",
            ACQUIRE_PATH: "viewpoint-acquire MCP",
            PLAN_JSON_PATH: "viewpoint-planning JSON API",
            ACQUIRE_JSON_PATH: "viewpoint-acquire JSON API",
        },
    )
    app, managers = _build_app(host, planner_token, execution_token, allowed_hosts)
    serve_http_background(app, bind_host, port, managers=managers)
    wait_http_ready(bind_host, port)
    host.get_logger().info(
        f"Arachne MCP listening on http://{bind_host}:{port} "
        f"(MCP: {PLAN_PATH}, {ACQUIRE_PATH}; JSON: {PLAN_JSON_PATH}/..., "
        f"{ACQUIRE_JSON_PATH}/...; all bearer-token gated; "
        f"host allowlist: {'on: ' + ','.join(allowed_hosts) if allowed_hosts else 'off (LAN open)'}; "
        f"motion gate: motion_enabled="
        f"{host.get_parameter('motion_enabled').value}, "
        f"confirm_agent_motion={host.get_parameter('confirm_agent_motion').value})"
    )

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(host)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        host.cancel_acquire()
    finally:
        executor.shutdown()
        host.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
