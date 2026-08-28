"""Read-only viewpoint-planning MCP server (Streamable HTTP).

Module scope: the *planning* capability only — call ``/arachne/viewpoint/plan``,
persist the result to the shared plan directory, and read plans back by id.

Two exposure modes:

1. Standalone (debug/dry-run): ``viewpoint_planner_mcp`` entrypoint runs its
   own ROS node + HTTP listener on its own port.
2. Aggregated (normal operation): the ``mcp_servers`` entrypoint hosts this
   module alongside the execution module behind ONE port with two readable
   paths (``/arachne/viewpoint/plan`` and ``/arachne/viewpoint/acquire``).
   In that mode the planner logic runs as :class:`ViewpointPlannerApi`
   attached to the host (execution) node — see ``create_planner_api``.

This module never moves anything: no motion path, no gates. It shares the
plan directory (``viewpoint_plan_share_dir``) with the execution module so
``acquire_chain(plan_id, ...)`` can read the same table by id.
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
import json
import threading
from typing import Any

import rclpy
from arachne_operator.repo_paths import root_dir
from arachne_operator.teach_panel import (
    _parse_viewpoint_plan_file,
    _serialize_viewpoint_plan,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arachne_mcp.common import (
    ALLOWED_HOSTS_ENV,
    assert_tokens_configured,
    build_fastmcp,
    call_service_sync,
    log_requests,
    parse_allowed_hosts,
    plan_file_path,
    require_token_by_path,
    resolve_plan_share_dir,
    sanitize_ros_args,
    serve_http_background,
    to_float,
    to_int,
    wait_http_ready,
    with_health_check,
    write_plan_file,
)
from viewpoint_planner_msgs.srv import PlanViewpoints

DEFAULT_PLANNER_PORT = 8123
MAX_VIEWPOINT_COUNT = 128


def create_planner_api(
    node: Node,
    *,
    service_name: str | None = None,
    service_timeout_sec: float | None = None,
    share_dir: str | None = None,
    keep: int | None = None,
) -> "ViewpointPlannerApi":
    """Build the planner API bound to *node* (the execution host node in the
    aggregated mode). Parameters come from the host node; callers must have
    declared them there (see ``declare_planner_params`` on the host)."""
    return ViewpointPlannerApi(
        node=node,
        service_name=service_name
        or str(node.get_parameter("viewpoint_plan_service").value),
        service_timeout_sec=service_timeout_sec
        if service_timeout_sec is not None
        else to_float(
            node.get_parameter("viewpoint_plan_service_timeout_sec").value,
            "viewpoint_plan_service_timeout_sec",
        ),
        share_dir=resolve_plan_share_dir(
            root_dir(),
            share_dir
            or str(node.get_parameter("viewpoint_plan_share_dir").value),
        ),
        keep=keep
        if keep is not None
        else to_int(node.get_parameter("viewpoint_plan_share_keep").value, "viewpoint_plan_share_keep"),
    )


class ViewpointPlannerApi:
    """Planning capability bound to a ROS node (host-owned service client).

    Pure module logic: no HTTP, no node lifecycle. Used by both the
    standalone entrypoint (backed by :class:`ViewpointPlannerMcp`) and the
    aggregated ``mcp_servers`` entrypoint (backed by the execution host node).
    """

    def __init__(
        self,
        node: Node,
        *,
        service_name: str,
        service_timeout_sec: float,
        share_dir: Any,
        keep: int,
    ) -> None:
        self._node = node
        self._service_timeout_sec = service_timeout_sec
        self.share_dir = share_dir
        self._keep = keep
        self.plan_client = node.create_client(PlanViewpoints, service_name)
        node.get_logger().info(f"shared viewpoint plan dir: {self.share_dir}")

    def plan_viewpoints(
        self,
        target_xyz: list[float],
        strategy: str = "fibonacci_sphere",
        radius_m: float = 0.5,
        num_viewpoints: int = 32,
        filter_ik_reachable: bool = True,
        filter_collision_free: bool = True,
        up_vector: list[float] | None = None,
    ) -> str:
        """Call the viewpoint planner service, persist the result, return plan_id."""
        target = self._validated_xyz(target_xyz, "target_xyz")
        up = (
            self._validated_xyz(up_vector, "up_vector")
            if up_vector is not None
            else [0.0, 0.0, 1.0]
        )
        count = to_int(num_viewpoints, "num_viewpoints")
        if count < 1 or count > MAX_VIEWPOINT_COUNT:
            raise ValueError(f"num_viewpoints must be in [1, {MAX_VIEWPOINT_COUNT}]")
        request = PlanViewpoints.Request()
        request.target_xyz_base = [to_float(v, "target_xyz") for v in target]
        request.strategy = str(strategy).strip() or "fibonacci_sphere"
        request.radius_m = to_float(radius_m, "radius_m")
        request.num_viewpoints = count
        request.filter_ik_reachable = bool(filter_ik_reachable)
        request.filter_collision_free = bool(filter_collision_free)
        request.up_vector_base = [to_float(v, "up_vector") for v in up]

        try:
            response = call_service_sync(
                self.plan_client, request, self._service_timeout_sec
            )
        except (RuntimeError, TimeoutError) as exc:
            raise RuntimeError(f"viewpoint plan service call failed: {exc}") from exc
        if not response.success:
            raise RuntimeError(f"viewpoint plan failed: {response.message}")

        payload = _serialize_viewpoint_plan(request, response)
        plan_id, path = write_plan_file(self.share_dir, payload, keep=self._keep)
        payload["plan_id"] = plan_id
        payload["plan_file"] = str(path)
        self._node.get_logger().info(
            f"plan_viewpoints -> {plan_id} ({payload['num_viewpoints']} pts, "
            f"{payload['reachable_count']} reachable)"
        )
        return json.dumps(payload, ensure_ascii=False)

    def list_reachable_viewpoints(self, plan_id: str) -> str:
        """List ik-reachable && collision-free viewpoints of a stored plan."""
        path = plan_file_path(self.share_dir, plan_id)
        data = _parse_viewpoint_plan_file(path)
        rows = [
            row
            for row in data["viewpoints"]
            if row["ik_reachable"] and row["collision_free"]
        ]
        summary = []
        for index, row in enumerate(rows):
            summary.append(
                {
                    "index": index,
                    "tool0_xyz_base": row["tool0_xyz_base"],
                    "camera_xyz_base": row["camera_xyz_base"],
                    "ik_joints_deg": [round(v, 3) for v in row["ik_joints_deg"]],
                    "collision_clearance_m": round(row["collision_clearance"], 4),
                }
            )
        return json.dumps(
            {
                "plan_id": str(plan_id),
                "plan_file": str(path),
                "total_viewpoints": data["num_viewpoints"],
                "reachable_count": len(summary),
                "reachable_viewpoints": summary,
            },
            ensure_ascii=False,
        )

    @staticmethod
    def _validated_xyz(values: Any, label: str) -> list[float]:
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError(f"{label} must be a list of 3 numbers")
        return [to_float(value, label) for value in values]


class ViewpointPlannerMcp(Node):
    """ROS node embedding the planner API (standalone entrypoint only).

    The aggregated ``mcp_servers`` entrypoint does NOT use this class; it
    binds :class:`ViewpointPlannerApi` to the execution host node instead.
    """

    def __init__(self) -> None:
        super().__init__("arachne_viewpoint_planner_mcp")
        self.declare_parameter("viewpoint_plan_service", "/arachne/viewpoint/plan")
        self.declare_parameter("viewpoint_plan_service_timeout_sec", 60.0)
        self.declare_parameter("viewpoint_plan_share_dir", "log/viewpoint_plans")
        self.declare_parameter("viewpoint_plan_share_keep", 20)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", DEFAULT_PLANNER_PORT)
        self.declare_parameter("auth_token", "")
        self.declare_parameter("allowed_hosts", os.environ.get(ALLOWED_HOSTS_ENV, ""))
        self.api = create_planner_api(self)


def _register_tools(mcp: Any, api: ViewpointPlannerApi | None) -> None:
    @mcp.tool()
    def plan_viewpoints(
        target_xyz: list[float],
        strategy: str = "fibonacci_sphere",
        radius_m: float = 0.5,
        num_viewpoints: int = 32,
        filter_ik_reachable: bool = True,
        filter_collision_free: bool = True,
        up_vector: list[float] | None = None,
    ) -> str:
        """Plan viewpoints around a target position (read-only; never moves).

        target_xyz: target position in base_link frame [x, y, z] in meters.
        strategy: sampling strategy name (default "fibonacci_sphere").
        radius_m: sampling sphere radius around the target.
        num_viewpoints: number of candidate viewpoints to sample (1-128).
        filter_ik_reachable: keep only IK-reachable viewpoints.
        filter_collision_free: keep only collision-free viewpoints.
        up_vector: sphere reference axis in base_link frame (default [0,0,1]).
        Returns plan_id (used by acquire_chain/list_reachable_viewpoints),
        the full viewpoint list with per-point ik_reachable/collision_free
        markers, and the absolute plan file path.
        """
        if api is None:  # dry-run manifest only; never called
            raise RuntimeError("planner not available")
        return api.plan_viewpoints(
            target_xyz=target_xyz,
            strategy=strategy,
            radius_m=radius_m,
            num_viewpoints=num_viewpoints,
            filter_ik_reachable=filter_ik_reachable,
            filter_collision_free=filter_collision_free,
            up_vector=up_vector,
        )

    @mcp.tool()
    def list_reachable_viewpoints(plan_id: str) -> str:
        """List the ik-reachable && collision-free viewpoints of a stored plan.

        plan_id: id returned by plan_viewpoints (or the stem of a plan file
        in the shared plan directory).
        Returns per-point index (position inside the filtered list — pass it
        to acquire_chain), tool0/camera position and joint solution.
        """
        if api is None:  # dry-run manifest only; never called
            raise RuntimeError("planner not available")
        return api.list_reachable_viewpoints(plan_id)


def _tool_manifest(mcp: Any) -> str:
    names = [tool.name for tool in mcp._tool_manager.list_tools()]
    return ", ".join(names)


def main(argv: list[str] | None = None) -> None:
    from mcp.server.fastmcp import FastMCP

    parser = argparse.ArgumentParser(description="Arachne viewpoint planner MCP server")
    parser.add_argument(
        "--dry-run-check",
        action="store_true",
        help="validate imports and print the tool manifest without starting ROS/HTTP",
    )
    args, ros_args = parser.parse_known_args(argv)

    if args.dry_run_check:
        mcp = FastMCP("viewpoint-planner", log_level="ERROR")
        _register_tools(mcp, None)
        print(f"viewpoint_planner_mcp dry-run check ok; tools: {_tool_manifest(mcp)}")
        return
    rclpy.init(args=sanitize_ros_args(ros_args))
    node = ViewpointPlannerMcp()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = str(node.get_parameter("bind_host").value)
    port = to_int(node.get_parameter("port").value, "port")
    token = str(node.get_parameter("auth_token").value)
    assert_tokens_configured({"/": token}, {"/": "viewpoint-planner MCP"})

    mcp = build_fastmcp(
        "viewpoint-planner",
        parse_allowed_hosts(node.get_parameter("allowed_hosts").value),
        log_level="ERROR",
    )
    _register_tools(mcp, node.api)
    mcp_app = mcp.streamable_http_app()
    # log_requests outermost so 401s are logged too; /health liveness probe
    # sits outside the bearer guard (the standalone guard is "/" wide).
    app = log_requests(
        with_health_check(
            require_token_by_path(mcp_app, {"/": token}), "viewpoint-planner"
        ),
        node.get_logger().info,
    )
    serve_http_background(app, host, port, managers=(mcp._session_manager,))
    wait_http_ready(host, port)
    node.get_logger().info(
        f"viewpoint-planner MCP listening on http://{host}:{port} "
        f"(read-only; token auth enabled)"
    )
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info("viewpoint-planner MCP interrupted, shutting down")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()