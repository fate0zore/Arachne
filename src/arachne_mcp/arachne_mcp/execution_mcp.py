"""Viewpoint-acquire execution MCP server (Streamable HTTP, motion gated).

Exposes the viewpoint acquire chain (move to viewpoint -> capture RGB/depth/
cloud -> persist) to a remote agent, reusing the headless ``TeachPanelNode``
motion/capture core (plan §2.4 option A + constraint list):

- No autostart of managed processes, no teach-config autoload, own node name
  (``arachne_execution_mcp``) and own ``runtime_log_root``.
- Motion gate: ``motion_enabled`` AND ``confirm_agent_motion`` must both be
  true, else ``acquire_chain`` is rejected (defaults closed — same semantics
  as arachne_agent_bridge ``_require_motion_allowed``).
- Real motion still goes through the existing control-owner file (owner name
  ``teach_panel``, mutually exclusive with the GUI) + teach gate + MoveIt
  collision planning path; no bypass entrypoints are added.
- Viewpoints are read from the shared plan directory by ``plan_id`` (never
  raw joint arrays from the agent).

Tools:
- ``acquire_chain(plan_id, indices)``: asynchronously run the chain.
- ``get_acquire_status()``: poll progress / per-point results.
- ``cancel_acquire()``: actively interrupt in-flight motion (SDK stopJoint +
  velocity-stream stop), terminate the chain and finalize the manifest.
"""

# ROS packages and user-site MCP SDK are outside the workspace LSP's scan
# path; imports resolve at runtime (verified via py_compile + smoke tests).
# pyright: reportMissingImports=false, reportAttributeAccessIssue=false, reportArgumentType=false, reportAssignmentType=false, reportReturnType=false
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
from arachne_operator.teach_panel import (
    AuboDirectJsonRpc,
    TeachPanelNode,
    _parse_viewpoint_plan_file,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from arachne_mcp.common import (
    ALLOWED_HOSTS_ENV,
    assert_tokens_configured,
    build_fastmcp,
    log_requests,
    parse_allowed_hosts,
    plan_file_path,
    require_token_by_path,
    resolve_plan_share_dir,
    sanitize_ros_args,
    serve_http_background,
    to_int,
    wait_http_ready,
    with_health_check,
)

DEFAULT_EXECUTION_PORT = 8124
# Deliberate default per mcp_exposure_todo_plan.md §2.2: the remote agent
# reaches this endpoint over the LAN (bearer-token gated, opt-in startup).
# Override with ARACHNE_MCP_BIND_HOST or bind_host:=127.0.0.1 for local only.
DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_ACQUIRE_CHAIN_MAX_POINTS = 32


class ExecutionMcpNode(TeachPanelNode):
    """TeachPanelNode core + MCP layer: params, motion gate, chain state."""

    def __init__(self) -> None:
        super().__init__(node_name="arachne_execution_mcp")
        # Hard guard: the execution MCP is a headless chain driver — it must
        # NEVER auto-start managed services (camera/grasp_server/...) on its
        # own, or it would fight the GUI for service ownership (plan §2.4
        # constraint list). Enforced here regardless of launch params; the
        # launch file / managed-service command also pass the empty value.
        self.set_parameters([Parameter("autostart_managed_processes", value="")])
        self.autostart_managed_processes = []
        self.declare_parameter(
            "bind_host", os.environ.get("ARACHNE_MCP_BIND_HOST", DEFAULT_BIND_HOST)
        )
        self.declare_parameter("port", DEFAULT_EXECUTION_PORT)
        self.declare_parameter(
            "auth_token", os.environ.get("ARACHNE_MCP_EXECUTION_TOKEN", "")
        )
        self.declare_parameter("allowed_hosts", os.environ.get(ALLOWED_HOSTS_ENV, ""))
        # Motion gate (agent_bridge semantics): both must be true to move.
        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("confirm_agent_motion", False)
        self.declare_parameter("viewpoint_plan_share_dir", "log/viewpoint_plans")
        self.declare_parameter("viewpoint_plan_share_keep", 20)
        self.declare_parameter(
            "acquire_chain_max_points", DEFAULT_ACQUIRE_CHAIN_MAX_POINTS
        )

        self.share_dir = resolve_plan_share_dir(
            self.workspace_root,
            str(self.get_parameter("viewpoint_plan_share_dir").value),
        )
        self._acquire_lock = threading.Lock()
        self._acquire_busy = False
        self._acquire_cancel_requested = False
        self._acquire_thread: threading.Thread | None = None
        self._acquire_state: dict | None = None
        self.get_logger().info(f"shared viewpoint plan dir: {self.share_dir}")

    # -- motion gate ----------------------------------------------------------

    def _require_motion_allowed(self, tool: str) -> None:
        if not bool(self.get_parameter("motion_enabled").value):
            raise PermissionError(
                f"{tool} rejected: motion_enabled:=false (execution MCP motion is "
                "disabled by default; enable with motion_enabled:=true and "
                "confirm_agent_motion:=true)"
            )
        if not bool(self.get_parameter("confirm_agent_motion").value):
            raise PermissionError(
                f"{tool} rejected: confirm_agent_motion:=false (an operator must "
                "explicitly confirm agent motion before acquire_chain may move the arm)"
            )

    # -- acquire chain --------------------------------------------------------

    def acquire_chain(self, plan_id: str, indices: Any = "all") -> str:
        """Gate, load plan, resolve points, start the async chain worker."""
        try:
            self._require_motion_allowed("acquire_chain")
        except PermissionError as exc:
            return self._rejection(str(exc))
        with self._acquire_lock:
            if self._acquire_busy:
                return self._rejection(
                    "acquire_chain rejected: another chain is running; "
                    "poll get_acquire_status or cancel_acquire first"
                )
        try:
            path = plan_file_path(self.share_dir, plan_id)
            data = _parse_viewpoint_plan_file(path)
        except (LookupError, ValueError, KeyError, OSError) as exc:
            return self._rejection(f"acquire_chain rejected: {exc}")
        rows = [
            row
            for row in data["viewpoints"]
            if row["ik_reachable"] and row["collision_free"]
        ]
        if not rows:
            return self._rejection(
                f"acquire_chain rejected: plan {plan_id!r} has no "
                "ik_reachable && collision-free viewpoints"
            )
        try:
            selected = self._resolve_indices(indices, len(rows))
        except ValueError as exc:
            return self._rejection(f"acquire_chain rejected: {exc}")
        max_points = to_int(
            self.get_parameter("acquire_chain_max_points").value,
            "acquire_chain_max_points",
        )
        if len(selected) > max_points:
            return self._rejection(
                f"acquire_chain rejected: {len(selected)} points exceed "
                f"acquire_chain_max_points={max_points}"
            )

        target = data.get("request", {}).get("target_xyz_base")
        try:
            run_state = self.viewpoint_acquire_begin(len(selected), target, len(rows))
        except (OSError, ValueError) as exc:
            return self._rejection(f"acquire_chain init failed: {exc}")

        state: dict = {
            "plan_id": str(plan_id),
            "plan_file": str(path),
            "run_id": run_state["run_id"],
            "run_dir": str(run_state["root"]),
            "target_xyz_base": target,
            "total_reachable": len(rows),
            "rows": rows,
            "indices": selected,
            "current": 0,
            "results": [],
            "started_stamp": self._now_stamp(),
            "finished_stamp": None,
            "cancelled": False,
            "error": None,
        }
        with self._acquire_lock:
            self._acquire_busy = True
            self._acquire_cancel_requested = False
            self._acquire_state = state
        self.cancel_event.clear()
        self._write_event(
            "viewpoint_acquire_all",
            f"mcp acquire chain started: {len(selected)} viewpoints",
            run_id=run_state["run_id"],
            plan_id=str(plan_id),
            num_viewpoints=len(selected),
            indices=selected,
        )
        self._acquire_thread = threading.Thread(
            target=self._acquire_chain_worker, args=(state, run_state), daemon=True
        )
        self._acquire_thread.start()
        return json.dumps(
            {
                "success": True,
                "message": (
                    f"acquire chain accepted: {len(selected)} viewpoints "
                    f"(plan {plan_id!r}, batch {run_state['run_id']})"
                ),
                "plan_id": str(plan_id),
                "run_id": run_state["run_id"],
                "run_dir": str(run_state["root"]),
                "num_viewpoints": len(selected),
                "note": "chain runs asynchronously; poll get_acquire_status",
            },
            ensure_ascii=False,
        )

    def _acquire_chain_worker(self, state: dict, run_state: dict) -> None:
        """Loop over selected viewpoints: move -> capture -> persist."""
        node = self
        try:
            for pos, row in enumerate(state["rows"]):
                if self._acquire_cancel_requested:
                    break
                state["current"] = pos
                index = state["indices"][pos]
                start_joints = node._current_arm_vector()
                ok = node._move_arm_to_joints_worker(
                    row["ik_joints_deg"], f"mcp acquire #{index}"
                )
                if self._acquire_cancel_requested:
                    state["results"].append(
                        {"index": index, "ok": False, "reason": "cancelled"}
                    )
                    break
                if ok:
                    entry = node.viewpoint_acquire_point(run_state, row, index)
                    run_state["manifest"]["viewpoints"].append(entry)
                    try:
                        manifest_path = run_state["root"] / "manifest.json"
                        manifest_path.write_text(
                            json.dumps(
                                run_state["manifest"], ensure_ascii=False, indent=2
                            ),
                            encoding="utf-8",
                        )
                    except OSError as exc:
                        node.get_logger().warning(
                            f"viewpoint acquire manifest write failed: {exc}"
                        )
                    state["results"].append(
                        {
                            "index": index,
                            "ok": True,
                            "path": str(run_state["root"] / str(index)),
                            "errors": entry["errors"] or None,
                            "cloud_point_count": entry.get("cloud_point_count", 0),
                        }
                    )
                    node._status(f"mcp acquire #{index} captured")
                else:
                    end_joints = node._current_arm_vector()
                    node._write_event(
                        "viewpoint_acquire_skip",
                        f"mcp acquire motion to #{index} failed",
                        run_id=run_state["run_id"],
                        viewpoint_index=index,
                        start_joints_rad=(
                            [float(v) for v in start_joints]
                            if start_joints is not None
                            else None
                        ),
                        end_joints_rad=(
                            [float(v) for v in end_joints]
                            if end_joints is not None
                            else None
                        ),
                        target_joints_rad=row.get("ik_joints_rad"),
                    )
                    state["results"].append(
                        {
                            "index": index,
                            "ok": False,
                            "reason": "motion blocked/timeout (MoveIt plan failure or "
                            "keepout/arrival rejection)",
                        }
                    )
                    node._status(
                        f"mcp acquire #{index} skipped (motion blocked/timeout)",
                        warn=True,
                    )
            cancelled = self._acquire_cancel_requested
            if cancelled:
                node._write_event(
                    "viewpoint_acquire_cancel",
                    f"mcp acquire chain cancelled after {len(state['results'])} points",
                    run_id=run_state["run_id"],
                    plan_id=state["plan_id"],
                )
            node._write_event(
                "viewpoint_acquire_all",
                (
                    f"mcp acquire chain finished: {len(state['results'])} points "
                    f"({'cancelled' if cancelled else 'complete'})"
                ),
                run_id=run_state["run_id"],
                cancelled=cancelled,
            )
            node._status(
                f"mcp acquire finished: {len(state['results'])} points"
                + (" (cancelled)" if cancelled else "")
                + (f" (batch {run_state['run_id']})" if run_state["run_id"] else "")
            )
        except Exception as exc:  # noqa: BLE001 - worker must always release busy
            node.get_logger().error(f"mcp acquire chain failed: {exc}")
            state["error"] = str(exc)
        finally:
            state["finished_stamp"] = self._now_stamp()
            with self._acquire_lock:
                self._acquire_busy = False

    def get_acquire_status(self) -> str:
        """Poll current chain state / last run result."""
        with self._acquire_lock:
            state = self._acquire_state
        if state is None:
            return json.dumps(
                {"busy": False, "message": "no acquire chain has run yet"},
                ensure_ascii=False,
            )
        return json.dumps(
            {
                "busy": self._acquire_busy,
                "plan_id": state["plan_id"],
                "run_id": state["run_id"],
                "run_dir": state["run_dir"],
                "target_xyz_base": state["target_xyz_base"],
                "total_points": len(state["indices"]),
                "current_index": state["current"],
                "completed": len(state["results"]),
                "cancelled": state["cancelled"],
                "error": state["error"],
                "started_stamp": state["started_stamp"],
                "finished_stamp": state["finished_stamp"],
                "points": state["results"],
            },
            ensure_ascii=False,
        )

    def cancel_acquire(self) -> str:
        """Actively interrupt in-flight motion and terminate the chain."""
        with self._acquire_lock:
            if not self._acquire_busy or self._acquire_state is None:
                return json.dumps(
                    {
                        "success": False,
                        "message": "no active acquire chain to cancel",
                    },
                    ensure_ascii=False,
                )
            state = self._acquire_state
        self._acquire_cancel_requested = True
        self.cancel_event.set()
        # 1) stop the velocity stream (and publish zeros)
        try:
            self.stop_arm_velocity_hold()
        except Exception as exc:  # noqa: BLE001 - best effort
            self.get_logger().warning(f"cancel: velocity hold stop failed: {exc}")
        # 2) actively interrupt any in-flight SDK moveJoint
        try:
            ip = str(self.get_parameter("aubo_sdk_ip").value)
            port = to_int(
                self.get_parameter("aubo_sdk_rpc_port").value, "aubo_sdk_rpc_port"
            )
            timeout = max(
                float(self.get_parameter("aubo_sdk_rpc_timeout_sec").value), 0.1
            )
            with AuboDirectJsonRpc(ip, port, timeout) as rpc:
                self._sdk_stop_joint(rpc, "mcp acquire cancel", warn_only=True)
        except Exception as exc:  # noqa: BLE001 - best effort
            self.get_logger().warning(f"cancel: SDK stopJoint failed: {exc}")
        # 3) wait for the chain worker to release busy + finalize the manifest
        thread = self._acquire_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=8.0)
        with self._acquire_lock:
            state["cancelled"] = True
            still_busy = self._acquire_busy
        return json.dumps(
            {
                "success": True,
                "message": (
                    "cancel requested: in-flight motion stopped (stopJoint + "
                    "velocity-zero), chain terminated"
                    if not still_busy
                    else "cancel requested but worker is still finalizing; "
                    "poll get_acquire_status"
                ),
                "cancelled": True,
                "points_completed": len(state["results"]),
            },
            ensure_ascii=False,
        )

    # -- helpers --------------------------------------------------------------

    @staticmethod
    def _rejection(reason: str) -> str:
        return json.dumps(
            {"success": False, "message": str(reason)}, ensure_ascii=False
        )

    @staticmethod
    def _now_stamp() -> str:
        from datetime import datetime

        return datetime.now().isoformat(timespec="milliseconds")

    @staticmethod
    def _resolve_indices(indices: Any, total: int) -> list[int]:
        if indices is None or indices == "all":
            return list(range(total))
        if isinstance(indices, str):
            indices = [part.strip() for part in indices.split(",") if part.strip()]
        if not isinstance(indices, (list, tuple)):
            raise ValueError("indices must be 'all' or a list of integers")
        resolved: list[int] = []
        for raw in indices:
            index = to_int(raw, "indices")
            if index < 0 or index >= total:
                raise ValueError(
                    f"index {index} out of range [0, {total}) "
                    "(indices refer to the filtered reachable list)"
                )
            if index not in resolved:
                resolved.append(index)
        if not resolved:
            raise ValueError("indices resolved to an empty list")
        return resolved


def _register_tools(mcp: Any, node: ExecutionMcpNode | None) -> None:
    @mcp.tool()
    def acquire_chain(plan_id: str, indices: Any = "all") -> str:
        """Run the viewpoint acquire chain for a stored plan (moves the arm!).

        plan_id: id returned by plan_viewpoints (shared plan directory).
        indices: 'all' (default) or a list of positions inside the filtered
        ik-reachable && collision-free list (see list_reachable_viewpoints).
        Requires motion_enabled:=true AND confirm_agent_motion:=true on the
        execution MCP server; otherwise the call is rejected and nothing moves.
        Runs asynchronously: returns immediately with run_id; poll
        get_acquire_status for per-point results and absolute capture paths.
        """
        if node is None:  # dry-run manifest only; never called
            raise RuntimeError("execution node not available")
        return node.acquire_chain(plan_id, indices)

    @mcp.tool()
    def get_acquire_status() -> str:
        """Poll the running (or last) acquire chain: busy flag, progress,
        per-point success/failure, absolute capture paths, cancellation state.
        """
        if node is None:  # dry-run manifest only; never called
            raise RuntimeError("execution node not available")
        return node.get_acquire_status()

    @mcp.tool()
    def cancel_acquire() -> str:
        """Safely stop the running acquire chain: actively interrupts the
        in-flight motion (SDK stopJoint + velocity-stream stop), terminates
        the chain, and finalizes the batch manifest. Safe to call at any time.
        """
        if node is None:  # dry-run manifest only; never called
            raise RuntimeError("execution node not available")
        return node.cancel_acquire()


def _tool_manifest(mcp: Any) -> str:
    names = [tool.name for tool in mcp._tool_manager.list_tools()]
    return ", ".join(names)


def main(argv: list[str] | None = None) -> None:
    from mcp.server.fastmcp import FastMCP

    parser = argparse.ArgumentParser(description="Arachne viewpoint acquire MCP server")
    parser.add_argument(
        "--dry-run-check",
        action="store_true",
        help="validate imports and print the tool manifest without starting ROS/HTTP",
    )
    args, ros_args = parser.parse_known_args(argv)

    if args.dry_run_check:
        mcp = FastMCP("viewpoint-acquire", log_level="ERROR")
        _register_tools(mcp, None)
        print(f"viewpoint_acquire_mcp dry-run check ok; tools: {_tool_manifest(mcp)}")
        return
    rclpy.init(args=sanitize_ros_args(ros_args))
    node = ExecutionMcpNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = str(node.get_parameter("bind_host").value)
    port = to_int(node.get_parameter("port").value, "port")
    token = str(node.get_parameter("auth_token").value)
    assert_tokens_configured({ "/": token }, {"/": "viewpoint-acquire MCP"})

    mcp = build_fastmcp(
        "viewpoint-acquire",
        parse_allowed_hosts(node.get_parameter("allowed_hosts").value),
        log_level="ERROR",
    )
    _register_tools(mcp, node)
    mcp_app = mcp.streamable_http_app()
    # log_requests outermost so 401s are logged too; /health liveness probe
    # sits outside the bearer guard (the standalone guard is "/" wide).
    app = log_requests(
        with_health_check(
            require_token_by_path(mcp_app, {"/": token}), "viewpoint-acquire"
        ),
        node.get_logger().info,
    )
    serve_http_background(app, host, port, managers=(mcp._session_manager,))
    wait_http_ready(host, port)
    node.get_logger().info(
        f"viewpoint-acquire MCP listening on http://{host}:{port} "
        f"(motion gate: motion_enabled={node.get_parameter('motion_enabled').value}, "
        f"confirm_agent_motion={node.get_parameter('confirm_agent_motion').value})"
    )
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        node.cancel_acquire()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
