"""Shared helpers for the Arachne MCP servers.

Pure stdlib + starlette/rclpy building blocks so the two server entrypoints
stay thin and testable offline.
"""

# pyright: reportMissingImports=false
# starlette/uvicorn live in the user site-packages (~/.local/...), which the
# workspace LSP does not scan; imports resolve at runtime (verified).
from __future__ import annotations

import json
import logging
import threading
from datetime import datetime
from pathlib import Path
from typing import Any

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import JSONResponse

logger = logging.getLogger("arachne_mcp")

# Shared plan-file prefix; plan_id == the millisecond-timestamp stem.
PLAN_FILE_PREFIX = "viewpoint_plan_"
PLAN_FILE_SUFFIX = ".json"


def to_float(value: Any, label: str) -> float:
    """Convert MCP tool input to float with a clear error (never throws raw)."""
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: expected a number, got {value!r}") from exc


def to_int(value: Any, label: str) -> int:
    """Convert MCP tool input to int with a clear error (never throws raw)."""
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: expected an integer, got {value!r}") from exc


# ---------------------------------------------------------------------------
# Shared viewpoint plan directory (data flow between the two servers)
# ---------------------------------------------------------------------------


def resolve_plan_share_dir(workspace_root: Path, configured: str) -> Path:
    """Resolve the shared plan directory (workspace-relative when relative)."""
    path = Path(str(configured)).expanduser()
    if not path.is_absolute():
        path = Path(workspace_root) / path
    path.mkdir(parents=True, exist_ok=True)
    return path.resolve()


def write_plan_file(share_dir: Path, payload: dict, keep: int = 20) -> tuple[str, Path]:
    """Persist a serialized viewpoint plan; returns (plan_id, path).

    plan_id is the millisecond timestamp stem, matching the file naming used
    by the teach panel (``viewpoint_plan_<stamp>.json``). Keeps the newest
    ``keep`` files, pruning older ones lazily (no background thread).
    """
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = share_dir / f"{PLAN_FILE_PREFIX}{stamp}{PLAN_FILE_SUFFIX}"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    _prune_plan_files(share_dir, keep)
    return stamp, path


def plan_file_path(share_dir: Path, plan_id: str) -> Path:
    """Validate a plan_id and return its file path (raises LookupError)."""
    plan_id = str(plan_id).strip()
    if not plan_id:
        raise LookupError("plan_id must not be empty")
    if "/" in plan_id or "\\" in plan_id or ".." in plan_id:
        raise LookupError(f"invalid plan_id: {plan_id!r}")
    path = share_dir / f"{PLAN_FILE_PREFIX}{plan_id}{PLAN_FILE_SUFFIX}"
    if not path.is_file():
        raise LookupError(
            f"viewpoint plan {plan_id!r} not found in shared plan dir "
            f"{share_dir} (run plan_viewpoints first, or check the share dir)"
        )
    return path


def list_plan_ids(share_dir: Path) -> list[str]:
    """Return plan ids (newest first) for operator/agent discovery."""
    if not share_dir.is_dir():
        return []
    ids = []
    for path in sorted(share_dir.glob(f"{PLAN_FILE_PREFIX}*{PLAN_FILE_SUFFIX}")):
        ids.append(path.name[len(PLAN_FILE_PREFIX) : -len(PLAN_FILE_SUFFIX)])
    return list(reversed(ids))


def _prune_plan_files(share_dir: Path, keep: int) -> None:
    try:
        keep = max(int(keep), 1)
    except (TypeError, ValueError):
        keep = 20
    files = sorted(share_dir.glob(f"{PLAN_FILE_PREFIX}*{PLAN_FILE_SUFFIX}"))
    for stale in files[:-keep]:
        try:
            stale.unlink()
        except OSError as exc:
            logger.warning("plan prune failed for %s: %s", stale, exc)


# ---------------------------------------------------------------------------
# Path-scoped bearer-token middleware (streamable HTTP resource guard)
# ---------------------------------------------------------------------------


def require_token_by_path(app: Any, rules: dict[str, str]) -> Any:
    """Wrap the MCP Starlette app so requests are authorized per-path.

    ``rules`` maps a path prefix (e.g. ``/arachne/viewpoint/plan``) to the
    bearer token that guards it. A request whose path starts with a rule
    prefix must carry ``Authorization: Bearer <token>``; requests that match
    no prefix are passed through to the app (which 404s them since a single
    wrapper app only mounts the known MCP endpoints).

    Tokens are expected to be non-empty (see ``assert_tokens_configured``),
    so this path only runs with real secrets; the check is still applied for
    defense in depth.
    """

    def _token_for(path: str) -> str | None:
        for prefix, token in rules.items():
            if path.startswith(prefix):
                return token
        return None

    class _PathBearerGuard(BaseHTTPMiddleware):
        async def dispatch(self, request: Request, call_next):
            token = _token_for(request.url.path)
            if token is not None:
                header = request.headers.get("authorization", "")
                if not header.lower().startswith("bearer ") or header[7:].strip() != token:
                    return JSONResponse(
                        {
                            "error": "unauthorized",
                            "detail": "missing or invalid bearer token",
                        },
                        status_code=401,
                    )
            return await call_next(request)

    return _PathBearerGuard(app)


def assert_tokens_configured(tokens: dict[str, str], labels: dict[str, str]) -> None:
    """Refuse to serve any endpoint whose path token is empty (no token, no
    listener for that capability). Keeps every MCP capability opt-in and
    credential-gated, matching the previous per-server semantics."""
    missing = [labels.get(path, path) for path, token in tokens.items() if not str(token or "").strip()]
    if missing:
        raise RuntimeError(
            "no MCP auth token configured for: "
            + ", ".join(missing)
            + ". Set the matching ARACHNE_MCP_*_TOKEN environment variable "
            "(or ROS parameter); refusing to expose the endpoint(s)."
        )


# Token-free liveness probe: answers whether the HTTP listener is up without
# requiring an MCP handshake or a credential. It exposes no capability, no
# state and no data, only "the process that serves MCP is listening".
HEALTH_PATH = "/health"


def with_health_check(app: Any, service_name: str) -> Any:
    """ASGI wrapper answering ``GET/HEAD /health`` with a small liveness JSON.

    Purpose: let agents / scripts / monitoring decide "is the MCP up?" with a
    plain ``curl http://<host>:<port>/health`` instead of a full MCP
    initialize exchange. It must sit OUTSIDE the bearer-token guard (a probe
    that needs a credential cannot probe), which is safe because the response
    carries only ``status=ok`` + a service name — no capability surface, no
    data. All other paths pass through to *app* unchanged.
    """
    payload = json.dumps({"status": "ok", "service": service_name}).encode("utf-8")

    async def _health(scope, receive, send) -> None:
        if scope.get("type") != "http" or scope.get("path") != HEALTH_PATH:
            await app(scope, receive, send)
            return
        method = (scope.get("method") or "GET").upper()
        if method not in ("GET", "HEAD"):
            await app(scope, receive, send)
            return
        await send(
            {
                "type": "http.response.start",
                "status": 200,
                "headers": [
                    (b"content-type", b"application/json; charset=utf-8"),
                    (b"content-length", str(len(payload)).encode()),
                    (b"cache-control", b"no-store"),
                ],
            }
        )
        await send({"type": "http.response.body", "body": payload if method == "GET" else b""})

    return _health

# ---------------------------------------------------------------------------
# FastMCP construction with configurable Host-header (DNS rebinding) checks
# ---------------------------------------------------------------------------

ALLOWED_HOSTS_ENV = "ARACHNE_MCP_ALLOWED_HOSTS"


def parse_allowed_hosts(raw: Any) -> list[str]:
    """Split a comma-separated allowed-Host pattern list; empty -> unset."""
    if raw is None:
        return []
    parts: list[Any] = list(raw) if isinstance(raw, (list, tuple)) else str(raw).split(",")
    return [p.strip() for p in parts if str(p).strip()]


def transport_security_for(allowed_hosts: list[str]) -> Any:
    """TransportSecuritySettings for the LAN-facing MCP servers.

    The MCP SDK auto-enables DNS-rebinding protection when the FastMCP
    ``host`` looks like localhost, which 421-rejects any LAN-IP ``Host``
    header. Our servers bind 0.0.0.0 by design (LAN serving behind bearer
    tokens), so protection is OFF unless the operator configures explicit
    ``allowed_hosts`` patterns. Patterns are ``host`` or ``host:*``; the
    wildcard covers the port only and the host matches exactly up to the
    colon (the SDK's own matching semantics, e.g. ``192.168.1.50:*``).
    Explicit settings replace the SDK's implicit loopback allowance: add
    ``127.0.0.1:*`` to keep serving local clients when tightening.
    """
    from mcp.server.transport_security import TransportSecuritySettings

    if not allowed_hosts:
        return TransportSecuritySettings(enable_dns_rebinding_protection=False)
    return TransportSecuritySettings(
        enable_dns_rebinding_protection=True,
        allowed_hosts=list(allowed_hosts),
        # Origin is browser-only; absent Origin passes. Derive from the host
        # patterns so the check stays meaningful if a web client appears.
        allowed_origins=[f"http://{pattern}" for pattern in allowed_hosts],
    )


def build_fastmcp(name: str, allowed_hosts: list[str], **kwargs: Any) -> Any:
    """FastMCP factory for every Arachne serving entrypoint: one construction
    path so the Host-header policy cannot drift between them."""
    from mcp.server.fastmcp import FastMCP

    return FastMCP(
        name, transport_security=transport_security_for(allowed_hosts), **kwargs
    )


def sanitize_ros_args(ros_args: list[str]) -> list[str]:
    """Drop ``-p name:=`` overrides whose value the shell left empty.

    ``${VAR:-}`` expansion in launch command strings turns unset variables
    into ``name:=`` with no value, which rcl rejects ("error not set") at
    ``rclpy.init``. Dropping the rule lets the parameter keep its declared
    default -- which is exactly the unset-variable semantics we want.
    """
    param_flags = ("-p", "--param")
    out: list[str] = []
    i = 0
    while i < len(ros_args):
        arg = ros_args[i]
        if arg in param_flags and i + 1 < len(ros_args) and ros_args[i + 1].endswith(":="):
            i += 2  # drop the flag and its empty-valued name together
            continue
        out.append(arg)
        i += 1
    return out

def log_requests(
    app: Any,
    log: Any,
    max_log: int = 1024,
    *,
    include_headers: bool = False,
    include_response_bodies: bool = False,
) -> Any:
    """ASGI middleware with concise per-request logging via *log*.

    Default output per request (2-3 lines):
    - ``request <METHOD> <path> [body (NB): ...]`` -- body once received;
    - ``response <status> for <METHOD> <path>``.

    ``include_response_bodies=True`` also logs every response body chunk
    (SSE streams log one line per chunk) -- very verbose, debugging only.
    Without it, bodies are logged only for non-2xx responses (errors stay
    diagnosable without flooding the log with 10KB+ plan JSON). 
    ``include_headers=True`` adds a redacted header line (Authorization
    value masked to its scheme). Bodies/chunks are truncated to *max_log*.
    """

    def _clip(data: bytes) -> str:
        text = data[:max_log].decode("utf-8", "replace")
        if len(data) > max_log:
            text += f" ...[truncated {len(data) - max_log}B]"
        return text

    def _headers_line(scope: Any) -> str:
        parts: list[str] = []
        for raw_key, raw_val in scope.get("headers") or []:
            key = raw_key.decode("latin-1")
            val = raw_val.decode("latin-1")
            if key.lower() == "authorization":
                scheme = val.split(" ", 1)[0]
                val = f"{scheme} ***redacted***"
            parts.append(f"{key}={val}")
        return " ".join(parts)

    async def _logged(scope, receive, send) -> None:
        if scope.get("type") != "http":
            await app(scope, receive, send)
            return
        method = scope.get("method", "?")
        path = scope.get("path", "?")
        header_note = f" headers: {_headers_line(scope)}" if include_headers else ""
        log(f"request {method} {path}{header_note}")
        body_buf = bytearray()
        body_logged = False
        status = 0

        async def _receive() -> Any:
            nonlocal body_logged
            message = await receive()
            if message.get("type") == "http.request" and not body_logged:
                body_buf.extend(message.get("body", b""))
                if not message.get("more_body", False):
                    body_logged = True
                    if body_buf:
                        log(f"request body ({len(body_buf)}B): {_clip(bytes(body_buf))}")
            return message

        async def _send(message: Any) -> None:
            nonlocal status
            if message.get("type") == "http.response.start":
                status = message.get("status", 0)
                log(f"response {status} for {method} {path}")
            elif message.get("type") == "http.response.body":
                chunk = message.get("body", b"")
                if chunk and (include_response_bodies or status >= 400):
                    log(f"response body ({len(chunk)}B): {_clip(chunk)}")
            await send(message)

        try:
            await app(scope, _receive, _send)
        except Exception as exc:
            log(f"request {method} {path} failed: {exc!r}")
            raise

    return _logged


# ---------------------------------------------------------------------------
# Thread-safe ROS service call from MCP tool threads
# ---------------------------------------------------------------------------


def call_service_sync(client: Any, request: Any, timeout_sec: float) -> Any:
    """Call a ROS service from a non-spin thread while an executor owns the
    node elsewhere. ``client.call_async`` is documented thread-safe; we wait
    on a done-callback event so no executor API is touched from this thread.
    """
    if not client.service_is_ready():
        client.wait_for_service(timeout_sec=0.05)
    if not client.service_is_ready():
        raise RuntimeError(
            f"ROS service {client.srv_name} not available "
            "(start the viewpoint planner first)"
        )
    future = client.call_async(request)
    done = threading.Event()

    def _on_done(_future: Any) -> None:
        done.set()

    future.add_done_callback(_on_done)
    if not done.wait(timeout_sec):
        future.cancel()
        raise TimeoutError(
            f"ROS service {client.srv_name} timed out after {timeout_sec:.0f}s"
        )
    return future.result()


# ---------------------------------------------------------------------------
# HTTP serving on a background thread
# ---------------------------------------------------------------------------


def serve_http_background(app: Any, host: str, port: int, *, managers: tuple = ()) -> threading.Thread:
    """Run uvicorn for the MCP app on a daemon thread (rclpy stays on the
    main thread). Returns the thread; callers should verify readiness.

    ``managers``: optional FastMCP ``StreamableHTTPSessionManager`` instances
    whose ``run()`` context must stay alive for the whole serving lifetime
    (their ASGI apps error out otherwise). uvicorn's own lifespan is
    disabled because the top-level app is a plain ASGI dispatcher/middleware,
    not the FastMCP Starlette app that declares a lifespan.
    """
    import asyncio
    import uvicorn  # noqa: PLC0415 - keep heavy HTTP import out of import path

    async def _serve() -> None:
        from contextlib import AsyncExitStack

        async with AsyncExitStack() as stack:
            for manager in managers:
                await stack.enter_async_context(manager.run())
            config = uvicorn.Config(
                app,
                host=host,
                port=port,
                log_level="warning",
                lifespan="off",
            )
            await uvicorn.Server(config).serve()

    thread = threading.Thread(
        target=lambda: asyncio.run(_serve()),
        daemon=True,
        name="mcp-http",
    )
    thread.start()
    return thread


def wait_http_ready(host: str, port: int, timeout_sec: float = 10.0) -> None:
    """Block until the HTTP listener accepts connections (or timeout)."""
    import socket
    import time

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError(f"MCP HTTP listener did not come up on {host}:{port}")
