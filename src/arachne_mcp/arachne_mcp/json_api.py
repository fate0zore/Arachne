"""Plain-JSON HTTP overlay for the Arachne MCP capabilities.

The Streamable-HTTP MCP endpoints speak the MCP JSON-RPC protocol. This
module exposes the *same* five tools as simple JSON POST/GET endpoints on the
same single port, so a non-MCP agent (plain HTTP client + bearer token) can
drive the identical capability surface.

Endpoints (all bearer-token gated, same privilege split as the MCP paths):

    POST /arachne/json/viewpoint/plan/plan_viewpoints           (planner token)
    POST /arachne/json/viewpoint/plan/list_reachable_viewpoints (planner token)
    POST /arachne/json/viewpoint/acquire/acquire_chain          (execution token)
    GET  /arachne/json/viewpoint/acquire/status                 (execution token)
    POST /arachne/json/viewpoint/acquire/cancel                 (execution token)

Request: JSON object of the tool's arguments (see the tool docstrings).
Response: JSON object, either
    {"ok": true,  "result": <tool output object>}
or  {"ok": false, "error": "<reason>"}
with an HTTP status of 200 / 400 (bad args) / 403 (motion gate) / 404
(plan_id not found) / 503 (planner service unavailable) / 500 (internal).

Example (planner):
    curl -sS -X POST http://<host>:8123/arachne/json/viewpoint/plan/plan_viewpoints \
      -H "Authorization: Bearer $ARACHNE_MCP_PLANNER_TOKEN" \
      -H "Content-Type: application/json" \
      -d '{"target_xyz":[0.3,0.0,0.2],"num_viewpoints":32}'

Example (execution, motion must be explicitly enabled):
    curl -sS -X POST http://<host>:8123/arachne/json/viewpoint/acquire/acquire_chain \
      -H "Authorization: Bearer $ARACHNE_MCP_EXECUTION_TOKEN" \
      -H "Content-Type: application/json" \
      -d '{"plan_id":"<plan_id>","indices":"all"}'
    curl -sS http://<host>:8123/arachne/json/viewpoint/acquire/status \
      -H "Authorization: Bearer $ARACHNE_MCP_EXECUTION_TOKEN"
"""

from __future__ import annotations

import json
from typing import Any

from starlette.responses import JSONResponse

# Path prefixes, matching the per-capability bearer-token split of the MCP
# paths (planner read-only vs execution motion). Everything under each prefix
# inherits that capability's token.
PLAN_JSON_PREFIX = "/arachne/json/viewpoint/plan"
ACQUIRE_JSON_PREFIX = "/arachne/json/viewpoint/acquire"

# Full endpoint path -> (HTTP method, handler name). Handlers are methods on
# :class:`JsonApi` taking the parsed request body dict.
_ENDPOINTS: dict[str, tuple[str, str]] = {
    f"{PLAN_JSON_PREFIX}/plan_viewpoints": ("POST", "plan_viewpoints"),
    f"{PLAN_JSON_PREFIX}/list_reachable_viewpoints": ("POST", "list_reachable_viewpoints"),
    f"{ACQUIRE_JSON_PREFIX}/acquire_chain": ("POST", "acquire_chain"),
    f"{ACQUIRE_JSON_PREFIX}/status": ("GET", "status"),
    f"{ACQUIRE_JSON_PREFIX}/cancel": ("POST", "cancel"),
}


def _require(body: dict, key: str) -> Any:
    """Return *body*[key] or raise a clear 400-worthy error."""
    if key not in body:
        raise ValueError(f"missing required argument {key!r}")
    return body[key]


class JsonApi:
    """Thin JSON facade over the planner API + execution node.

    Holds no ROS/HTTP state itself: every handler calls the same underlying
    capability the MCP tools call and parses the returned JSON string, so the
    JSON surface and the MCP surface are always in lockstep.
    """

    def __init__(self, planner_api: Any, node: Any) -> None:
        self._api = planner_api
        self._node = node

    # -- handlers (all return JSON-serializable dicts) ----------------------

    def plan_viewpoints(self, body: dict) -> dict:
        return json.loads(
            self._api.plan_viewpoints(
                target_xyz=_require(body, "target_xyz"),
                strategy=body.get("strategy", "fibonacci_sphere"),
                radius_m=body.get("radius_m", 0.5),
                num_viewpoints=body.get("num_viewpoints", 32),
                filter_ik_reachable=body.get("filter_ik_reachable", True),
                filter_collision_free=body.get("filter_collision_free", True),
                up_vector=body.get("up_vector"),
            )
        )

    def list_reachable_viewpoints(self, body: dict) -> dict:
        return json.loads(
            self._api.list_reachable_viewpoints(_require(body, "plan_id"))
        )

    def acquire_chain(self, body: dict) -> dict:
        out = self._node.acquire_chain(
            _require(body, "plan_id"), body.get("indices", "all")
        )
        return self._require_ok(out)

    def status(self, body: dict) -> dict:
        return json.loads(self._node.get_acquire_status())

    def cancel(self, body: dict) -> dict:
        return self._require_ok(self._node.cancel_acquire())

    @staticmethod
    def _require_ok(out: str) -> dict:
        """Surface a ``success:false`` tool result as an HTTP error."""
        result = json.loads(out)
        if isinstance(result, dict) and result.get("success") is False:
            raise RuntimeError(result.get("message", "request rejected"))
        return result

    # -- ASGI dispatch ------------------------------------------------------

    async def route(self, scope: dict, receive: Any, send: Any) -> bool:
        """Handle a JSON endpoint if *scope* path matches; else return False."""
        if scope.get("type") != "http":
            return False
        path = scope.get("path", "")
        entry = _ENDPOINTS.get(path)
        if entry is None:
            return False
        expected_method, handler = entry
        method = (scope.get("method") or "GET").upper()
        if method != expected_method:
            await JSONResponse(
                {
                    "ok": False,
                    "error": f"method not allowed (expected {expected_method})",
                },
                status_code=405,
            )(scope, receive, send)
            return True
        try:
            body = await self._read_body(receive)
            payload = json.loads(body.decode("utf-8") or "{}") if body else {}
            result = getattr(self, handler)(payload)
            await JSONResponse({"ok": True, "result": result}, status_code=200)(
                scope, receive, send
            )
        except json.JSONDecodeError as exc:
            await JSONResponse(
                {"ok": False, "error": f"invalid JSON body: {exc}"},
                status_code=400,
            )(scope, receive, send)
        except LookupError as exc:
            await JSONResponse(
                {"ok": False, "error": str(exc)}, status_code=404
            )(scope, receive, send)
        except PermissionError as exc:
            await JSONResponse(
                {"ok": False, "error": str(exc)}, status_code=403
            )(scope, receive, send)
        except ValueError as exc:
            await JSONResponse(
                {"ok": False, "error": str(exc)}, status_code=400
            )(scope, receive, send)
        except RuntimeError as exc:
            await JSONResponse(
                {"ok": False, "error": str(exc)}, status_code=503
            )(scope, receive, send)
        except Exception as exc:  # noqa: BLE001 - always return structured JSON
            await JSONResponse(
                {"ok": False, "error": f"internal error: {exc}"},
                status_code=500,
            )(scope, receive, send)
        return True

    @staticmethod
    async def _read_body(receive: Any) -> bytes:
        body = b""
        while True:
            message = await receive()
            body += message.get("body", b"")
            if not message.get("more_body", False):
                return body
