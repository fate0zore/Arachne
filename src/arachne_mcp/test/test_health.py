"""Offline unit tests for the token-free /health liveness probe.

Pure stdlib + MCP SDK (no ROS / hardware). Exercises:
- ``with_health_check``: GET/HEAD /health answers 200 with a minimal JSON
  payload (no capability data), all other paths pass through unchanged;
- the probe sits OUTSIDE the bearer guard in the real composition used by
  both the standalone entrypoints (guard "/" wide) and the aggregate
  (guard per path prefix): /health stays reachable WITHOUT a token while
  every guarded path still 401s without one.

Run from the workspace root with install/setup.bash sourced:

    python3 -m unittest discover -s src/arachne_mcp/test -p "test_*.py"
"""

# Workspace packages resolve at runtime via install/setup.bash (colcon); the
# LSP cannot see them, same as the server modules.
# pyright: reportMissingImports=false
from __future__ import annotations

import json
import unittest

import anyio
import httpx
from arachne_mcp.common import (
    HEALTH_PATH,
    require_token_by_path,
    with_health_check,
)


async def _inner_handler(scope, receive, send):
    """Echoes the request path back so passthrough behavior is observable."""
    path = scope.get("path", "?").encode()
    await send(
        {
            "type": "http.response.start",
            "status": 200,
            "headers": [
                (b"content-type", b"text/plain; charset=utf-8"),
                (b"content-length", str(len(path)).encode()),
            ],
        }
    )
    await send({"type": "http.response.body", "body": path})


class HealthProbeTest(unittest.TestCase):
    def _request(self, app, method: str, path: str, headers=None) -> httpx.Response:
        async def _main() -> httpx.Response:
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://t:1"
            ) as client:
                return await client.request(method, path, headers=headers)

        return anyio.run(_main)

    def test_get_health_returns_ok_json_without_token(self) -> None:
        app = with_health_check(_inner_handler, "arachne_mcp")
        response = self._request(app, "GET", HEALTH_PATH)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            json.loads(response.content),
            {"status": "ok", "service": "arachne_mcp"},
        )
        self.assertEqual(response.headers["content-type"], "application/json; charset=utf-8")
        self.assertEqual(response.headers["cache-control"], "no-store")

    def test_head_health_returns_ok_with_empty_body(self) -> None:
        app = with_health_check(_inner_handler, "viewpoint-planner")
        response = self._request(app, "HEAD", HEALTH_PATH)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.content, b"")

    def test_other_paths_pass_through(self) -> None:
        app = with_health_check(_inner_handler, "arachne_mcp")
        response = self._request(app, "GET", "/arachne/viewpoint/plan")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.content, b"/arachne/viewpoint/plan")

    def test_non_get_head_on_health_passes_through(self) -> None:
        # POST /health is not a probe; it must reach the inner app (which 404s
        # on unknown MCP paths) instead of being answered as liveness.
        app = with_health_check(_inner_handler, "arachne_mcp")
        response = self._request(app, "POST", HEALTH_PATH)
        self.assertEqual(response.content, HEALTH_PATH.encode())

    def test_health_ignores_query_string(self) -> None:
        app = with_health_check(_inner_handler, "arachne_mcp")
        response = self._request(app, "GET", f"{HEALTH_PATH}?x=1")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(json.loads(response.content)["status"], "ok")


class HealthOutsideBearerGuardTest(HealthProbeTest):
    """The real composition: standalone guard is "/" wide, aggregate guard is
    per path prefix. In both, /health must stay reachable without a token
    while guarded paths keep 401ing."""

    def test_standalone_composition_health_without_token(self) -> None:
        app = with_health_check(
            require_token_by_path(_inner_handler, {"/": "secret"}), "viewpoint-acquire"
        )
        response = self._request(app, "GET", HEALTH_PATH)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            json.loads(response.content)["service"], "viewpoint-acquire"
        )

    def test_standalone_composition_guarded_path_still_401(self) -> None:
        app = with_health_check(
            require_token_by_path(_inner_handler, {"/": "secret"}), "viewpoint-acquire"
        )
        response = self._request(app, "GET", "/mcp")
        self.assertEqual(response.status_code, 401)
        # ... and succeeds with the token.
        response = self._request(
            app, "GET", "/mcp", headers={"Authorization": "Bearer secret"}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.content, b"/mcp")

    def test_aggregate_composition_health_without_token(self) -> None:
        app = with_health_check(
            require_token_by_path(
                _inner_handler,
                {"/arachne/viewpoint/plan": "plan-tok", "/arachne/viewpoint/acquire": "exe-tok"},
            ),
            "arachne_mcp",
        )
        response = self._request(app, "GET", HEALTH_PATH)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(json.loads(response.content)["service"], "arachne_mcp")

    def test_aggregate_composition_guarded_path_has_own_token(self) -> None:
        app = with_health_check(
            require_token_by_path(
                _inner_handler,
                {"/arachne/viewpoint/plan": "plan-tok", "/arachne/viewpoint/acquire": "exe-tok"},
            ),
            "arachne_mcp",
        )
        no_token = self._request(app, "GET", "/arachne/viewpoint/acquire")
        self.assertEqual(no_token.status_code, 401)
        wrong_token = self._request(
            app,
            "GET",
            "/arachne/viewpoint/acquire",
            headers={"Authorization": "Bearer plan-tok"},
        )
        self.assertEqual(wrong_token.status_code, 401)
        ok = self._request(
            app,
            "GET",
            "/arachne/viewpoint/acquire",
            headers={"Authorization": "Bearer exe-tok"},
        )
        self.assertEqual(ok.status_code, 200)
        self.assertEqual(ok.content, b"/arachne/viewpoint/acquire")


if __name__ == "__main__":
    unittest.main()
