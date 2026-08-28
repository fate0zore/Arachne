"""Offline unit tests for the MCP Host-header allowlist (transport security).

Pure stdlib + MCP SDK (no ROS / hardware). Exercises:
- ``parse_allowed_hosts`` comma/space/empty parsing;
- the actual SDK behavior through ``build_fastmcp`` + ``streamable_http_app``:
  allowlist off -> LAN-IP Host accepted; allowlist on -> whitelisted Host
  accepted, foreign LAN-IP Host rejected with 421 (the regression this
  guards: the SDK's implicit localhost rebinding protection).

Run from the workspace root with install/setup.bash sourced:

    python3 -m unittest discover -s src/arachne_mcp/test -p "test_*.py"
"""

# Workspace packages resolve at runtime via install/setup.bash (colcon); the
# LSP cannot see them, same as the server modules.
# pyright: reportMissingImports=false
from __future__ import annotations

import unittest

import anyio
import httpx

from arachne_mcp.common import (
    build_fastmcp,
    log_requests,
    parse_allowed_hosts,
    sanitize_ros_args,
    transport_security_for,
)


class SanitizeRosArgsTest(unittest.TestCase):
    """Regression: launch command strings expand unset ${VAR:-} into
    ``-p name:=``, which rcl rejects at rclpy.init."""

    def test_drops_empty_override(self) -> None:
        self.assertEqual(
            sanitize_ros_args(
                ["--ros-args", "-p", "mcp_allowed_hosts:=",
                 "-p", "mcp_planner_token:=arachne"]
            ),
            ["--ros-args", "-p", "mcp_planner_token:=arachne"],
        )

    def test_keeps_valued_overrides_and_other_args(self) -> None:
        self.assertEqual(
            sanitize_ros_args(
                ["--ros-args", "--param", "motion_enabled:=false", "-d", "42"]
            ),
            ["--ros-args", "--param", "motion_enabled:=false", "-d", "42"],
        )


class LogRequestsTest(unittest.TestCase):
    """Default mode: request line + body, response status; response bodies
    only for non-2xx (concise). Headers/response-bodies are opt-in and the
    Authorization value is always redacted."""

    def _run(self, handler, headers=None, **kwargs) -> list[str]:
        import httpx

        lines: list[str] = []

        async def _main() -> httpx.Response:
            app = log_requests(handler, lines.append, **kwargs)
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://t:1"
            ) as client:
                return await client.post("/x", json={"ping": 1}, headers=headers)

        response = anyio.run(_main)
        lines.append(f"__status__={response.status_code}")
        return lines

    @staticmethod
    def _echo_handler(status: int = 200):
        async def handler(scope, receive, send):
            body = b""
            while True:
                message = await receive()
                body += message.get("body", b"")
                if not message.get("more_body", False):
                    break
            payload = b'{"ok":true,"echo":' + body + b"}"
            await send({"type": "http.response.start", "status": status,
                        "headers": [(b"content-type", b"application/json"),
                                    (b"content-length", str(len(payload)).encode())]})
            await send({"type": "http.response.body", "body": payload})

        return handler

    def test_default_is_concise(self) -> None:
        lines = self._run(self._echo_handler(), headers={"Authorization": "Bearer secret-token"})
        joined = "\n".join(lines)
        self.assertIn("request POST /x", joined)
        self.assertNotIn("headers:", joined)          # headers opt-in
        self.assertNotIn("secret-token", joined)      # token never logged
        self.assertIn("request body (10B): {\"ping\":1}", joined)
        self.assertIn("response 200 for POST /x", joined)
        self.assertNotIn("response body", joined)     # 2xx bodies suppressed
        self.assertIn("__status__=200", joined)

    def test_error_body_logged_by_default(self) -> None:
        lines = self._run(self._echo_handler(status=500))
        joined = "\n".join(lines)
        self.assertIn("response 500 for POST /x", joined)
        self.assertIn("response body", joined)

    def test_verbose_mode_logs_headers_and_bodies(self) -> None:
        lines = self._run(
            self._echo_handler(),
            headers={"Authorization": "Bearer secret-token"},
            include_headers=True,
            include_response_bodies=True,
        )
        joined = "\n".join(lines)
        self.assertIn("authorization=Bearer ***redacted***", joined)
        self.assertNotIn("secret-token", joined)
        self.assertIn("response body", joined)

    def test_truncates_large_bodies(self) -> None:
        async def handler(scope, receive, send):
            while True:
                if (await receive()).get("more_body", False) is False:
                    break
            big = b"x" * 10000
            await send({"type": "http.response.start", "status": 200, "headers": []})
            await send({"type": "http.response.body", "body": big})

        lines = self._run(handler, include_response_bodies=True)
        self.assertTrue(any("truncated" in line for line in lines), lines)


class ParseAllowedHostsTest(unittest.TestCase):
    def test_empty_variants_disable(self) -> None:
        for raw in (None, "", "  ", ",,,"):
            self.assertEqual(parse_allowed_hosts(raw), [], repr(raw))

    def test_comma_split_and_trim(self) -> None:
        self.assertEqual(
            parse_allowed_hosts(" 192.168.1.50:* , localhost:* ,10.0.0.5:*"),
            ["192.168.1.50:*", "localhost:*", "10.0.0.5:*"],
        )

    def test_list_passthrough(self) -> None:
        self.assertEqual(parse_allowed_hosts(["a:*", " b:* "]), ["a:*", "b:*"])


class TransportSecuritySettingsTest(unittest.TestCase):
    def test_off_when_unconfigured(self) -> None:
        settings = transport_security_for([])
        self.assertFalse(settings.enable_dns_rebinding_protection)

    def test_on_with_derived_origins(self) -> None:
        settings = transport_security_for(["192.168.1.50:*", "localhost:*"])
        self.assertTrue(settings.enable_dns_rebinding_protection)
        self.assertEqual(settings.allowed_hosts, ["192.168.1.50:*", "localhost:*"])
        self.assertEqual(
            settings.allowed_origins,
            ["http://192.168.1.50:*", "http://localhost:*"],
        )


_INIT_REQUEST = {
    "jsonrpc": "2.0",
    "id": 1,
    "method": "initialize",
    "params": {
        "protocolVersion": "2025-06-18",
        "capabilities": {},
        "clientInfo": {"name": "test", "version": "0"},
    },
}


async def _post_initialize(mcp, host_header: str) -> httpx.Response:
    app = mcp.streamable_http_app()
    async with (
        mcp.session_manager.run(),
        httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://robot:8123"
        ) as client,
    ):
        return await client.post(
            "/mcp",
            json=_INIT_REQUEST,
            headers={
                "Host": host_header,
                "Accept": "application/json, text/event-stream",
            },
        )


class BuildFastmcpHostCheckTest(unittest.TestCase):
    """End-to-end Host-header behavior through the real SDK transport check."""

    def _make_mcp(self, allowed_hosts: list[str]):
        mcp = build_fastmcp("t", allowed_hosts, log_level="ERROR")

        @mcp.tool()
        def ping() -> str:
            return "pong"

        return mcp

    def test_allowlist_off_accepts_lan_host(self) -> None:
        response = anyio.run(
            _post_initialize, self._make_mcp([]), "192.168.1.50:8123"
        )
        self.assertEqual(response.status_code, 200)

    def test_allowlist_on_accepts_whitelisted_and_loopback(self) -> None:
        # Explicit settings replace the SDK's implicit loopback allowance:
        # local clients must have 127.0.0.1:* whitelisted too.
        # A fresh FastMCP per request: StreamableHTTPSessionManager.run()
        # can only be entered once per instance.
        mcps = [self._make_mcp(["192.168.1.50:*", "127.0.0.1:*"]) for _ in range(2)]
        for mcp, host_header in zip(
            mcps, ("192.168.1.50:8123", "127.0.0.1:8123")
        ):
            response = anyio.run(_post_initialize, mcp, host_header)
            self.assertEqual(response.status_code, 200, host_header)

    def test_allowlist_on_rejects_other_lan_hosts(self) -> None:
        for host_header in ("192.168.1.51:8123", "10.0.0.5:8123"):
            response = anyio.run(
                _post_initialize, self._make_mcp(["192.168.1.50:*"]), host_header
            )
            self.assertEqual(response.status_code, 421, host_header)



if __name__ == "__main__":
    unittest.main()
