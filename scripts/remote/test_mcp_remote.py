#!/usr/bin/env python3
"""Remote MCP / JSON-endpoint smoke test for an Arachne robot.

Run this on ANY host (Windows: ``py test_mcp_remote.py ...``) against the
robot's aggregate MCP listener (default port 8123). Pure standard library --
no pip installs needed.

What it checks:
  1. /health liveness probe answers 200 WITHOUT any token and reports the
     aggregate service id ("arachne_mcp") -- the plain "is the MCP up?"
     probe; HEAD is supported as well.
  2. JSON endpoints reject requests without a bearer token (401).
  3. JSON acquire/status accepts the execution token (read-only, NO motion).
  4. MCP protocol path accepts a request whose Host header is the robot's
     real LAN IP (the DNS-rebinding / Host-allowlist regression check).
  5. MCP protocol path behavior for a FORGED Host header:
       - allowlist ON  -> 421 (rejected)
       - allowlist OFF -> 200 (LAN open by design)
     Use --expect allowlist|open to enforce; default just reports.

Safety: this script only issues read-only calls. It never calls
acquire_chain or any motion path.

Examples:
  py test_mcp_remote.py --robot-ip 192.168.1.50
  py test_mcp_remote.py --robot-ip 192.168.1.50 ^
      --planner-token AAA --execution-token BBB --expect allowlist
"""

from __future__ import annotations

import argparse
import http.client
import json
import sys

INIT_BODY = json.dumps(
    {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "initialize",
        "params": {
            "protocolVersion": "2025-06-18",
            "capabilities": {},
            "clientInfo": {"name": "arachne-remote-test", "version": "1.0"},
        },
    }
)

HEALTH_PATH = "/health"
PLAN_PATH = "/arachne/viewpoint/plan"
STATUS_PATH = "/arachne/json/viewpoint/acquire/status"
# Service id returned by the aggregate listener's /health probe (matches
# arachne_mcp.mcp_servers; standalone debug servers report their own names).
HEALTH_SERVICE = "arachne_mcp"

results: list[tuple[str, str, str]] = []


def request(host: str, port: int, method: str, path: str,
            headers: dict[str, str], body: bytes | None = None,
            timeout: float = 10.0) -> tuple[int, str]:
    conn = http.client.HTTPConnection(host, port, timeout=timeout)
    try:
        conn.request(method, path, body=body, headers=headers)
        resp = conn.getresponse()
        return resp.status, resp.read(4096).decode("utf-8", "replace")
    finally:
        conn.close()


def record(name: str, ok: bool | None, detail: str) -> None:
    tag = {True: "PASS", False: "FAIL", None: "SKIP"}[ok]
    results.append((tag, name, detail))
    print(f"[{tag}] {name}: {detail}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Arachne remote MCP smoke test")
    parser.add_argument("--robot-ip", required=True, help="robot LAN IP")
    parser.add_argument("--port", type=int, default=8123)
    parser.add_argument("--planner-token", default="")
    parser.add_argument("--execution-token", default="")
    parser.add_argument("--expect", choices=("open", "allowlist"), default="",
                        help="enforce forged-Host behavior; default: report only")
    parser.add_argument("--forged-host", default="10.255.255.1",
                        help="Host header for the forged-host probe")
    args = parser.parse_args()

    robot = f"{args.robot_ip}:{args.port}"
    print(f"Target: {robot}\n")

    # 1. /health liveness probe: token-free "is the MCP up?" check.
    try:
        status, body = request(args.robot_ip, args.port, "GET", HEALTH_PATH, {})
        ok = False
        if status == 200:
            try:
                payload = json.loads(body)
            except ValueError:
                detail = f"HTTP 200 but non-JSON body: {body[:120]!r}"
            else:
                detail = f"HTTP {status}, body: {body[:120]!r}"
                ok = (payload.get("status") == "ok"
                      and payload.get("service") == HEALTH_SERVICE)
        else:
            detail = f"HTTP {status} (expected 200)"
        record("health probe (token-free GET)", ok, detail)
    except OSError as exc:
        record("health probe (token-free GET)", False, f"connection error: {exc}")

    try:
        status, _ = request(args.robot_ip, args.port, "HEAD", HEALTH_PATH, {})
        record("health probe (HEAD)", status == 200,
               f"HTTP {status} (expected 200)")
    except OSError as exc:
        record("health probe (HEAD)", False, f"connection error: {exc}")

    # 2. JSON endpoint must reject token-less requests.
    try:
        status, _ = request(args.robot_ip, args.port, "GET", STATUS_PATH, {})
        record("json status without token", status == 401,
               f"HTTP {status} (expected 401)")
    except OSError as exc:
        record("json status without token", False, f"connection error: {exc}")

    # 3. JSON endpoint with execution token (read-only status).
    if args.execution_token:
        try:
            status, body = request(
                args.robot_ip, args.port, "GET", STATUS_PATH,
                {"Authorization": f"Bearer {args.execution_token}"},
            )
            record("json status with execution token", status == 200,
                   f"HTTP {status}, body head: {body[:120]!r}")
        except OSError as exc:
            record("json status with execution token", False,
                   f"connection error: {exc}")
    else:
        record("json status with execution token", None,
               "no --execution-token given")

    # 4./5. MCP protocol Host-header checks need the planner token to pass
    # our bearer layer and reach the SDK's Host check.
    if not args.planner_token:
        record("mcp Host check (correct LAN IP)", None, "no --planner-token given")
        record("mcp Host check (forged IP)", None, "no --planner-token given")
    else:
        auth = {"Authorization": f"Bearer {args.planner_token}",
                "Content-Type": "application/json",
                "Accept": "application/json, text/event-stream"}

        # 4. Correct Host header (the robot's own LAN IP) must be accepted.
        try:
            status, _ = request(args.robot_ip, args.port, "POST", PLAN_PATH,
                                auth, INIT_BODY.encode())
            record("mcp Host check (correct LAN IP)",
                   status in (200, 400), f"HTTP {status} (expected 200)")
        except OSError as exc:
            record("mcp Host check (correct LAN IP)", False,
                   f"connection error: {exc}")

        # 5. Forged Host header: 421 when the allowlist is on, 200 when off.
        try:
            status, _ = request(args.robot_ip, args.port, "POST", PLAN_PATH,
                                {**auth, "Host": args.forged_host},
                                INIT_BODY.encode())
            if status == 421:
                mode = "allowlist ON"
            elif status in (200, 400):
                mode = "allowlist OFF (LAN open)"
            else:
                mode = f"unexpected HTTP {status}"
            if args.expect == "allowlist":
                record("mcp Host check (forged IP)", status == 421,
                       f"{mode}; expected 421")
            elif args.expect == "open":
                record("mcp Host check (forged IP)", status in (200, 400),
                       f"{mode}; expected non-421")
            else:
                record("mcp Host check (forged IP)", True, mode)
        except OSError as exc:
            record("mcp Host check (forged IP)", False, f"connection error: {exc}")

    failed = sum(1 for tag, _, _ in results if tag == "FAIL")
    skipped = sum(1 for tag, _, _ in results if tag == "SKIP")
    print(f"\nSummary: {len(results) - failed - skipped} passed, "
          f"{failed} failed, {skipped} skipped")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
