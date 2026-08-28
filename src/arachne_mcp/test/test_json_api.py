"""Offline unit tests for the plain-JSON MCP overlay (arachne_mcp.json_api).

Pure stdlib + starlette (no ROS / HTTP listener / hardware). Exercises the
handler mapping, argument coercion, structured error mapping, and the
``success:false`` detection on the execution endpoints.

Run from the workspace root with install/setup.bash sourced:

    python3 -m unittest discover -s src/arachne_mcp/test -p "test_*.py"
"""

# Workspace packages resolve at runtime via install/setup.bash (colcon); the
# LSP cannot see them, same as the server modules.
# pyright: reportMissingImports=false
from __future__ import annotations

import json
import unittest

from arachne_mcp.json_api import (
    ACQUIRE_JSON_PREFIX,
    JsonApi,
    PLAN_JSON_PREFIX,
    _ENDPOINTS,
    _require,
)


class _FakePlannerApi:
    """Minimal stand-in for ViewpointPlannerApi (returns JSON strings)."""

    def __init__(self) -> None:
        self.planned = []

    def plan_viewpoints(self, **kwargs) -> str:
        self.planned.append(kwargs)
        return json.dumps({"plan_id": "p1", "num_viewpoints": kwargs["num_viewpoints"]})

    def list_reachable_viewpoints(self, plan_id: str) -> str:
        if plan_id == "missing":
            raise LookupError(f"plan {plan_id!r} not found")
        return json.dumps({"plan_id": plan_id, "reachable_count": 1})


class _FakeExecutionNode:
    """Minimal stand-in for ExecutionMcpNode."""

    def __init__(self, reject: bool = False) -> None:
        self.reject = reject

    def acquire_chain(self, plan_id: str, indices="all"):
        if self.reject:
            return json.dumps(
                {"success": False, "message": "motion_enabled:=false rejected"},
                ensure_ascii=False,
            )
        return json.dumps(
            {
                "success": True,
                "plan_id": plan_id,
                "run_id": "r1",
                "num_viewpoints": 2,
            },
            ensure_ascii=False,
        )

    def get_acquire_status(self) -> str:
        return json.dumps({"busy": True, "completed": 1}, ensure_ascii=False)

    def cancel_acquire(self) -> str:
        return json.dumps(
            {"success": True, "cancelled": True, "points_completed": 1},
            ensure_ascii=False,
        )


class JsonApiHandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.api = JsonApi(_FakePlannerApi(), _FakeExecutionNode())

    def test_endpoint_table_covers_all_five_tools(self) -> None:
        paths = set(_ENDPOINTS)
        self.assertEqual(
            paths,
            {
                f"{PLAN_JSON_PREFIX}/plan_viewpoints",
                f"{PLAN_JSON_PREFIX}/list_reachable_viewpoints",
                f"{ACQUIRE_JSON_PREFIX}/acquire_chain",
                f"{ACQUIRE_JSON_PREFIX}/status",
                f"{ACQUIRE_JSON_PREFIX}/cancel",
            },
        )

    def test_require_raises_on_missing(self) -> None:
        with self.assertRaises(ValueError):
            _require({}, "target_xyz")

    def test_plan_viewpoints_defaults_and_result(self) -> None:
        result = self.api.plan_viewpoints({"target_xyz": [0.3, 0.0, 0.2]})
        self.assertEqual(result["plan_id"], "p1")
        called = self.api._api.planned[0]
        self.assertEqual(called["num_viewpoints"], 32)  # default applied
        self.assertEqual(called["strategy"], "fibonacci_sphere")
        self.assertIsNone(called["up_vector"])

    def test_plan_viewpoints_requires_target(self) -> None:
        with self.assertRaises(ValueError):
            self.api.plan_viewpoints({})

    def test_list_reachable_viewpoints_missing_plan(self) -> None:
        with self.assertRaises(LookupError):
            self.api.list_reachable_viewpoints({"plan_id": "missing"})

    def test_acquire_chain_success_returns_result(self) -> None:
        result = self.api.acquire_chain({"plan_id": "p1", "indices": [0, 1]})
        self.assertTrue(result["success"])

    def test_acquire_chain_rejection_surfaces_as_error(self) -> None:
        node = _FakeExecutionNode(reject=True)
        api = JsonApi(_FakePlannerApi(), node)
        with self.assertRaises(RuntimeError) as ctx:
            api.acquire_chain({"plan_id": "p1"})
        self.assertIn("motion_enabled", str(ctx.exception))

    def test_status_and_cancel_pass_through(self) -> None:
        self.assertEqual(self.api.status({}), {"busy": True, "completed": 1})
        self.assertTrue(self.api.cancel({})["cancelled"])


if __name__ == "__main__":
    unittest.main()
