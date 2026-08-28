"""Offline unit tests for the execution MCP motion gate and chain plumbing.

Constructs a real ExecutionMcpNode WITHOUT spinning it (no hardware I/O):
- the motion gate rejects acquire_chain with motion disabled (default);
- after enabling the gate, a missing plan_id is rejected cleanly;
- cancel_acquire with no active chain is a clean no-op;
- index resolution semantics.

Run from the workspace root with install/setup.bash sourced (colcon-built
packages required):
    python3 -m unittest discover -s src/arachne_mcp/test -p "test_*.py"
"""

# ROS packages resolve at runtime via install/setup.bash; the workspace LSP
# cannot see them, same as the server modules.
# pyright: reportMissingImports=false
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import rclpy
from arachne_mcp.execution_mcp import ExecutionMcpNode
from rclpy.parameter import Parameter


def _init_args(share_dir: Path) -> list[str]:
    return [
        "--ros-args",
        "-p",
        "teach_config_autoload:=false",
        "-p",
        'autostart_managed_processes:=""',
        "-p",
        f"runtime_log_root:={share_dir / 'log'}",
        "-p",
        f"viewpoint_plan_share_dir:={share_dir / 'plans'}",
        "-p",
        "aubo_sdk_rpc_timeout_sec:=0.2",
        "-p",
        "aubo_sdk_ip:=127.0.0.1",
    ]


class ExecutionMotionGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory()
        cls.share_dir = Path(cls.tmp.name)
        rclpy.init(args=_init_args(cls.share_dir))
        cls.node = ExecutionMcpNode()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cls.tmp.cleanup()

    def _set_gate(self, motion: bool, confirm: bool) -> None:
        self.node.set_parameters(
            [
                Parameter("motion_enabled", value=motion),
                Parameter("confirm_agent_motion", value=confirm),
            ]
        )

    def test_gate_closed_by_default(self) -> None:
        self._set_gate(False, False)
        result = json.loads(self.node.acquire_chain("whatever"))
        self.assertFalse(result["success"])
        self.assertIn("motion_enabled:=false", result["message"])

    def test_gate_requires_both_flags(self) -> None:
        self._set_gate(True, False)
        result = json.loads(self.node.acquire_chain("whatever"))
        self.assertFalse(result["success"])
        self.assertIn("confirm_agent_motion:=false", result["message"])
        self._set_gate(False, True)
        result = json.loads(self.node.acquire_chain("whatever"))
        self.assertFalse(result["success"])
        self.assertIn("motion_enabled:=false", result["message"])

    def test_gate_open_but_plan_missing(self) -> None:
        self._set_gate(True, True)
        result = json.loads(self.node.acquire_chain("no_such_plan"))
        self.assertFalse(result["success"])
        self.assertIn("not found", result["message"])

    def test_indices_resolution(self) -> None:
        self.assertEqual(ExecutionMcpNode._resolve_indices("all", 5), [0, 1, 2, 3, 4])
        self.assertEqual(ExecutionMcpNode._resolve_indices([2, 0, 2], 5), [2, 0])
        self.assertEqual(ExecutionMcpNode._resolve_indices("1,3", 5), [1, 3])
        with self.assertRaises(ValueError):
            ExecutionMcpNode._resolve_indices([7], 5)
        with self.assertRaises(ValueError):
            ExecutionMcpNode._resolve_indices([], 5)

    def test_cancel_with_no_active_chain(self) -> None:
        result = json.loads(self.node.cancel_acquire())
        self.assertFalse(result["success"])
        self.assertIn("no active acquire chain", result["message"])

    def test_get_status_before_any_chain(self) -> None:
        result = json.loads(self.node.get_acquire_status())
        self.assertFalse(result["busy"])
        self.assertIn("no acquire chain has run", result["message"])


if __name__ == "__main__":
    unittest.main()
