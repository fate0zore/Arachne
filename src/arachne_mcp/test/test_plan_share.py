"""Offline unit tests for arachne_mcp shared plan-directory helpers.

Pure stdlib (no ROS / HTTP / hardware). Run from the workspace root with
install/setup.bash sourced:

    python3 -m unittest discover -s src/arachne_mcp/test -p "test_*.py"
"""

# Workspace packages resolve at runtime via install/setup.bash (colcon); the
# LSP cannot see them, same as the server modules.
# pyright: reportMissingImports=false
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from arachne_mcp.common import (
    plan_file_path,
    resolve_plan_share_dir,
    to_float,
    to_int,
    write_plan_file,
)


class PlanShareDirTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.share = resolve_plan_share_dir(Path(self.tmp.name), "plans")
        self.payload = {
            "planning_stamp": "2026-08-26T00:00:00.000",
            "request": {
                "target_xyz_base": [0.3, 0.0, 0.2],
                "strategy": "fibonacci_sphere",
                "radius_m": 0.5,
                "num_viewpoints": 2,
                "filter_ik_reachable": True,
                "filter_collision_free": True,
                "up_vector_base": [0.0, 0.0, 1.0],
            },
            "server_message": "ok",
            "num_viewpoints": 2,
            "viewpoints": [
                {
                    "stamp": {"sec": 1, "nanosec": 2},
                    "camera_xyz_base": [0.3, 0.5, 0.2],
                    "camera_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "tool0_xyz_base": [0.1, 0.2, 0.3],
                    "tool0_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "ik_reachable": True,
                    "ik_joints": [0.0, 1.5707963267948966, 0.0, 0.0, 0.0, 0.0],
                    "collision_free": True,
                    "collision_clearance": 0.05,
                },
                {
                    "stamp": {"sec": 1, "nanosec": 3},
                    "camera_xyz_base": [0.3, -0.5, 0.2],
                    "camera_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "tool0_xyz_base": [0.0, 0.0, 0.0],
                    "tool0_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "ik_reachable": False,
                    "ik_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "collision_free": False,
                    "collision_clearance": -0.01,
                },
            ],
        }

    def test_write_and_read_roundtrip(self) -> None:
        plan_id, path = write_plan_file(self.share, self.payload, keep=5)
        self.assertEqual(path.name, f"viewpoint_plan_{plan_id}.json")
        self.assertTrue(path.is_file())
        raw = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(raw["num_viewpoints"], 2)
        # the execution/planner side resolves the file by plan_id
        resolved = plan_file_path(self.share, plan_id)
        self.assertEqual(resolved, path)

    def test_parse_roundtrip_with_teach_panel_parser(self) -> None:
        from arachne_operator.teach_panel import _parse_viewpoint_plan_file

        plan_id, path = write_plan_file(self.share, self.payload, keep=5)
        data = _parse_viewpoint_plan_file(path)
        self.assertEqual(data["num_viewpoints"], 2)
        self.assertEqual(data["reachable_count"], 1)
        self.assertAlmostEqual(data["viewpoints"][0]["ik_joints_deg"][1], 90.0)

    def test_missing_plan_id_raises_lookup_error(self) -> None:
        with self.assertRaises(LookupError):
            plan_file_path(self.share, "does_not_exist")

    def test_empty_plan_id_raises(self) -> None:
        with self.assertRaises(LookupError):
            plan_file_path(self.share, "")

    def test_path_traversal_rejected(self) -> None:
        with self.assertRaises(LookupError):
            plan_file_path(self.share, "../secret")

    def test_prune_keeps_newest(self) -> None:
        import time

        first_id, _ = write_plan_file(self.share, self.payload, keep=2)
        time.sleep(0.002)  # distinct millisecond stamps -> distinct plan_ids
        second_id, _ = write_plan_file(self.share, self.payload, keep=2)
        time.sleep(0.002)
        third_id, _ = write_plan_file(self.share, self.payload, keep=2)
        self.assertFalse(
            (self.share / f"viewpoint_plan_{first_id}.json").exists(),
            "oldest plan should be pruned when keep=2",
        )
        self.assertTrue((self.share / f"viewpoint_plan_{second_id}.json").exists())
        self.assertTrue((self.share / f"viewpoint_plan_{third_id}.json").exists())

    def test_converters(self) -> None:
        self.assertAlmostEqual(to_float("1.5", "x"), 1.5)
        self.assertEqual(to_int("3", "n"), 3)
        with self.assertRaises(ValueError):
            to_float("abc", "x")
        with self.assertRaises(ValueError):
            to_int("3.7", "n")


if __name__ == "__main__":
    unittest.main()
