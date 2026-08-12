"""Offline unit tests for teach_panel viewpoint-plan JSON parsing (pure stdlib).

Run from the workspace root with install/setup.bash sourced:
    python3 -m unittest discover -s src/arachne_operator/test -p "test_*.py"
"""

import json
import math
import tempfile
import unittest
from pathlib import Path

from arachne_operator.teach_panel import _parse_viewpoint_plan_file


def _sample_payload() -> dict:
    return {
        "planning_stamp": "2026-08-10T12:00:00.000",
        "request": {
            "target_xyz_base": [0.3, 0.0, 0.2],
            "strategy": "fibonacci_sphere",
            "radius_m": 0.5,
            "num_viewpoints": 2,
            "filter_ik_reachable": True,
            "up_vector_base": [0.0, 0.0, 1.0],
        },
        "server_message": "planned 2 viewpoints",
        "num_viewpoints": 2,
        "ik_reachable_count": 1,
        "viewpoints": [
            {
                "stamp": {"sec": 1, "nanosec": 2},
                "camera_xyz_base": [0.3, 0.5, 0.2],
                "camera_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                "tool0_xyz_base": [0.1, 0.2, 0.3],
                "tool0_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                "ik_reachable": True,
                "ik_joints": [0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0],
            },
            {
                "stamp": {"sec": 1, "nanosec": 3},
                "camera_xyz_base": [0.3, -0.5, 0.2],
                "camera_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                "tool0_xyz_base": [0.0, 0.0, 0.0],
                "tool0_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
                "ik_reachable": False,
                "ik_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        ],
    }


class ParseViewpointPlanFileTest(unittest.TestCase):
    def _write(self, payload: dict) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "plan.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_valid_structure(self) -> None:
        data = _parse_viewpoint_plan_file(self._write(_sample_payload()))
        self.assertEqual(data["num_viewpoints"], 2)
        self.assertEqual(data["ik_reachable_count"], 1)
        self.assertEqual(data["request"]["radius_m"], 0.5)
        self.assertEqual(data["planning_stamp"], "2026-08-10T12:00:00.000")
        self.assertEqual(len(data["viewpoints"]), 2)

    def test_ik_joints_radians_to_degrees(self) -> None:
        data = _parse_viewpoint_plan_file(self._write(_sample_payload()))
        joints_deg = data["viewpoints"][0]["ik_joints_deg"]
        self.assertEqual(len(joints_deg), 6)
        self.assertAlmostEqual(joints_deg[1], 90.0)
        self.assertAlmostEqual(data["viewpoints"][0]["ik_joints_rad"][1], math.pi / 2)

    def test_unreachable_marked(self) -> None:
        data = _parse_viewpoint_plan_file(self._write(_sample_payload()))
        self.assertTrue(data["viewpoints"][0]["ik_reachable"])
        self.assertFalse(data["viewpoints"][1]["ik_reachable"])

    def test_missing_viewpoints_raises(self) -> None:
        payload = _sample_payload()
        del payload["viewpoints"]
        with self.assertRaises(KeyError):
            _parse_viewpoint_plan_file(self._write(payload))

    def test_wrong_ik_joints_length_raises(self) -> None:
        payload = _sample_payload()
        payload["viewpoints"][0]["ik_joints"] = [0.0, 0.0, 0.0]
        with self.assertRaises(ValueError):
            _parse_viewpoint_plan_file(self._write(payload))

    def test_non_object_json_raises(self) -> None:
        with self.assertRaises(ValueError):
            _parse_viewpoint_plan_file(self._write([1, 2, 3]))

    def test_missing_file_raises(self) -> None:
        with self.assertRaises(OSError):
            _parse_viewpoint_plan_file(Path("/nonexistent/viewpoint_plan.json"))


if __name__ == "__main__":
    unittest.main()
