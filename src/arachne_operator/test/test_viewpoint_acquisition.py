"""Offline unit tests for teach_panel viewpoint acquisition serialization helpers.

Pure stdlib/numpy helpers (serialize_pcd / depth_image_to_uint16 /
decimate_cloud_points / camera_info_to_dict), duck-typed so no ROS is needed.

Run from the workspace root with install/setup.bash sourced:
    python3 -m unittest discover -s src/arachne_operator/test -p "test_*.py"
"""

import unittest
from types import SimpleNamespace

import numpy as np

from arachne_operator.teach_panel import (
    camera_info_to_dict,
    decimate_cloud_points,
    depth_image_to_uint16,
    make_viewpoint_run_id,
    pointcloud_xyz_hw,
    serialize_pcd,
)


class SerializePcdTest(unittest.TestCase):
    def test_header_and_points(self):
        points = np.array([[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]], dtype=np.float32)
        text = serialize_pcd(points)
        self.assertIn("VERSION .7", text)
        self.assertIn("FIELDS x y z", text)
        self.assertIn("SIZE 4 4 4", text)
        self.assertIn("TYPE F F F", text)
        self.assertIn("WIDTH 2", text)
        self.assertIn("HEIGHT 1", text)
        self.assertIn("POINTS 2", text)
        self.assertIn("DATA ascii", text)
        # 每行 3 个数值 + 结尾换行
        lines = text.splitlines()
        data_lines = lines[lines.index("DATA ascii") + 1 :]
        self.assertEqual(len(data_lines), 2)
        first = [float(value) for value in data_lines[0].split()]
        self.assertEqual(first, [0.0, 1.0, 2.0])

    def test_empty_points(self):
        text = serialize_pcd(np.empty((0, 3), dtype=np.float32))
        self.assertIn("POINTS 0", text)
        self.assertTrue(text.rstrip().endswith("DATA ascii"))

    def test_bad_shape_raises(self):
        with self.assertRaises(ValueError):
            serialize_pcd(np.zeros((4, 2), dtype=np.float32))
        with self.assertRaises(ValueError):
            serialize_pcd(np.zeros((4, 3, 1), dtype=np.float32))


class DepthImageTest(unittest.TestCase):
    def _depth(self, width=2, height=3, encoding="16UC1"):
        values = np.arange(width * height, dtype=np.uint16)
        return SimpleNamespace(
            encoding=encoding,
            width=width,
            height=height,
            data=values.tobytes(),
        )

    def test_uint16_roundtrip(self):
        img = self._depth()
        arr = depth_image_to_uint16(img)
        self.assertEqual(arr.shape, (3, 2))
        self.assertEqual(arr.dtype, np.uint16)
        self.assertTrue(
            np.array_equal(arr, np.arange(6, dtype=np.uint16).reshape(3, 2))
        )

    def test_wrong_encoding_raises(self):
        with self.assertRaises(ValueError):
            depth_image_to_uint16(self._depth(encoding="32FC1"))

    def test_geometry_mismatch_raises(self):
        img = self._depth()
        img.data = bytes(10)  # 与 w*h*2=12 不符
        with self.assertRaises(ValueError):
            depth_image_to_uint16(img)


class DecimateCloudTest(unittest.TestCase):
    def test_step_and_nan_filter(self):
        # 6x6 网格,保留每 2 行/列 -> 3x3=9 点;其中 (4,4),(5,5) 置 NaN/Inf
        grid = np.arange(6 * 6 * 3, dtype=np.float32).reshape(6, 6, 3)
        grid[4, 4] = np.nan
        grid[5, 5] = np.inf
        points = decimate_cloud_points(grid, step=2)
        # 取 [::2,::2] 共 9 点,其中含 (4,4) NaN -> 去掉 1,剩 8
        self.assertEqual(points.shape[1], 3)
        self.assertEqual(points.shape[0], 8)
        self.assertTrue(np.all(np.isfinite(points)))

    def test_step_lt_1_raises(self):
        with self.assertRaises(ValueError):
            decimate_cloud_points(np.zeros((2, 2, 3), dtype=np.float32), step=0)


class PointCloudParseTest(unittest.TestCase):
    def _pc(self, grid, point_step=12):
        # create_cloud_xyz32 布局:x,y,z 连续 float32
        fields = [
            SimpleNamespace(name="x", offset=0),
            SimpleNamespace(name="y", offset=4),
            SimpleNamespace(name="z", offset=8),
        ]
        height, width = grid.shape[:2]
        return SimpleNamespace(
            width=width,
            height=height,
            point_step=point_step,
            fields=fields,
            data=grid.reshape(-1, 3).tobytes(),
        )

    def test_roundtrip(self):
        grid = np.array(
            [[[0, 1, 2], [3, 4, 5]], [[6, 7, 8], [9, 10, 11]]], dtype=np.float32
        )
        out = pointcloud_xyz_hw(self._pc(grid))
        self.assertEqual(out.shape, (2, 2, 3))
        self.assertTrue(np.array_equal(out, grid))

    def test_bad_layout_raises(self):
        pc = self._pc(np.zeros((2, 2, 3), dtype=np.float32), point_step=12)
        pc.fields = [
            SimpleNamespace(name="x", offset=0),
            SimpleNamespace(name="z", offset=8),
        ]
        with self.assertRaises(ValueError):
            pointcloud_xyz_hw(pc)

    def test_geometry_mismatch_raises(self):
        pc = self._pc(np.zeros((2, 2, 3), dtype=np.float32))
        pc.data = bytes(3)  # 与 num*point_step=48 不符
        with self.assertRaises(ValueError):
            pointcloud_xyz_hw(pc)


class CameraInfoTest(unittest.TestCase):
    def test_roundtrip(self):
        roi = SimpleNamespace(
            x_offset=1, y_offset=2, height=10, width=20, do_rectify=True
        )
        info = SimpleNamespace(
            width=640,
            height=480,
            distortion_model="plumb_bob",
            d=[0.1, 0.0, 0.0, 0.0, 0.0],
            k=[1.0] * 9,
            r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
            p=[1.0] * 12,
            binning_x=1,
            binning_y=1,
            roi=roi,
        )
        data = camera_info_to_dict(info)
        self.assertEqual(data["width"], 640)
        self.assertEqual(data["height"], 480)
        self.assertEqual(data["distortion_model"], "plumb_bob")
        self.assertEqual(len(data["k"]), 9)
        self.assertEqual(data["roi"]["do_rectify"], True)
        # 数值/类型 JSON 可序列化
        import json

        json.dumps(data)

    def test_none(self):
        self.assertIsNone(camera_info_to_dict(None))


class ViewpointRunIdTest(unittest.TestCase):
    def test_timestamp_shape(self):
        import datetime
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_id = make_viewpoint_run_id(
                root, now=datetime.datetime(2026, 8, 21, 15, 30, 12)
            )
            self.assertEqual(run_id, "20260821_153012")

    def test_same_second_dedup(self):
        import datetime
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            now = datetime.datetime(2026, 8, 21, 15, 30, 12)
            first = make_viewpoint_run_id(root, now=now)
            (root / first).mkdir()
            second = make_viewpoint_run_id(root, now=now)
            (root / second).mkdir()
            third = make_viewpoint_run_id(root, now=now)
            self.assertEqual(
                [first, second, third],
                ["20260821_153012", "20260821_153012_2", "20260821_153012_3"],
            )

    def test_default_now(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_id = make_viewpoint_run_id(root)
            import datetime

            self.assertEqual(
                run_id[: len("20260821_153012")],
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),
            )


if __name__ == "__main__":
    unittest.main()
