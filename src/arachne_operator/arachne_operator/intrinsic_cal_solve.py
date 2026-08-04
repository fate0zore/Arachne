#!/usr/bin/env python3
"""离线求解 Gemini335 color 相机内参（复用 AprilTag 手眼标定板采集图）。

输入：teach_panel「Camera Cal」采集目录（默认 recordings/calibration/intrinsics），
包含 frame_*.png 与 capture_manifest.json（板参数）。
流程：板参考图映射 tag 物理 3D 角点 -> 逐帧 pupil_apriltags 检测 ->
      cv2.calibrateCamera 求解 K 与畸变（plumb_bob，5 参数）。
输出：报告 JSON（默认 <dir>/intrinsics_result_<时间戳>.json）+ 控制台摘要。
验收：RMS < 0.5 px（文档 hand_eye_calibration_next_steps 阶段 A 步骤 3）。
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from arachne_operator.repo_paths import root_dir

# 检测器参数与 apriltag_hand_eye_calibrator 保持一致（refine_edges 提供亚像素角点）
DETECTOR_DEFAULTS = {
    "nthreads": 4,
    "quad_decimate": 1.0,
    "quad_sigma": 0.0,
    "refine_edges": True,
    "decode_sharpening": 0.25,
}

MIN_USABLE_FRAMES = 10  # 建议：至少 10 帧可用才可信
HARD_MIN_FRAMES = 3  # cv2.calibrateCamera 硬性下限
MIN_CORNERS = 4  # 单帧最少角点数（1 个 tag = 4 角点）
ACCEPT_RMS_PX = 0.5  # 验收阈值

# 历史归档默认路径（相对工作区根）：记录旧猜测参数 + 历次标定结果，供后续测试兜底
HISTORY_DEFAULT = "config/gemini335_intrinsics/intrinsics_history.json"

# 旧参数：gemini335_v4l2_node._camera_info 的默认猜测（camera_fx=0 时 fx=fy=width*0.9）
LEGACY_CAMERA_MATRIX = [[576.0, 0.0, 319.5], [0.0, 576.0, 239.5], [0.0, 0.0, 1.0]]
LEGACY_DIST_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]


def _make_detector(families: str):
    from pupil_apriltags import Detector

    return Detector(families=families, **DETECTOR_DEFAULTS)


def load_board_object_corners(detector, board: dict) -> dict[int, np.ndarray]:
    """从板参考图检测每个 tag 的物理 3D 角点（板中心为原点）。

    与 apriltag_hand_eye_calibrator._load_apriltag_board_reference 同法：
    像素按实测 board_width_m/board_height_m 映射，不依赖 tag_size/tag_pitch 假设，
    避免打印缩放导致整板系统性误差。
    """
    path = Path(board["board_image_path"]).expanduser()
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"board reference image not readable: {path}")
    detections = list(detector.detect(image, estimate_tag_pose=False))
    if not detections:
        raise RuntimeError(f"no tags detected in board reference: {path}")
    height, width = image.shape[:2]
    scale_x = float(board["board_width_m"]) / width
    scale_y = float(board["board_height_m"]) / height
    corners: dict[int, np.ndarray] = {}
    for det in detections:
        px = np.asarray(det.corners, dtype=np.float64)
        physical = np.zeros((4, 3), dtype=np.float64)
        physical[:, 0] = (px[:, 0] - width * 0.5) * scale_x
        physical[:, 1] = (height * 0.5 - px[:, 1]) * scale_y
        corners[int(det.tag_id)] = physical
    return corners


def solve_intrinsics(cal_dir: Path, detector, max_outlier_px: float = 2.0) -> dict:
    """对采集目录求解相机内参，返回报告 dict（不落盘）。

    max_outlier_px>0 时做角点级剔除：迭代剔除重投影误差超阈值的
    outlier 角点（运动模糊/反光边缘的坏 tag 角点），最多 4 轮。
    """
    manifest_path = cal_dir / "capture_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"no capture_manifest.json in {cal_dir}")
    manifest = json.loads(manifest_path.read_text())
    if not isinstance(manifest, list) or not manifest:
        raise ValueError(f"manifest empty or malformed: {manifest_path}")
    board = manifest[0]["board"]
    board_corners = load_board_object_corners(detector, board)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    per_frame: list[dict] = []
    image_size: tuple[int, int] | None = None
    for entry in manifest:
        gray = cv2.imread(str(cal_dir / entry["file"]), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            per_frame.append(
                {"file": entry["file"], "corners": 0, "used": False, "reason": "unreadable"}
            )
            continue
        if image_size is None:
            image_size = (int(gray.shape[1]), int(gray.shape[0]))
        detections = list(detector.detect(gray, estimate_tag_pose=False))
        obj: list[np.ndarray] = []
        img: list[np.ndarray] = []
        for det in detections:
            tag_id = int(det.tag_id)
            corners = board_corners.get(tag_id)
            if corners is None:
                continue  # 背景 tag，不在板参考内
            obj.extend(corners)
            # 物理角点与图像角点均为 pupil-apriltags 同一 corners 顺序，按序对应
            img.extend(np.asarray(det.corners, dtype=np.float64))
        if len(img) < MIN_CORNERS:
            per_frame.append(
                {
                    "file": entry["file"],
                    "corners": len(img),
                    "used": False,
                    "reason": f"<{MIN_CORNERS} corners",
                }
            )
            continue
        object_points.append(np.asarray(obj, dtype=np.float32))
        image_points.append(np.asarray(img, dtype=np.float32))
        per_frame.append({"file": entry["file"], "corners": len(img), "used": True})

    if image_size is None:
        raise RuntimeError("no readable frames in calibration directory")
    if len(object_points) < HARD_MIN_FRAMES:
        raise RuntimeError(
            f"usable frames {len(object_points)} < {HARD_MIN_FRAMES}; cannot calibrate"
        )

    rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )

    removed_total = 0
    refine_rounds = 0
    if max_outlier_px > 0:
        kept_obj, kept_img = list(object_points), list(image_points)
        for _ in range(4):
            rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(
                kept_obj, kept_img, image_size, None, None
            )
            new_obj, new_img = [], []
            bad = 0
            for o, i, rv, tv in zip(kept_obj, kept_img, rvecs, tvecs):
                projected, _ = cv2.projectPoints(o, rv, tv, k, dist)
                err = np.linalg.norm(projected.reshape(-1, 2) - i, axis=1)
                keep = err < max_outlier_px
                bad += int(np.sum(~keep))
                new_obj.append(o[keep])
                new_img.append(i[keep])
            kept_obj, kept_img = new_obj, new_img
            removed_total += bad
            refine_rounds += 1
            if bad == 0:
                break
        # 以剔除后的点集做最终求解
        rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(
            kept_obj, kept_img, image_size, None, None
        )
        object_points, image_points = kept_obj, kept_img

    for i, frame_info in enumerate(per_frame):
        if not frame_info["used"]:
            continue
        projected, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], k, dist)
        error = np.linalg.norm(projected.reshape(-1, 2) - image_points[i], axis=1)
        frame_info["reproj_px"] = float(np.mean(error)) if len(error) else 0.0
        frame_info["reproj_max_px"] = float(np.max(error)) if len(error) else 0.0

    return {
        "created": datetime.now().isoformat(timespec="seconds"),
        "source_dir": str(cal_dir),
        "image_size": list(image_size),
        "frames_total": len(manifest),
        "frames_used": len(object_points),
        "points_total": sum(len(o) for o in object_points),
        "points_removed": removed_total,
        "refine_rounds": refine_rounds,
        "rms_px": float(rms),
        "accepted": float(rms) < ACCEPT_RMS_PX,
        "camera_matrix": k.tolist(),
        "dist_coeffs": dist.flatten().tolist(),
        "per_frame": per_frame,
        "board": board,
    }


def archive_to_history(report: dict, history_path: Path) -> None:
    """把本次标定结果追加到历史档案，并更新 current 指针（幂等：同报告不重复追加）。

    档案结构：{"format": ..., "entries": [...], "current": entry_id}
    entries 首条为 legacy 猜测参数（未标定），之后每次求解追加一条。
    """
    if history_path.exists():
        history = json.loads(history_path.read_text())
        entries = history.get("entries", [])
    else:
        history = {
            "format": "arachne_gemini335_intrinsics_history_v1",
            "sensor": "gemini335 color",
            "image_size": report["image_size"],
        }
        entries = [
            {
                "id": "legacy_default",
                "saved_at": datetime.now().isoformat(timespec="seconds"),
                "source": "gemini335_v4l2_node._camera_info default (width*0.9 guess)",
                "source_dir": "",
                "image_size": report["image_size"],
                "camera_matrix": LEGACY_CAMERA_MATRIX,
                "dist_coeffs": LEGACY_DIST_COEFFS,
                "rms_px": None,
                "accepted": None,
                "notes": "未标定的旧猜测内参（launch 默认 camera_fx=0），作为历史兜底",
            }
        ]
        history["entries"] = entries

    report_name = Path(report.get("report_path", "")).stem
    entry_id = report_name.replace("intrinsics_result_", "")
    if any(e["id"] == entry_id for e in entries):
        history["current"] = entry_id
        history_path.write_text(json.dumps(history, indent=2))
        return

    entries.append(
        {
            "id": entry_id,
            "saved_at": report["created"],
            "source": "intrinsic_cal_solve",
            "source_dir": report["source_dir"],
            "image_size": report["image_size"],
            "camera_matrix": report["camera_matrix"],
            "dist_coeffs": report["dist_coeffs"],
            "rms_px": report["rms_px"],
            "accepted": report["accepted"],
            "notes": (
                "RMS>0.5px（纸板变形/检测噪声所致）；用户接受，后续可重采覆盖"
                if not report["accepted"]
                else "验收通过"
            ),
        }
    )
    history["current"] = entry_id
    history_path.parent.mkdir(parents=True, exist_ok=True)
    history_path.write_text(json.dumps(history, indent=2))


def _print_report(report: dict) -> None:
    print("=" * 64)
    print(f"camera intrinsic calibration report  ({report['source_dir']})")
    print(f"  image size   : {report['image_size'][0]} x {report['image_size'][1]}")
    print(f"  frames used  : {report['frames_used']} / {report['frames_total']}")
    if report.get("points_removed"):
        print(
            f"  outlier corns: {report['points_removed']} removed "
            f"({report['refine_rounds']} rounds, {report['points_total']} kept)"
        )
    verdict = "PASS" if report["accepted"] else "FAIL"
    print(
        f"  RMS reproj   : {report['rms_px']:.4f} px  "
        f"(accept < {ACCEPT_RMS_PX} px)  -> {verdict}"
    )
    k = report["camera_matrix"]
    print(f"  fx           : {k[0][0]:.4f}")
    print(f"  fy           : {k[1][1]:.4f}")
    print(f"  cx           : {k[0][2]:.4f}")
    print(f"  cy           : {k[1][2]:.4f}")
    d = report["dist_coeffs"]
    print(
        f"  distortion   : k1={d[0]:.6f} k2={d[1]:.6f} "
        f"p1={d[2]:.6f} p2={d[3]:.6f} k3={d[4]:.6f}"
    )
    worst = sorted(
        (f for f in report["per_frame"] if f.get("used")),
        key=lambda f: f.get("reproj_px", 0.0),
        reverse=True,
    )[:3]
    if worst:
        print("  worst frames (mean reproj px):")
        for frame_info in worst:
            print(
                f"    {frame_info['file']}: "
                f"mean={frame_info['reproj_px']:.3f} max={frame_info['reproj_max_px']:.3f}"
            )
    print("=" * 64)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Solve Gemini335 color camera intrinsics from Camera Cal captures"
    )
    parser.add_argument(
        "--dir",
        default="recordings/calibration/intrinsics",
        help="capture directory with frame_*.png and capture_manifest.json "
        "(relative paths resolve under workspace root)",
    )
    parser.add_argument(
        "--out",
        default="",
        help="report JSON path (default: <dir>/intrinsics_result_<timestamp>.json)",
    )
    parser.add_argument(
        "--max-outlier-px",
        type=float,
        default=2.0,
        help="reject corner points with reproj error above this (0 disables)",
    )
    parser.add_argument(
        "--history",
        default=HISTORY_DEFAULT,
        help="history JSON path (relative paths resolve under workspace root)",
    )
    parser.add_argument(
        "--no-archive", action="store_true", help="do not append result to history JSON"
    )
    parser.add_argument(
        "--dry-run-check", action="store_true", help="verify imports/entrypoint only"
    )
    args = parser.parse_args(argv)

    if args.dry_run_check:
        _make_detector("tagStandard41h12")  # 验证 pupil_apriltags 可用
        print("intrinsic_cal_solve dry-run check ok")
        return 0

    cal_dir = Path(args.dir).expanduser()
    if not cal_dir.is_absolute():
        cal_dir = root_dir() / cal_dir

    try:
        board = json.loads((cal_dir / "capture_manifest.json").read_text())[0]["board"]
    except (FileNotFoundError, IndexError, json.JSONDecodeError) as exc:
        print(f"intrinsic_cal_solve: cannot read manifest in {cal_dir}: {exc}", file=sys.stderr)
        return 2
    detector = _make_detector(board["tag_family"])
    report = solve_intrinsics(cal_dir, detector, max_outlier_px=args.max_outlier_px)

    out_path = args.out
    if not out_path:
        name = f"intrinsics_result_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        out_path = str(cal_dir / name)
    report["report_path"] = str(Path(out_path).expanduser())
    Path(out_path).expanduser().write_text(json.dumps(report, indent=2))
    print(f"report saved: {report['report_path']}")
    if not args.no_archive:
        history_path = Path(args.history).expanduser()
        if not history_path.is_absolute():
            history_path = root_dir() / history_path
        archive_to_history(report, history_path)
        print(f"archived to  : {history_path}")
    _print_report(report)
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    sys.exit(main())
