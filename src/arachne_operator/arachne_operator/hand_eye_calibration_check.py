"""手眼标定质量门禁：离线校验任意标定 JSON，输出分级报告。

设计：
- 纯函数 + CLI，不依赖 ROS（仅 stdlib + numpy），供
  `ros2 run arachne_operator hand_eye_calibration_check <json>...` 与
  teach_panel 的 convert/apply 软告警复用。
- 每项指标按 docs/todo/hand_eye_calibration_next_steps.md 验收线分级：
  pass / warn / fail（软告警语义：fail 即建议拦截项）。
- exit code：0 = 无 fail，1 = 存在 fail（供脚本判断）。

验收线来源（文档 §四 阶段 B / §四.1 Phase 3）：
- 5 方法解 span < 0.1 m（x/y/z 各轴）
- tool0→camera ≈ (0, 0, 0.20) m ± 0.05 m（物理先验，经验值 (0.0201, 0, 0.05)）
- 固定外参一致性检验 mean 偏差 < 50 mm
- reproj_px < 1.5
- 姿态覆盖：累计相对旋转 > 180°（warn 90~180）
- 样本量 ≥ 30/次
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

# 物理先验（tool0→camera 经验值，文档 §一/§四；launch 外参即由它换算而来）
# 2026-08-04：姿态部分改为 (0,0,-90°)（原 roll=0.196/pitch=-0.024 为无来源经验值，与 URDF 及 6/18 标定矛盾）
PHYSICAL_PRIOR_XYZ = (0.0201, 0.0, 0.05)
PHYSICAL_PRIOR_RPY = (0.0, 0.0, -1.570796327)

# 阈值表：name -> (pass 上限, warn 上限, 单位说明, 显示名)
# 数值语义：value < p 为 pass，p <= value < w 为 warn，value >= w 为 fail。
# 注意姿态覆盖与样本数方向相反（越大越好），单独处理。

_PASS = "pass"
_WARN = "warn"
_FAIL = "fail"


def _grade(value: float, pass_limit: float, warn_limit: float) -> str:
    """数值越小越好的通用分级：value<pass_limit pass，<warn_limit warn，否则 fail。"""
    if value < pass_limit:
        return _PASS
    if value < warn_limit:
        return _WARN
    return _FAIL


def _inv(m: np.ndarray) -> np.ndarray:
    R, t = m[:3, :3], m[:3, 3]
    return np.block([[R.T, -R.T @ t.reshape(3, 1)], [np.zeros((1, 3)), 1.0]])


def _rotvec_deg(R: np.ndarray) -> float:
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def _candidates_translations(calibration: dict) -> list[tuple[str, bool, np.ndarray]]:
    """返回 [(method, valid, tool0->camera 平移)], 空 candidates 时回退顶层字段。"""
    cands = calibration.get("hand_eye_candidates")
    if isinstance(cands, list) and cands:
        out = []
        for c in cands:
            if not isinstance(c, dict):
                continue
            g2c = c.get("gripper_to_camera_matrix") or (c.get("gripper_to_camera") or {}).get("matrix")
            if g2c is None:
                continue
            out.append((str(c.get("method", "?")), bool(c.get("valid", True)), np.array(g2c)[:3, 3]))
        return out
    g2c = (calibration.get("gripper_to_camera") or {}).get("matrix")
    if g2c is not None:
        return [("(top-level)", True, np.array(g2c)[:3, 3])]
    return []


def _selected_solution_matrix(calibration: dict) -> np.ndarray | None:
    """选中方法的 gripper_to_camera 4x4 矩阵；回退顶层 gripper_to_camera.matrix。"""
    selected = calibration.get("selected_method", "")
    cands = calibration.get("hand_eye_candidates")
    if isinstance(cands, list):
        for c in cands:
            if isinstance(c, dict) and c.get("method") == selected:
                g2c = c.get("gripper_to_camera_matrix") or (c.get("gripper_to_camera") or {}).get("matrix")
                if g2c is not None:
                    return np.array(g2c)
    g2c = (calibration.get("gripper_to_camera") or {}).get("matrix")
    if g2c is not None:
        return np.array(g2c)
    return None


def _fixed_x_projection_mean_mm(calibration: dict, X: np.ndarray) -> float | None:
    """附录 C 固定外参一致性检验：板投影离散度 mean（mm）。无 samples 返回 None。"""
    samples = calibration.get("samples")
    if not isinstance(samples, list) or not samples:
        return None
    pts = []
    for s in samples:
        if not isinstance(s, dict):
            continue
        b2g = (s.get("base_to_gripper") or {}).get("matrix")
        t2c = (s.get("target_to_camera") or {}).get("matrix")
        if b2g is None or t2c is None:
            continue
        b2t = np.array(b2g) @ X @ _inv(np.array(t2c))
        pts.append(b2t[:3, 3])
    if not pts:
        return None
    arr = np.array(pts)
    dev = np.linalg.norm(arr - arr.mean(axis=0), axis=1)
    return float(dev.mean() * 1000.0)


def _max_relative_rotation_deg(calibration: dict) -> float | None:
    """相对首帧的最大相对旋转（deg）。无 samples 返回 None。"""
    samples = calibration.get("samples")
    if not isinstance(samples, list) or not samples:
        return None
    R0 = None
    max_rot = 0.0
    for s in samples:
        if not isinstance(s, dict):
            continue
        b2g = (s.get("base_to_gripper") or {}).get("matrix")
        if b2g is None:
            continue
        R = np.array(b2g)[:3, :3]
        if R0 is None:
            R0 = R
            continue
        max_rot = max(max_rot, _rotvec_deg(R @ R0.T))
    return max_rot


def check_calibration(calibration: dict) -> dict:
    """校验单个标定 JSON（dict），返回分级报告。

    report = {
      "items": [ {name, value, unit, status, detail, threshold} ... ],
      "has_fail": bool, "has_warn": bool, "selected_method": str,
    }
    """
    items = []

    sample_count = int(calibration.get("sample_count") or len(calibration.get("samples", [])) or 0)
    if sample_count >= 30:
        sc_status = _PASS
    elif sample_count >= 20:
        sc_status = _WARN
    else:
        sc_status = _FAIL
    items.append(
        {
            "name": "sample_count",
            "value": str(sample_count),
            "unit": "",
            "status": sc_status,
            "detail": f"{sample_count} 样本",
            "threshold": "≥30 pass / 20~30 warn / <20 fail",
        }
    )

    # 5 方法解 span（tool0->camera 平移，x/y/z 各轴 max-min）
    ts = _candidates_translations(calibration)
    if ts:
        arr = np.array([t for _, _, t in ts])
        spans = arr.max(axis=0) - arr.min(axis=0)
        span_max = float(spans.max())
        span_status = _grade(span_max, 0.1, 0.3)
        n_valid = sum(1 for _, v, _ in ts if v)
        items.append(
            {
                "name": "method_span",
                "value": f"{spans[0]:.3f}/{spans[1]:.3f}/{spans[2]:.3f}",
                "unit": "m",
                "status": span_status,
                "detail": f"{n_valid}/{len(ts)} 方法 valid，span_max={span_max:.3f} m",
                "threshold": "各轴 <0.1 pass / <0.3 warn / ≥0.3 fail",
            }
        )
    else:
        items.append(
            {
                "name": "method_span",
                "value": "n/a",
                "unit": "m",
                "status": _WARN,
                "detail": "无 hand_eye_candidates 候选解",
                "threshold": "需 5 方法候选",
            }
        )

    # 选中解 vs 物理先验（tool0->camera 平移距离）
    selected = str(calibration.get("selected_method", ""))
    if ts:
        sel_t = None
        for m, _, t in ts:
            if m == selected:
                sel_t = t
                break
        if sel_t is None:
            sel_t = ts[0][2]
        dev_m = float(np.linalg.norm(sel_t - np.array(PHYSICAL_PRIOR_XYZ)))
        pri_status = _grade(dev_m, 0.05, 0.15)
        items.append(
            {
                "name": "vs_physical_prior",
                "value": f"{dev_m * 1000:.0f}",
                "unit": "mm",
                "status": pri_status,
                "detail": f"选中解({selected or '?'}) 与物理先验 {PHYSICAL_PRIOR_XYZ} 偏差 {dev_m * 1000:.0f} mm",
                "threshold": "<50 pass / 50~150 warn / ≥150 fail",
            }
        )
    else:
        items.append(
            {
                "name": "vs_physical_prior",
                "value": "n/a",
                "unit": "mm",
                "status": _WARN,
                "detail": "无候选解可比较",
                "threshold": "<50 pass",
            }
        )

    # 固定外参一致性（附录 C）
    X = _selected_solution_matrix(calibration)
    mean_mm = _fixed_x_projection_mean_mm(calibration, X) if X is not None else None
    if mean_mm is not None:
        fx_status = _grade(mean_mm, 50.0, 150.0)
        items.append(
            {
                "name": "fixed_x_projection",
                "value": f"{mean_mm:.0f}",
                "unit": "mm",
                "status": fx_status,
                "detail": f"固定选中解 X 板投影离散度 mean={mean_mm:.0f} mm",
                "threshold": "<50 pass / 50~150 warn / ≥150 fail",
            }
        )
    else:
        items.append(
            {
                "name": "fixed_x_projection",
                "value": "n/a",
                "unit": "mm",
                "status": _WARN,
                "detail": "无 samples 或选中解矩阵，无法计算",
                "threshold": "<50 pass",
            }
        )

    # 姿态覆盖
    rot_deg = _max_relative_rotation_deg(calibration)
    if rot_deg is not None:
        if rot_deg > 180.0:
            rot_status = _PASS
        elif rot_deg > 90.0:
            rot_status = _WARN
        else:
            rot_status = _FAIL
        items.append(
            {
                "name": "rotation_coverage",
                "value": f"{rot_deg:.1f}",
                "unit": "deg",
                "status": rot_status,
                "detail": f"最大相对旋转 {rot_deg:.1f}°",
                "threshold": ">180 pass / 90~180 warn / <90 fail",
            }
        )
    else:
        items.append(
            {
                "name": "rotation_coverage",
                "value": "n/a",
                "unit": "deg",
                "status": _WARN,
                "detail": "无 samples",
                "threshold": ">180 pass",
            }
        )

    # reproj_px
    reproj = float(calibration.get("mean_reprojection_error_px") or 0.0)
    if reproj > 0.0:
        reproj_status = _grade(reproj, 1.5, 2.5)
        items.append(
            {
                "name": "reprojection_px",
                "value": f"{reproj:.2f}",
                "unit": "px",
                "status": reproj_status,
                "detail": f"mean reprojection {reproj:.2f} px",
                "threshold": "<1.5 pass / 1.5~2.5 warn / ≥2.5 fail",
            }
        )
    else:
        items.append(
            {
                "name": "reprojection_px",
                "value": "n/a",
                "unit": "px",
                "status": _WARN,
                "detail": "JSON 未记录 reproj_px",
                "threshold": "<1.5 pass",
            }
        )

    has_fail = any(i["status"] == _FAIL for i in items)
    has_warn = any(i["status"] == _WARN for i in items)
    return {
        "items": items,
        "has_fail": has_fail,
        "has_warn": has_warn,
        "selected_method": selected,
    }


def check_pose(pose_xyz: list, pose_rpy: list, ref_xyz: list, ref_rpy: list) -> dict:
    """对单个外参位姿（ee_camera_link->camera xyz/rpy）与参考值做偏差分级。

    供 teach_panel apply 软告警使用（payload 无 samples 时只能查这一项）。
    """
    xyz = [float(v) for v in pose_xyz]
    rpy = [float(v) for v in pose_rpy]
    ref_xyz = [float(v) for v in ref_xyz]
    ref_rpy = [float(v) for v in ref_rpy]
    trans_dev = float(np.linalg.norm(np.array(xyz) - np.array(ref_xyz)))
    rot_dev = sum(abs(a - b) for a, b in zip(rpy, ref_rpy))
    trans_status = _grade(trans_dev, 0.05, 0.15)
    rot_status = _grade(rot_dev, 0.5, 1.5)
    items = [
        {
            "name": "translation_dev",
            "value": f"{trans_dev * 1000:.0f}",
            "unit": "mm",
            "status": trans_status,
            "detail": f"与参考平移偏差 {trans_dev * 1000:.0f} mm",
            "threshold": "<50 pass / 50~150 warn / ≥150 fail",
        },
        {
            "name": "rotation_dev",
            "value": f"{rot_dev:.3f}",
            "unit": "rad",
            "status": rot_status,
            "detail": f"与参考欧拉角差之和 {rot_dev:.3f} rad",
            "threshold": "<0.5 pass / 0.5~1.5 warn / ≥1.5 fail",
        },
    ]
    return {
        "items": items,
        "has_fail": any(i["status"] == _FAIL for i in items),
        "has_warn": any(i["status"] == _WARN for i in items),
        "selected_method": "",
    }


def format_report(report: dict, title: str = "") -> str:
    """人类可读报告。"""
    lines = []
    if title:
        lines.append(f"== {title} ==")
    for i in report["items"]:
        status = i["status"].upper()
        lines.append(
            f"  [{status:4s}] {i['name']}: {i['value']} {i['unit']}  ({i['detail']})\n"
            f"         阈值: {i['threshold']}"
        )
    lines.append(f"== 结论: {'FAIL（建议拦截）' if report['has_fail'] else 'WARN（软告警）' if report['has_warn'] else 'PASS'} ==")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    """CLI：`hand_eye_calibration_check <json>...`。exit 0 = 无 fail，1 = 有 fail。"""
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print(__doc__)
        print("用法: hand_eye_calibration_check <标定 JSON 路径>...")
        return 2
    any_fail = False
    for arg in argv:
        path = Path(arg).expanduser()
        try:
            calibration = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"{path}: 读取失败: {exc}", file=sys.stderr)
            any_fail = True
            continue
        report = check_calibration(calibration)
        print(format_report(report, title=str(path)))
        print()
        if report["has_fail"]:
            any_fail = True
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
