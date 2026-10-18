# 抓取失败：moveJoint 未到位（Aubo speed fraction = 0.26）

- 日期：2026-09-21
- 现象类别：real execution blocked（真实抓取执行被安全拦截，机械臂未完成抓取段）
- 影响：当天 10:02 / 10:05 / 10:26 / 10:30 连续 4 次真实抓取全部失败

## 现象

每次都在第一段关节运动（`grasp:close`，即移动到抓取位姿）超时：

```
[WARN] REAL moveJoint 1/5 grasp:close: t=1.81s duration=0.00s
[ERROR] real execution blocked: Aubo SDK moveJoint arrival timeout at
        grasp:close: max_error=0.475rad tolerance=0.045rad
```

机械臂确实在动（残留 0.47~0.55 rad，而不是保持初始偏差 ~1.3 rad），但走不完。

| 尝试 | 起始偏差 Δ | 等待 | 实际转过 | 平均角速度 | 实际/配置速度 |
|---|---|---|---|---|---|
| 10:02:57 | 1.379 rad | 17.66 s | 0.832 rad | 0.0471 rad/s | 0.262 |
| 10:05:23 | 1.386 rad | 17.70 s | 0.838 rad | 0.0473 rad/s | 0.263 |
| 10:26:25 | 1.283 rad | 17.13 s | 0.809 rad | 0.0472 rad/s | 0.262 |
| 10:30:54 | 1.292 rad | 17.18 s | 0.817 rad | 0.0476 rad/s | 0.264 |

配置速度 `real_sdk_move_speed=0.18 rad/s`，实测只有约 26%。

## 根因

控制器侧的全局"速度和加速度比例"被改成了 0.26（默认应为 1.0，在示教器上被调低）。`moveJoint(q, a, v, ...)` 的 `v`/`a` 会再乘这个比例，因此：

- 实际速度 ≈ 0.18 × 0.26 = 0.047 rad/s（与上表实测一致，误差 <1%）
- 而 `_real_sdk_wait_arrival()` 的超时预算按配置值估算：`max(1.94, 1.386/0.18) + 10 = 17.7s`，没有乘该比例，比实际所需（≈30s）少约 3.8 倍 → 必然超时。

相关代码/配置：

- 超时预算：`src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` `_real_sdk_wait_arrival()`（`timeout = max(segment_dt, max_delta/speed) + padding`）
- padding：`--real-sdk-arrival-timeout-padding 10`（`src/arachne_operator/arachne_operator/teach_panel.py` 启动串里写死）
- 速度/加速度：`src/arachne_operator/launch/grasp_task_server.launch.py`（`real_sdk_move_speed=0.18`、`real_sdk_move_accel=0.25`）
- 到位容差：`--real-sdk-goal-tolerance-rad` 默认 0.045

已排除：目标不可达 / IK 失败。规划日志 `max_pos_error=0.003m`、`local_strict_ik` 通过；用仓库自带运动学复算同一位姿，tool0 目标半径 0.83 m 在工作空间内（门限 1.03 m）且关节均在标称限位内。

## 排查方法（只读，无运动）

直接向控制器查询速度和加速度比例：

```bash
# JSON-RPC over 192.168.127.128:30004，只读查询
{"jsonrpc":"2.0","method":"rob1.MotionControl.getSpeedFraction","params":[],"id":1}
```

Aubo SDK 头文件（`include/aubo/robot/motion_control.h`）明确 `getSpeedFraction()` 返回"速度和加速度比例，默认为 1"。

## 处置与验证

- 在示教器把速度与加速度比例恢复为 100%。
- 复验：`getSpeedFraction` 返回 `1.0`（`isSpeedFractionCritical=False`，`RobotModeType=Running`）。

## 经验

- 机械臂"动了但没到位"时，先查 `MotionControl.getSpeedFraction`，再怀疑机械/负载/碰撞。
- 不要用直接调大 `real_sdk_move_speed` 的方式补偿倍率：倍率一旦恢复 1.0，速度会突然放大数倍，有安全风险。
- 超时预算只按配置速度估算，不含控制器倍率；倍率低于 1 时，运动幅度较大的段（>0.6 rad）必然超时。
