# 手眼标定后续任务规划（hand_eye_calibration_next_steps）

> 状态：阶段 A 全部完成（2026-08-04 第四阶段）：出厂内参已真机导出并接入驱动（2026-08-04 第五阶段补接 teach_panel/depth_to_pointcloud 入口的 `factory_intrinsics_file` 默认值），真机 `ros2 topic echo` 确认 color/depth K 为出厂值、color D 非零（旧猜测 576 已取代）；附录 C 历史样本基线复现 mean 313~425 mm（< 50 mm 验收移至阶段 B）；2026-08-04 第二次不可信标定被 apply 后已回滚，Phase 5 质量门禁软告警版已实现。**2026-08-04 晚：外参姿态/平移经验值二次修正**——点云相对地面歪斜根因定位（roll=0.196=11.2° 为 home 时 tool0 相对竖直倾角，被误当 tool0→camera 安装角），清零 roll/pitch 后地面水平；高度恒定偏移（平移误差）迭代调 `camera_optical_z` 0.2196→0.05 后相机到点云映射基本无偏差。**当前生效外参：xyz=(0.0201, 0, 0.05)、rpy=(0, 0, -1.570796327)**（§四.2 修改点清单）
> 创建：2026-08-03
> 更新：2026-08-04（第六次：外参经验值二次修正——8/4 标定回滚后 RViz 点云仍相对地面严重歪斜（相对地面有角度偏移）。排查：roll=0.196 rad=11.2° 为无来源经验值，且 FK 计算证明其恰等于 home 位置时 tool0 z 轴相对竖直的倾角（URDF `camera_rpy=(0,0,90°)` 与 6/18 标定均矛盾），即被误当 tool0→camera 安装角；清零 `camera_optical_roll/pitch`（8 处同步）后地面水平实锤。随后高度恒定偏移（处处相同=平移误差非尺度）迭代调 `camera_optical_z`：0.2196→0.19→0.10→0.05，0.05 下相机到点云映射基本无偏差。**当前生效外参 xyz=(0.0201,0,0.05) / rpy=(0,0,-1.570796327)**；全部修改点与路径见 §四.2；`.bak` 备份保留旧值可回滚）
> 更新：2026-08-04（第五次：发现 teach_panel/demo_orchestrator/depth_to_pointcloud 入口未传 `factory_intrinsics_file`（gemini335.launch.py 默认空串→旧猜测 576），已把默认值改为 `config/gemini335_intrinsics/factory_calibration.json` 一处生效；同日 13:15 用户重采标定 `hand_eye_20260804_131502.json`（36 样本）离线校验仍不可信（5 方法 span 0.47/1.73/2.51 m、固定 X 投影 mean 320 mm、姿态覆盖 63°）且被 apply 改写 3 处 launch/帮助文本导致 RViz 点云严重偏移（yaw≈0° vs 应≈-90°），已回滚为物理先验经验值；Phase 5 质量门禁软告警版实现：新增 `hand_eye_calibration_check` 模块 + entrypoint（6 项指标 pass/warn/fail），teach_panel convert/apply 前软告警，7 个历史标定 JSON 全部判 FAIL 与已知结论一致）
> 更新：2026-08-04（第四次：真机 `ros2 topic echo /camera/color/camera_info` 与 `/camera/depth/camera_info` 确认——color K=fx461.147/fy461.369/cx323.604/cy246.426、D=[0.0053,-0.0473,0.0006,-4.7e-05,0.0325] 非零；depth K=fx=fy372.082/cx319.400/cy241.5、D 全零；与 `factory_calibration.json` 完全一致，旧猜测 fx=576 已被出厂值取代；阶段 A 全部完成）
> 更新：2026-08-04（第三次：真机导出 `factory_calibration.json`（SDK v1.10.27，color fx=461.15/fy=461.37/cx=323.60/cy=246.43、D 非零；depth fx=fy=372.08）并核对通过；附录 C 对 5 组历史样本完成基线复现 mean 313~425 mm / max 592~730 mm，与接入前记录一致——确认该检验不依赖相机内参且历史样本无采集帧可重算，mean < 50 mm 验收移至阶段 B（新采样重标定后））
> 更新：2026-08-04（第二次：阶段 A 驱动接入完成——新增 `factory_intrinsics_file` 参数 + `--dry-run-check` + SDK 导出工具 `scripts/vision/orbbec_*`；待真机导出 `factory_calibration.json` 并完成附录 C 验证）
> 更新：2026-08-04（第一次：事实纠正——Gemini 335 为 Orbbec 商品级深度相机，出厂已标定内参、使用中无需二次标定；原"内参从未标定"判断错误，真实问题为驱动未接入出厂内参；AprilTag 自标定流程标注为历史尝试不再使用；阶段 A/§0/§四.1/附录 A 同步修正）
> 来源：对 5 组历史/新标定数据的离线分析结论（2026-07-22 ~ 2026-08-03）
> 目标读者：后续接手手眼标定与视点规划任务的开发者

---

## 0. 本次任务总结（2026-08-03）

### 完成项

> **2026-08-04 事实纠正**：Gemini 335 为 Orbbec 商品级深度相机，**出厂已标定内参**（标定数据烧录于设备内，Orbbec SDK 功能含"设备校准数据访问"），使用过程中**无需二次标定**。下述完成项 1-4 的 AprilTag 自标定流程基于错误前提（"相机无内参"）完成，**不再作为内参来源**；主线改为读取并接入出厂内参（§四 阶段 A）。

1. **teach_panel 新增「Camera Cal」tab（内参标定图像采集）**：Node 侧新增内参采集能力（`camera_image_topic`/`intrinsic_cal_dir` 参数、Image 订阅、capture/delete/clear/count 方法、`capture_manifest.json`），UI 侧新增 Camera Cal tab（板参数录入、采集/删除/清空、计数与状态区）。标定板**复用手眼标定的 AprilTag 板**（12 tag，板 0.42×0.297 m，tag 0.07 m / pitch 0.1 m，`tagStandard41h12`），无需新板。
2. **完成 28 帧采集**：`recordings/calibration/intrinsics/frame_20260803_*.png` + `capture_manifest.json`（28 条，含板参数）。
3. **新增离线求解 entrypoint `intrinsic_cal_solve`**（`arachne_operator`）：参考图映射 3D 角点（不依赖 tag_size 假设）+ pupil_apriltags 检测 + `cv2.calibrateCamera`（plumb_bob 5 系数）+ 角点级 outlier 迭代剔除（>2 px，103 坏点剔除、1237 保留）。
4. **求解结果（已归档）**：重投影 **RMS 0.7431 px**（验收线 0.5 px 未达，用户已接受并同意后续可重采覆盖）；K `fx=462.537 fy=460.320 cx=379.275 cy=242.142`；dist `[0.02886, -0.12987, -0.00340, 0.04060, 0.11036]`。归档至 `config/gemini335_intrinsics/intrinsics_history.json`（`legacy_default` 576 猜测 + `20260803_192920` 本次标定 + `current` 指针，幂等已验证）。
5. **手眼重标结果诊断与回滚**：用户重做手眼标定后 RViz 点云严重偏移（近似垂直）。诊断：新标定物理不可信（ANDREFF 平移范数 0.565 m，URDF 物理先验 ≤0.15 m；5 方法解分歧 0.56~1.44 m；板距 0.68~1.33 m 过远；pitch 仅 ±29° 旋转约束不足；PnP 仍用猜测内参 576）。已**回滚** 3 处 launch/源码外参为经验值 `(0.0201, 0, 0.2196 / 0.196, -0.024, -1.570796327)`（`gemini335.launch.py`、`teach_panel.launch.py`、`teach_panel.py` 帮助文本），`config/hand_eye_camera_extrinsics/current.json` 指针回退至 `current_effective.json`；新标定文件 `hand_eye_20260803_194622.json` 保留存档（未生效）。

### 遗留项（下一步缺口）

- 内参**接入已完成（2026-08-04）**：`gemini335_v4l2_node` 新增 `factory_intrinsics_file` 参数，启动时读取出厂内参填入 color/depth `camera_info`（K 用出厂 fx/fy/cx/cy、D 为 plumb_bob 映射），缺失/解析失败 log warning 回退旧行为；`--dry-run-check` 离线验证通过；出厂内参已真机导出（`config/gemini335_intrinsics/factory_calibration.json`，SDK v1.10.27，color fx=461.15/fy=461.37/cx=323.60/cy=246.43 + D 非零、depth fx=fy=372.08）；真机 `ros2 topic echo` 已确认 color/depth `camera_info` K 为出厂值（见 §四.1 Phase 2 记录）。
- 附录 C 固定外参一致性检验已对 5 组历史样本完成**基线复现**（mean 313~425 mm / max 592~730 mm，与接入前记录一致）：该检验仅依赖 `base_to_gripper`/`target_to_camera` 矩阵、不依赖相机内参，且历史样本未存采集帧，无法用出厂内参重算；**mean < 50 mm 验收移至阶段 B**（出厂内参生效后新采样+重标定后验证）。
- 自标定流程（Camera Cal tab、`intrinsic_cal_solve`、`intrinsics_history.json`）已标注为历史尝试，不再作为内参来源。
- 手眼未按新采集规范重标（阶段 B 未启动）。
- 阶段 D 质量门禁未实现。

---

## 背景与目标

- 任务来源：排查手眼标定问题，目标是获得**可信的 `tool0 → camera` 外参**，为后续**视点规划（viewpoint planning，尚未实现）**铺路——视点规划的"期望相机位姿 → tool0 目标位姿"变换依赖该外参。
- 现状：共完成 5 组标定（2026-07-31、08-01、08-03×3），**全部不可信**。
- 根本原因（已定位）：Gemini335 为**商品级深度相机，出厂已标定内参**，但驱动从未读取/接入出厂内参，`camera_info` 使用猜测内参（fx=fy=576、零畸变），PnP 基于错误 K 求解 `target_to_camera`，产生随视角变化的系统性误差，污染手眼方程 AX=XB。
- 结论一句话：**换姿势、多采集解决不了问题，必须先接入相机出厂内参（无需二次标定）。**

---

## 一、现状结论：5 组标定全部不可信

| 文件 | 时间 | 样本数 | 选中方法 | reproj_px | 5 方法解 span (m) | 姿态覆盖(最大相对旋转) | tool0→camera (xyz) | 与物理先验偏差 |
|---|---|---|---|---|---|---|---|---|
| `hand_eye_20260731_163823.json` | 07-31 16:38 | 17 | HORAUD | 1.29 | [1.01, 2.62, 3.12] | 61.5° | (-0.558, -0.290, 0.242) | 0.630 m |
| `hand_eye_20260801_111119.json` | 08-01 11:11 | 30 | ANDREFF | 1.33 | [0.87, 1.00, 3.36] | 78.2° | (0.098, -0.473, -0.123) | 0.581 m |
| `hand_eye_20260803_105306.json` | 08-03 10:53 | 32 | HORAUD | 1.33 | [1.10, 1.77, 3.70] | 72.9° | (-0.540, 0.008, -0.125) | 0.630 m |
| `hand_eye_20260803_133313.json` | 08-03 13:33 | 34 | ANDREFF | 1.16 | [0.67, 1.98, 2.90] | 66.6° | (-0.409, 0.140, -0.373) | 0.718 m |
| `hand_eye_20260803_134527.json` | 08-03 13:45 | 31 | HORAUD | 1.41 | [0.48, 1.42, 2.49] | 87.2° | (-0.371, -0.482, 0.552) | 0.703 m |

说明：
- **5 方法解 span**：同一次标定内 TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS 五种方法求出的 `tool0→camera` 平移解的范围（x/y/z 各自 max-min）。可信标定应 < 0.1 m，实测 0.5~3.7 m；**DANIILIDIS 每次都发散到 z≈2.1~2.9 m**。
- **物理先验**：按 URDF 物理安装，相机应在 `tool0` 正下方约 `(0, 0, +0.20) m`。所有解偏离 0.6~0.7 m，方向/尺度均与物理不符。
- **姿态覆盖**：相对首帧的最大相对旋转角，全部 < 90°（建议 > 180°）——采样旋转不足，是求解退化的放大因素，但不是根因。

---

## 二、根因证据链（已定位）

### 根因（主）：驱动未接入相机出厂内参，PnP 用猜测 K

1. **代码事实**：`src/arachne_sensors/arachne_sensors/gemini335_v4l2_node.py` 的 `_camera_info()`（约 L32-45）：
   ```python
   fx = float(fx) if fx > 0.0 else float(width) * 0.9   # 默认：640*0.9 = 576
   fy = float(fy) if fy > 0.0 else fx
   msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]     # cx=width/2, cy=height/2
   # 畸变 d 未设置（全零）
   ```
   `src/arachne_sensors/launch/gemini335.launch.py`（L183-184）声明 `camera_fx`/`camera_fy` 默认 `0.0` → 触发猜测值；**全仓库未见读取/接入相机出厂内参**（出厂内参经 Orbbec SDK 的设备校准数据访问可读，见 §四 阶段 A）。
2. **传导路径**：标定器 `src/arachne_operator/arachne_operator/apriltag_hand_eye_calibrator.py`（L386-387）从 `/camera/color/camera_info` 读取该猜测 K 与零畸变 → PnP（`cv2.solvePnP`/`solvePnPRansac`，如 L468、L624）求解 `target_to_camera` → 每个样本的板位姿带随视角变化的系统性误差 → 手眼方程 AX=XB 的输入污染。
3. **决定性证据（固定外参一致性检验）**：假设外参 X 固定（分别用 `config/hand_eye_camera_extrinsics/current_effective.json` 的物理先验值 `(0.0201, 0, 0.2196)` 与 6/18 历史标定值），把每个样本的标定板投影回 `base_link`：
   `base_to_target = base_to_gripper @ X @ target_to_camera⁻¹`
   标定板在采样全程**静止不动**，但投影位置离散度实测 **mean 313~425 mm / max 592~730 mm**（5 组数据一致：07-31 为 425/643、08-01 为 354/726、08-03 三组为 313~394/592~730，单位 mm）。数据自洽时该值应为几 cm 量级。→ 证明 `base_to_gripper` 与 `target_to_camera` 存在系统性矛盾，**问题在数据本身，不在求解方法，也不在采集姿势**。

### 放大因素（次）：采样姿态旋转覆盖不足

- 5 组最大相对旋转 61.5°~87.2°，全部 < 90°（建议 > 180°）；旋转轴集中在相近方向（8/1 那次 30 样本中 20 个旋转轴相近）。
- 旋转不足导致 AX=XB 求解退化、多方法解发散。即使内参接入正确，也**必须**改进采样规范。

---

## 三、已排除项

| 嫌疑 | 结论 | 依据 |
|---|---|---|
| 板参数错误 | 排除 | 0.42×0.297 m 板，tag=0.07 m / pitch=0.1 m，`tagStandard41h12`，5 组声明一致且合理 |
| frame 语义错误 | 排除 | `base_link / tool0 / camera_color_optical_frame` 声明一致；TF `lookup_transform(base, tool0)` 返回 gripper→base，与 OpenCV `R_gripper2base` 语义一致；PnP 返回 target→camera，与 `R_target2cam` 一致 |
| 时间不同步 | 非主因 | 已要求静止采样；即使停稳，猜测内参依旧导致不可信 |
| 采样姿势 | 非根因 | 8/3 三次新标定已按建议加大覆盖（87.2°），结果仍同样发散 |

---

## 四、后续任务路线

### 阶段 A：Gemini335 color 相机内参接入（**必须先做，最高优先级**）

目标：把相机**出厂内参**（K + 畸变）接入相机链路，替换 `_camera_info()` 的猜测值（fx=fy=576、零畸变）。

> **事实纠正（2026-08-04）**：Gemini 335 为 Orbbec 商品级深度相机，**出厂已标定内参**（标定数据烧录于设备内，Orbbec SDK 功能列表含"设备校准数据访问"），使用过程中**无需二次标定**。原"相机内参从未标定、必须先自标定"的判断错误；真实问题是驱动从未读取/接入出厂内参。下方"历史尝试"为基于错误前提完成的工作，**不再作为内参来源**。

步骤（主线）：
1. **读取相机出厂内参** ~~（通过 Orbbec SDK 读取设备出厂标定）~~ **[已完成（2026-08-04）：`scripts/vision/orbbec_factory_intrinsics.sh` + `orbbec_dump_factory_intrinsics.cpp`，枚举 `getCalibrationCameraParamList()` 全部分辨率/镜像参数组并导出 `factory_calibration.json`（v1 schema：color/depth 各含 image_size/intrinsic/distortion[8]/is_mirrored + depth→color transform）；SDK v1.10.27 linux_arm64 预编译包，pyorbbecsdk 无 aarch64 wheel 故用 C++；当前环境已编译链接验证，真机导出待确认相机就绪]**——
   - C++：`ob::Device::getCalibrationCameraParamList()`（读取设备内烧录的 `OBCameraParam` 参数表，按分辨率/流配置含多组）；或开流后用 `ob::Pipeline::getCameraParam()` 取与当前流配置匹配的 `OBCameraParam`。
   - Python：`pyorbbecsdk`（`device.get_calibration_camera_param_list()` / `pipeline.get_camera_param()`）。
   - 字段：`rgbIntrinsic`（fx/fy/cx/cy）、`rgbDistortion`（k1..k6、p1/p2），depth 有对应字段；需按 640×480 color 流配置选取对应参数组（注意分辨率/镜像匹配）。
   - 参考：官方产品页（Orbbec SDK 功能含"设备校准数据访问"）：`https://www.orbbec.com.cn/index/Gemini330/info.html?cate=119&id=77`；SDK 文档：`github.com/orbbec/OrbbecSDK` / `orbbec.github.io/OrbbecSDK`。
2. **接入相机链路**：`gemini335_v4l2_node.py` 启动时读取出厂内参填入 `_camera_info`（color 用 rgb 标定、depth 用 depth 标定；核实 color/depth 是否共用 fx/fy）；读取失败/不可用回退旧行为（猜测 576），不阻塞启动。**[已完成（2026-08-04）**：`factory_intrinsics_file` 参数（launch 同步声明，默认空串向后兼容）、`_load_factory_intrinsics()`/`_apply_factory_intrinsics()`（plumb_bob 5 系数映射 `[k1,k2,p1,p2,k3]`，k4-k6 弃用）、`--dry-run-check` 断言通过、全量 build 通过。见 §四.1 Phase 1。**]**
3. **验证** **[已完成（2026-08-04）]**：`ros2 topic echo /camera/color/camera_info` 确认 K 为出厂值（color fx=461.147、D 非零；depth fx=fy=372.082、D 全零）；附录 C 固定外参一致性检验对 5 组历史样本完成**基线复现**（mean 313~425 mm，与接入前一致；该检验不依赖内参且历史样本无采集帧可重算，< 50 mm 验收移至阶段 B 新采样后）。

历史尝试（2026-08-03 完成，基于"相机无内参"的错误前提，**不再使用**，归档保留备查）：
1. ~~准备标定板~~：复用手眼标定 AprilTag 板（12 tag，板 0.42×0.297 m，tag_size 0.07 m / tag_pitch 0.1 m，`tagStandard41h12`，`/home/jetson/zhaoyang/arachne_floor_apriltag_board_a3.png`）。
2. ~~采集标定图像~~：teach_panel「Camera Cal」tab 采集 **28 帧**，`recordings/calibration/intrinsics/`（`frame_20260803_*.png` + `capture_manifest.json`，640×480）。
3. ~~求解内参~~：`intrinsic_cal_solve` entrypoint（参考图映射 3D 角点 + pupil_apriltags + `cv2.calibrateCamera` + 角点级 outlier 剔除，103 坏点剔除 / 1237 保留）。结果 **RMS 0.7431 px**（验收线 0.5 px 未达）；K `fx=462.537 fy=460.320 cx=379.275 cy=242.142`；dist `[0.02886, -0.12987, -0.00340, 0.04060, 0.11036]`。归档：`config/gemini335_intrinsics/intrinsics_history.json`。
4. ~~接入链路（方案 B）~~：读取 `intrinsics_history.json` 的 `current` entry 填入 `_camera_info`——**已废弃**，出厂内参优先，自标定结果不接入链路。

### 阶段 B：重做手眼标定（出厂内参接入后）

目标：产出可信 `tool0 → camera` 外参。

采集规范（沿用已验证建议）：
- **姿态覆盖**：相机沿"半球面"围绕标定板走（低/中/高、左/中/右），每个位姿 pitch/yaw 都变化；**累计相对旋转 > 180°**，旋转轴至少覆盖 3 个方向；每次标定更换起始位姿与轨迹。
- **静止采样**：每次 capture 前机械臂完全停稳 ≥ 1 s。
- **样本量**：≥ 30 样本/次，至少做 3 次独立标定。
- 每轮记录：样本数、最大相对旋转、5 方法离散度、tool0→camera、与先验偏差（对照文档第一节对比表格式）。

验收标准（全部满足才算可信）：
- 5 方法解 span < 0.1 m（x/y/z 各轴）；
- tool0→camera ≈ (0, 0, 0.20) m ± 0.05 m；
- 固定外参一致性检验（附录 C 脚本）mean 偏差 < 50 mm；
- reproj_px < 1.5；
- 3 次独立标定结果互相一致（互差 < 5 cm）。

### 阶段 C：可信结果接入运行时 TF

目标：让 RViz 点云、抓取链路使用真实外参，替代当前硬编码默认。

1. 换算：`tool0→camera`（标定输出）→ `ee_camera_link→camera_color_optical_frame`。`teach_panel.py` 已有 `hand_eye_convert_tool0_to_ee_camera()`（含 `_TOOL0_TO_EE_CAMERA_LINK` 组合链路），可复用。
2. 注意确认：点云/投影链路实际用 `camera_depth_optical_frame`，需核实 color 与 depth optical frame 的静态 TF 是否同一值（当前 `_camera_info` 对 color/depth 共用 fx/fy，TF 发布需核对）。
3. 替换 `src/arachne_sensors/launch/gemini335.launch.py` 的 `camera_optical_*` 参数。**现状（2026-08-04 晚第六次修正）**：当前生效值为 `xyz=(0.0201, 0, 0.05) / rpy=(0, 0, -1.570796327)`——2026-08-04 上午 8/4 第二次不可信标定被 apply 后已回滚为 `0.0201/0/0.2196 / 0.196/-0.024/-1.5708`，但当晚发现该经验值姿态部分为假值（见 §四.2），二次修正为上述值；`config/hand_eye_camera_extrinsics/current.json` 指针指向 `current_effective.json`，config 与 launch 一致；待 Phase 3 产出可信标定后统一替换，避免再次分叉。
4. 更新 `src/arachne_description/config/physical_parameters.yaml`：`hand_eye_tool0_to_camera_color_optical` 的 status 从 `archived_hand_eye_not_used_for_rviz_camera_tf` 改为当前生效，并更新数值/来源。
5. 验证：
   - RViz 点云与实物对齐（放垂直标定板，不再歪斜）；
   - 机械臂移动后点云投影跟随，不再偏移；
   - `grasp_preview_pipeline` 的 `base_grasp_xyz` 与实物位置对比 < 3 cm。

### 四.2 2026-08-04 晚外参经验值二次修正（修改点清单）

> 背景：8/4 第二次不可信标定回滚后，RViz 点云仍相对地面严重歪斜（角度偏移）。定位：`camera_optical_roll=0.196`（11.2°）/`pitch=-0.024` 为无来源经验值，FK 计算证明 0.196 恰为 **home 位置时 tool0 z 轴相对竖直的倾角**（URDF `ee_camera.xacro` 的 `camera_rpy=(0,0,90°)` 及 6/18 历史标定均无该倾角）——即该值本应是"home 时 tool0 的倾斜"，被误当成了 tool0→camera 的安装角。清零 roll/pitch 后地面水平（真机确认）。随后高度**恒定**偏移（处处相同 = 平移误差、非尺度误差）迭代调 z：0.2196→0.19→0.10→0.05，**0.05 下相机到点云映射基本无偏差**（真机确认，标定板放地面）。

**当前生效外参：`xyz=(0.0201, 0, 0.05)`、`rpy=(0, 0, -1.570796327)`**（对应 launch 参数 `camera_optical_x/y/z/roll/pitch/yaw := 0.0201/0.0/0.05/0.0/0.0/-1.570796327`）。

修改参数路径清单（每次调整需 8 处同步 + `./scripts/build/build_workspace.sh` 重建）：

| # | 路径 | 修改位置 | 本次改动 |
|---|---|---|---|
| 1 | `src/arachne_sensors/launch/gemini335.launch.py` | `camera_optical_roll/pitch/z` `DeclareLaunchArgument` 默认值 | 0.196/-0.024/0.2196 → 0.0/0.0/0.05 |
| 2 | `src/arachne_operator/launch/teach_panel.launch.py` | 帮助文本命令行传参串 `camera_optical_z:=0.05 camera_optical_roll:=0.0 ...` | 同步 |
| 3 | `src/arachne_operator/launch/demo_orchestrator.launch.py` | 同上命令行传参串 | 同步 |
| 4 | `src/arachne_sensors/launch/depth_to_pointcloud.launch.py` | `camera_optical_roll/pitch/z` 默认值 | 同步 |
| 5 | `src/arachne_description/launch/display.launch.py` | `camera_optical_roll/pitch/z` 默认值 | 同步 |
| 6 | `src/arachne_operator/arachne_operator/teach_panel.py` | `CURRENT_HAND_EYE_XYZ=[0.0201,0,0.05]`、`CURRENT_HAND_EYE_RPY=[0,0,-1.570796327]`（apply 校验基准）+ 帮助文本 L~581 | 同步 |
| 7 | `src/arachne_operator/arachne_operator/hand_eye_calibration_check.py` | `PHYSICAL_PRIOR_XYZ=(0.0201,0,0.05)`、`PHYSICAL_PRIOR_RPY=(0,0,-1.570796327)`（门禁物理先验基准）+ docstring | 同步 |
| 8 | `config/hand_eye_camera_extrinsics/current_effective.json` | `xyz`、`rpy`、`quaternion_xyzw`、`matrix_4x4` 四字段自洽更新 | 同步 |

> 旧值备份：`src/arachne_operator/launch/teach_panel.launch.py.bak`、`src/arachne_sensors/launch/gemini335.launch.py.bak` 保留本次修改前状态，可随时回滚。
> 遗留：xyz 平移（0.0201, 0, 0.05）仍为经验试调值（z 已真机确认、x/y 未单独验证），**正式值待 Phase 3 可信手眼标定产出后替换**；`physical_parameters.yaml` 的 `hand_eye_tool0_to_camera_color_optical` 仍为 archived 状态待更新。

### 阶段 D：标定流程代码改进（可与 A/B 并行，可选）

- **质量门禁**：solve 前检查样本数、姿态旋转覆盖（> 90°，提示 > 180°），不满足则拒绝/告警。
- **物理先验校验**：求解结果若 `tool0→camera` 偏离 (0,0,0.2) 超过阈值，直接判不可信。
- **一致性报告落盘**：把 5 方法离散度、consistency_score、姿态覆盖统计写入标定 JSON（当前 JSON 已含 `hand_eye_candidates`，但缺汇总指标）。
- **命名修正**：`apriltag_hand_eye_calibrator.py` 中 `base_to_gripper`/`r_gripper_to_base` 命名混乱（实际内容为 gripper→base），易误导后续维护。

### 阶段 E：视点规划衔接（远期）

- 当前仓库**不存在**视点规划模块（无 `viewpoint`/`view planning`/`NBV` 相关代码）；现有 `grasp_preview_pipeline.py` 的 `real_search_scan` 是关节空间摆动扫描，不依赖外参。
- 本任务铺路意义：阶段 C 产出可信 `tool0→camera` 后，视点规划的"期望相机位姿 → tool0 目标位姿（`期望tool0 = 期望相机位姿 × (tool0→camera)⁻¹`）→ IK"链路即具备输入前提。

### 四.1 下一步执行计划（2026-08-03 起；Phase 1/2 已按 2026-08-04 事实纠正更新）

> 依赖关系：Phase 1 → 2 → 3 → 4 串行；Phase 5 可与 Phase 1-3 并行（建议在 Phase 3 前完成门禁）。
> 真机约束：Phase 3、4 需用户确认机械臂/相机就绪；dry-run 与离线校验路径不触碰真实硬件。

**Phase 1 内参接入相机链路（阶段 A 步骤 2；代码，无真机风险）** **[已完成（2026-08-04）**：采用"SDK 一次性导出 `factory_calibration.json` + 节点启动读取"方案（pyorbbecsdk 无 aarch64 wheel，不引入 SDK 到驱动，避免 V4L2/SDK 同时占用 USB 设备冲突）；`gemini335_v4l2_node.py` 新增 `factory_intrinsics_file` 参数 + `_load_factory_intrinsics()`/`_apply_factory_intrinsics()` + `--dry-run-check`，launch 声明参数默认空串向后兼容；构建与 dry-run 均通过。**]**
- 用 Orbbec SDK（C++/pyorbbecsdk）读取 Gemini 335 出厂内参（`getCalibrationCameraParamList()` / `pipeline.getCameraParam()`，取 640×480 color 对应的 `rgbIntrinsic`/`rgbDistortion`）。实现方式二选一：直接集成 SDK 到驱动节点，或先用 SDK 导出出厂标定到配置文件（如 `config/gemini335_intrinsics/factory_calibration.json`）再由节点读取——需评估当前 V4L2 驱动架构是否引入 SDK 依赖。
- `gemini335_v4l2_node.py` 启动时读取出厂内参填入 `_camera_info`（color 用 rgb 标定、depth 用 depth 标定；核实 color/depth 是否共用 fx/fy）；读取失败/解析失败回退旧行为（猜测 576），不阻塞启动。
- 核实 color 与 depth 两条 CameraInfo 消费链（含 `depth_to_pointcloud`）均使用真实内参；`color/depth` 共用 fx/fy 的现状一并记录。
- 验证：`./scripts/build/build_workspace.sh` 重建 + `ros2 topic echo /camera/color/camera_info` 确认 K 为出厂标定值（与 SDK 读取一致）且畸变生效。

**Phase 2 内参回归验证（阶段 A 步骤 3）** **[已完成（2026-08-04）：附录 C 基线复现 mean 313~425 mm 与接入前一致]**
- 用附录 C 固定外参一致性检验脚本对 5 组历史手眼样本复算标定板投影离散度（mean/max mm）：结果 313~425/592~730 mm 与接入前记录一致。
- **口径修正（2026-08-04 用户确认）**：该检验仅用 `base_to_gripper`/`target_to_camera` 矩阵、不依赖相机内参，且历史样本未存采集帧无法用出厂内参重估 t2c，故"历史样本重算"结果必然不变；**mean < 50 mm 验收移至阶段 B**（新采样重标定后验证）。
- 分支 **[已完成（2026-08-04）]**：真机 `ros2 topic echo` 确认 color K=461.147/323.604/246.426 + D 非零、depth K=372.082 + D 全零 → **阶段 A 收尾**；< 50 mm 作为阶段 B 验收（见 Phase 3 每轮离线校验）。

**Phase 3 重做手眼标定（阶段 B；需真机，采集前用户确认机械臂就绪）**
- 采集规范：相机沿半球面绕板（低/中/高 × 左/中/右）、累计相对旋转 > 180°、旋转轴 ≥ 3 方向、每次静止 ≥ 1 s、≥ 30 样本/次、共 3 次独立标定。
- 每轮 solve 后**离线校验**（不自动写入任何 launch/config）：5 方法解 span < 0.1 m、`tool0→camera ≈ (0, 0, 0.2) ± 0.05 m`、固定 X 投影 mean < 50 mm、reproj < 1.5 px、3 次互差 < 5 cm。
- 全部达标才进入 Phase 4；任一不达标按指标重采该轮（沿用"先验证后生效"教训，避免再次出现不可信标定自动改写 launch）。

**Phase 4 可信外参接入运行时 TF（阶段 C）**
- 用 `teach_panel.hand_eye_convert_tool0_to_ee_camera()` 把标定输出换算为 `ee_camera_link → camera_color_optical_frame`。
- 核实 color/depth optical frame 的静态 TF 是否同值，点云链路实际消费 frame 以实际为准。
- 统一替换 3 处 `camera_optical_*`（`gemini335.launch.py`、`teach_panel.launch.py`、`teach_panel.py` 帮助文本）并同步 `config/hand_eye_camera_extrinsics/current_effective.json`，保证 config 与 launch 一致。
- 更新 `physical_parameters.yaml` 的 `hand_eye_tool0_to_camera_color_optical` 状态与数值。
- 验证：RViz 点云与实物对齐（垂直板不歪斜）、机械臂移动后点云跟随不偏移、`grasp_preview_pipeline` 的 `base_grasp_xyz` 与实物对比 < 3 cm。

**Phase 5（2026-08-04 已实现质量门禁软告警版；solve 前硬门禁与命名修正仍可做）流程门禁与清理（阶段 D）**
- **[已完成（2026-08-04）质量门禁软告警版]**：新增 `arachne_operator/hand_eye_calibration_check.py` 模块（纯函数，6 项指标：样本数/5 方法 span/物理先验偏差/固定 X 投影/姿态覆盖/reproj，pass/warn/fail 分级）+ console_scripts `hand_eye_calibration_check`（`ros2 run arachne_operator hand_eye_calibration_check <json>`，exit 0=无 fail、1=有 fail、2=用法错）；teach_panel convert 结果区显示门禁状态（FAIL/WARN/PASS + 不达标项列表），`_save_converted_extrinsics` 与 `_apply_hand_eye_extrinsics` 前 fail 项 askyesno 软告警（可确认后写入，不硬拦；apply 优先追溯 `source_json` 标定文件做全量校验，无则用 `check_pose` 与当前生效值比偏差）；对 `log/calibration/hand_eye/` 全部 7 个标定 JSON 输出与已知结论一致（全部 FAIL）。
- `apriltag_hand_eye_calibrator.py` solve 前加最小硬门禁：样本数、姿态旋转覆盖、物理先验偏离（`tool0→camera` 距离 > 阈值即拒）——**尚未做**（当前软告警已能阻止误 apply，硬门禁可后续再加）。
- 命名修正：`base_to_gripper`/`r_gripper_to_base` → 统一 gripper→base 语义——**尚未做**。
- 清理 `teach_panel.py.bak` 等备份、review 未提交改动（附录 B 列表）。

---

## 五、安全与流程约束（沿用 AGENTS.md）

- 真机标定/运动验证前需用户明确确认机械臂就绪。
- dry-run/headless 路径不触碰真实硬件。
- 改动后经 `./scripts/build/build_workspace.sh` 重建。
- `teach_panel.launch.py` 保持为正常操作入口。
- `scripts/` 仅放 setup/bootstrap/field helpers，不新增稳定运行时 wrapper。

---

## 附录 A：相关文件索引

| 文件 | 作用 |
|---|---|
| `src/arachne_operator/arachne_operator/apriltag_hand_eye_calibrator.py` | 手眼标定节点（capture/solve/reset），读 CameraInfo 做 PnP |
| `src/arachne_sensors/arachne_sensors/gemini335_v4l2_node.py` | Gemini335 驱动，`_camera_info()` 生成内参（旧：猜测 576 + 零畸变；**2026-08-04 已接入出厂内参**：`factory_intrinsics_file` 参数 + `_load_factory_intrinsics()`/`_apply_factory_intrinsics()` + `--dry-run-check`） |
| `src/arachne_sensors/launch/gemini335.launch.py` | 相机 launch，`camera_optical_*` 硬编码外参默认、`camera_fx/fy` 默认 0；`factory_intrinsics_file` 参数（默认空串→旧行为） |
| `src/arachne_description/config/physical_parameters.yaml` | 手眼标定归档（6/18，status=archived）、mount 参数 |
| `src/arachne_description/urdf/sensors/ee_camera.xacro` | ee_camera_link 安装链（tool0→ee_camera_link 结构） |
| `src/arachne_operator/arachne_operator/hand_eye_calibration_check.py` | **质量门禁模块（2026-08-04 新增）**：离线校验任意标定 JSON（6 项指标 pass/warn/fail 分级 + `check_pose` 位姿偏差），供 CLI entrypoint `hand_eye_calibration_check` 与 teach_panel convert/apply 软告警复用 |
| `src/arachne_operator/arachne_operator/teach_panel.py` | Hand-Eye 配置页、`hand_eye_convert_tool0_to_ee_camera()` 换算；convert/apply 前质量软告警（2026-08-04 接入） |
| `src/arachne_operator/arachne_operator/grasp_preview_pipeline.py` | 抓取预览/检测链路，`lookup_transform` 用相机 TF |
| `log/calibration/hand_eye/hand_eye_*.json` | 5 组标定结果（07-31、08-01、08-03×3） |
| `config/hand_eye_camera_extrinsics/current_effective.json` | config 文件：`xyz=(0.0201,0,0.05) / rpy=(0,0,-1.570796327)`（2026-08-04 晚第六次修正后值，见 §四.2）。`current.json` 指针指回本文件，与 launch 一致（待 Phase 3 可信标定后统一替换） |
| `config/gemini335_intrinsics/intrinsics_history.json` | 内参归档：`legacy_default`（576 猜测）+ `20260803_192920`（历史尝试自标定，**不再使用**）+ `current` 指针；保持只读，出厂内参独立承载于 `factory_calibration.json` |
| `config/gemini335_intrinsics/factory_calibration.json` | **出厂内参（阶段 A 主数据源，待真机导出）**：由 `scripts/vision/orbbec_factory_intrinsics.sh` 生成，v1 schema（color/depth 各含 image_size/intrinsic/distortion[8]/is_mirrored + transform），驱动经 `factory_intrinsics_file` 参数读取 |
| `recordings/calibration/intrinsics/` | 内参标定采集：28 帧 `frame_20260803_*.png` + `capture_manifest.json`（Camera Cal tab 产出；**历史尝试数据，不再使用**） |
| `src/arachne_operator/arachne_operator/intrinsic_cal_solve.py` | 内参离线求解 entrypoint（`ros2 run arachne_operator intrinsic_cal_solve --dir recordings/calibration/intrinsics`；**历史尝试，不再作为内参来源**） |
| `src/arachne_operator/arachne_operator/teach_panel.py`（Camera Cal tab） | 内参标定图像采集 UI：板参数录入、采集/删除/清空、计数与状态区（**历史尝试 UI，不再作为内参来源**） |
| `scripts/vision/orbbec_factory_intrinsics.sh` | **field helper（2026-08-04 新增）**：一键导出出厂内参——SDK 缺失时下载官方 v1.10.27 linux_arm64 到 `build/orbbec_sdk/`，g++ 链接 `libOrbbecSDK.so` 编译并运行导出工具 |
| `scripts/vision/orbbec_dump_factory_intrinsics.cpp` | **field helper（2026-08-04 新增）**：枚举 `getCalibrationCameraParamList()` 全部分辨率/镜像参数组（供人工核对），选 640×480/640×480 组导出 `factory_calibration.json`；无 USB 权限时干净退出并提示 udev 规则 |
| Orbbec SDK / 官方文档 | Gemini 335 出厂内参读取渠道：SDK 功能含"设备校准数据访问"（`getCalibrationCameraParamList()` / pyorbbecsdk）；产品页 `https://www.orbbec.com.cn/index/Gemini330/info.html?cate=119&id=77`；官方 arm64 预编译包：`github.com/orbbec/OrbbecSDK` releases |
| `docs/depth_points_projection_bug_fix_plan.md` | 早期"点云投影偏移"排查（根因 B 与本任务一致） |

## 附录 B：仓库未提交改动提醒（动手前先理清）

以下文件在分析期间即为修改/未跟踪状态，实施前建议先 review 或提交：

- `M src/arachne_operator/arachne_operator/apriltag_hand_eye_calibrator.py`（+302/-200，较大改动）
- `M src/arachne_operator/arachne_operator/teach_panel.py`（Camera Cal tab + 手眼帮助文本回滚经验值）
- `M src/arachne_sensors/arachne_sensors/gemini335_v4l2_node.py`
- `M src/arachne_sensors/launch/gemini335.launch.py`（`camera_optical_*` 已回滚经验值）
- `M src/arachne_operator/launch/teach_panel.launch.py`（帮助文本已回滚经验值）、`demo_orchestrator.launch.py`、`docs/step_demo_plan.md`
- `?? src/arachne_operator/arachne_operator/intrinsic_cal_solve.py`、`M src/arachne_operator/setup.py`（新增 `intrinsic_cal_solve` console_scripts）
- `?? src/arachne_operator/arachne_operator/teach_panel.py.bak` 等 `.bak` 备份文件（可清理）
- `?? scripts/vision/hand_eye_calibration_setup.sh`、`?? recordings/teach/*.json`（测试记录）；`recordings/calibration/intrinsics/`（内参采集数据）已被 `.gitignore` 忽略（`recordings/*`），不提交

## 附录 C：离线复算脚本（已固化为 `hand_eye_calibration_check`，下方脚本保留参考）

> **2026-08-04**：附录 C 的固定外参一致性检验已与其他指标（5 方法 span、物理先验偏差、姿态覆盖、reproj、样本数）一起固化为
> `ros2 run arachne_operator hand_eye_calibration_check <标定 JSON>...`（exit 0=无 fail、1=有 fail），无需再手跑下方脚本。

固定外参一致性检验 + 5 方法离散度 + 姿态覆盖复算：

```python
import json, numpy as np, math

def e2m(rpy):
    r, p, y = rpy
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def inv(m):
    R, t = m[:3,:3], m[:3,3]
    return np.block([[R.T, -R.T @ t.reshape(3,1)], [np.zeros((1,3)), 1.0]])

def rotvec_deg(R):
    c = max(-1.0, min(1.0, (np.trace(R)-1.0)/2.0))
    return math.degrees(math.acos(c))

def fixed_x_projection(path, X):
    """固定外参一致性检验：板投影离散度"""
    d = json.load(open(path))
    pts = []
    for s in d['samples']:
        b2g = np.array(s['base_to_gripper']['matrix'])
        t2c = np.array(s['target_to_camera']['matrix'])
        b2t = b2g @ X @ inv(t2c)
        pts.append(b2t[:3,3])
    pts = np.array(pts)
    dev = np.linalg.norm(pts - pts.mean(axis=0), axis=1)
    return dev.mean()*1000, dev.max()*1000  # mm

# 用法示例：X 取物理先验/候选标定解换算回 tool0 系
# X = np.eye(4); X[:3,:3] = e2m([0,0,-1.570796327]); X[:3,3] = [0.0201,0,0.05]  # 当前生效外参（§四.2）
# mean_mm, max_mm = fixed_x_projection('log/calibration/hand_eye/hand_eye_20260803_134527.json', X)
# print(mean_mm, max_mm)
```
