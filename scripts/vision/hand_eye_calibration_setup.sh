#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/ros_env.sh"
arachne_require_ros_distro

set +u
export ARACHNE_ENV_QUIET=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
unset ARACHNE_ENV_QUIET
set -u

arachne_source_workspace_setup \
  "${ROOT_DIR}" \
  "Workspace is not built yet. Run ./scripts/build/build_workspace.sh first."

# ── 可配置参数 ──────────────────────────────────────────────
AUBO_ROBOT_IP="${AUBO_ROBOT_IP:-192.168.127.128}"
AUBO_TYPE="${AUBO_TYPE:-aubo_i5}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-60}"
SKIP_ARM=false
SKIP_CAMERA=false
CONFIRM=false

usage() {
  cat <<EOF
Usage:
  ./scripts/vision/hand_eye_calibration_setup.sh --yes [选项]

一键启动手眼标定所需的全部组件：
  1. 环境检查
  2. 启动 Aubo 机械臂驱动（后台）
  3. 启动 Gemini335 相机（后台）
  4. 启动标定节点（后台）
  5. 启动交互式 GUI（前台，阻塞）
  退出 GUI 后自动清理所有后台进程。

  注意：机械臂上电需手动在示教器上完成（connect → power on → start）。

选项:
  -y, --yes           确认启动真实硬件（必须）
  --skip-arm          跳过机械臂启动（仅启动相机 + 标定）
  --skip-camera       跳过相机启动（仅启动机械臂 + 标定）
  --skip-env-check    跳过环境检查
  -h, --help          显示帮助

环境变量:
  AUBO_ROBOT_IP=${AUBO_ROBOT_IP}
  AUBO_TYPE=${AUBO_TYPE}
  WAIT_TIMEOUT_SEC=${WAIT_TIMEOUT_SEC}
  ARACHNE_HAND_EYE_IMAGE_TOPIC    图像话题（默认 /camera/color/image_raw）
  ARACHNE_HAND_EYE_TAG_SIZE_M     AprilTag 尺寸（默认 0.070）
  ARACHNE_HAND_EYE_OUTPUT_DIR     输出目录（默认 log/calibration/hand_eye）

GUI 操作:
  SPACE   采集一个样本（移动机械臂到不同位姿后按）
  s       求解手眼矩阵（至少采集 5 个样本）
  r       重置已采集样本
  q       退出

示例:
  ./scripts/vision/hand_eye_calibration_setup.sh --yes
  ./scripts/vision/hand_eye_calibration_setup.sh --yes --skip-arm
EOF
}

SKIP_ENV_CHECK=false

while (($#)); do
  case "$1" in
    -y|--yes)         CONFIRM=true ;;
    --skip-arm)       SKIP_ARM=true ;;
    --skip-camera)    SKIP_CAMERA=true ;;
    --skip-env-check) SKIP_ENV_CHECK=true ;;
    -h|--help)        usage; exit 0 ;;
    *)                echo "未知选项: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

if [[ "${CONFIRM}" != "true" ]]; then
  cat >&2 <<'EOF'
拒绝启动手眼标定流程，缺少 --yes 确认。

该脚本会启动真实 Aubo 机械臂驱动和 Gemini335 相机。
请确认工作空间安全、急停按钮在手边，然后运行：

  ./scripts/vision/hand_eye_calibration_setup.sh --yes
EOF
  exit 2
fi

# ── 日志目录 ────────────────────────────────────────────────
LOG_DIR="${ROOT_DIR}/log/hand_eye_calibration/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

BACKGROUND_PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if ((${#BACKGROUND_PIDS[@]})); then
    echo "正在停止后台进程..."
    for pid in "${BACKGROUND_PIDS[@]}"; do
      kill -INT "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in "${BACKGROUND_PIDS[@]}"; do
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    done
  fi
  echo "日志目录: ${LOG_DIR}"
  exit "${status}"
}
trap cleanup EXIT INT TERM

start_background() {
  local label="$1"
  local log_file="$2"
  shift 2
  echo "启动 ${label}; 日志: ${log_file}"
  "$@" >"${log_file}" 2>&1 &
  local pid=$!
  BACKGROUND_PIDS+=("${pid}")
  sleep 3
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "${label} 启动失败，日志：" >&2
    cat "${log_file}" >&2 || true
    exit 1
  fi
  echo "  PID: ${pid}"
}

wait_for_topic() {
  local topic="$1"
  local label="$2"
  local hint="${3:-}"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    # 使用 ros2 topic echo 检测话题是否有数据
    if timeout 2 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      echo "  就绪: ${label} (${topic})"
      return 0
    fi
    sleep 1
  done
  echo "等待超时: ${label} (${topic})" >&2
  if [[ -n "${hint}" ]]; then
    echo "  提示: ${hint}" >&2
  fi
  exit 1
}

wait_for_controller_active() {
  local controller="$1"
  local label="$2"
  local hint="${3:-}"
  local deadline=$((SECONDS + WAIT_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    local output
    output="$(timeout 3 ros2 control list_controllers 2>/dev/null || true)"
    local clean
    clean="$(python3 -c 'import re,sys; print(re.sub("\x1b\\[[0-9;]*m","",sys.stdin.read()),end="")' <<<"${output}")"
    if awk '{print $1, $NF}' <<<"${clean}" | grep -qx "${controller} active"; then
      echo "  就绪: ${label} (${controller})"
      return 0
    fi
    sleep 1
  done
  echo "等待超时: ${label} (${controller})" >&2
  if [[ -n "${hint}" ]]; then
    echo "  提示: ${hint}" >&2
  fi
  timeout 3 ros2 control list_controllers 2>/dev/null || true
  exit 1
}

# ── 开始 ────────────────────────────────────────────────────
echo "========================================="
echo "  Arachne 手眼标定一键启动"
echo "========================================="
echo "  工作空间:   ${ROOT_DIR}"
echo "  Aubo IP:    ${AUBO_ROBOT_IP} (${AUBO_TYPE})"
echo "  启动机械臂: $(! ${SKIP_ARM} && echo 是 || echo 否)"
echo "  启动相机:   $(! ${SKIP_CAMERA} && echo 是 || echo 否)"
echo "  日志目录:   ${LOG_DIR}"
echo "========================================="

cd "${ROOT_DIR}"

# 1. 环境检查
if [[ "${SKIP_ENV_CHECK}" != "true" ]]; then
  echo "[1/4] 环境检查..."
  "${ROOT_DIR}/scripts/hardware/check_real_hardware_env.sh" --strict \
    >"${LOG_DIR}/00_env_check.log" 2>&1 || {
      echo "环境检查失败，详见: ${LOG_DIR}/00_env_check.log" >&2
      exit 1
    }
  echo "  通过"
else
  echo "[1/4] 跳过环境检查"
fi

# 2. 启动 Aubo 机械臂
if [[ "${SKIP_ARM}" != "true" ]]; then
  echo "[2/4] 启动 Aubo 机械臂驱动..."
  
  # 启动 Aubo SDK 桥接节点
  start_background "Aubo SDK 桥接" \
    "${LOG_DIR}/01_aubo_driver.log" \
    env ARACHNE_CONFIRM_AUBO_DRIVER=YES \
        ARACHNE_AUBO_ALLOW_PRESTART=YES \
        AUBO_ROBOT_IP="${AUBO_ROBOT_IP}" \
        AUBO_TYPE="${AUBO_TYPE}" \
        "${ROOT_DIR}/scripts/hardware/real_aubo_bringup.sh" \
        use_aubo_ros2_driver:=false

  # 启动 Aubo ROS2 Control（提供 /joint_states）
  start_background "Aubo ROS2 Control" \
    "${LOG_DIR}/01b_aubo_control.log" \
    ros2 launch aubo_ros2_driver aubo_control.launch.py \
      use_fake_hardware:=false \
      robot_ip:="${AUBO_ROBOT_IP}" \
      aubo_type:="${AUBO_TYPE}"

  echo "  提示：请在示教器上完成 connect → power on → start"
  echo "  等待 Aubo 就绪..."
  wait_for_topic "/joint_states" "Aubo 关节状态" \
    "请在示教器上完成 connect → power on → start，或检查 Aubo IP ${AUBO_ROBOT_IP} 是否可达"
  wait_for_controller_active "forward_command_controller_velocity" "Aubo 速度控制器" \
    "控制器未激活，请确认机械臂已 Running 且驱动日志无错误"
  echo "  Aubo 就绪"
else
  echo "[2/4] 跳过机械臂启动"
fi

# 3. 启动相机
if [[ "${SKIP_CAMERA}" != "true" ]]; then
  echo "[3/4] 启动 Gemini335 相机..."
  
  # 检查相机设备是否存在
  COLOR_DEVICE="${ARACHNE_CAMERA_COLOR_DEVICE:-/dev/video6}"
  DEPTH_DEVICE="${ARACHNE_CAMERA_DEPTH_DEVICE:-/dev/video0}"
  
  if [[ ! -e "${COLOR_DEVICE}" ]]; then
    echo "错误: 彩色相机设备不存在: ${COLOR_DEVICE}" >&2
    echo "  请检查 Gemini335 相机是否已连接" >&2
    echo "  或设置 ARACHNE_CAMERA_COLOR_DEVICE 环境变量指定正确设备" >&2
    exit 1
  fi
  
  if [[ ! -e "${DEPTH_DEVICE}" ]]; then
    echo "错误: 深度相机设备不存在: ${DEPTH_DEVICE}" >&2
    echo "  请检查 Gemini335 相机是否已连接" >&2
    echo "  或设置 ARACHNE_CAMERA_DEPTH_DEVICE 环境变量指定正确设备" >&2
    exit 1
  fi
  
  start_background "Gemini335 相机" \
    "${LOG_DIR}/04_gemini335.log" \
    ros2 launch arachne_sensors gemini335.launch.py

  wait_for_topic "/camera/color/image_raw" "相机彩色图像" \
    "相机节点启动失败，请检查日志: ${LOG_DIR}/04_gemini335.log"
  echo "  相机就绪"
else
  echo "[3/4] 跳过相机启动"
fi

# 4. 启动标定节点
echo "[4/4] 启动标定节点..."
start_background "标定节点" \
  "${LOG_DIR}/05_calibrator.log" \
  "${ROOT_DIR}/scripts/vision/apriltag_hand_eye_calibration.sh"

sleep 2
echo "  标定节点就绪"

# 5. 启动交互式 GUI（前台，阻塞）
echo ""
echo "========================================="
echo "  所有组件已就绪，启动交互式 GUI"
echo "  操作说明:"
echo "    SPACE  采集样本（移动机械臂后按）"
echo "    s      求解手眼矩阵（≥5 个样本）"
echo "    r      重置样本"
echo "    q      退出"
echo "========================================="
echo ""

exec "${ROOT_DIR}/scripts/vision/apriltag_hand_eye_interactive.sh"
