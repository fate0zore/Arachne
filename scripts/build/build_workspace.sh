#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_WITH_GAZEBO="${ARACHNE_BUILD_WITH_GAZEBO:-auto}"

case "${BUILD_WITH_GAZEBO}" in
  auto|true|false) ;;
  *)
    echo "ARACHNE_BUILD_WITH_GAZEBO must be auto, true, or false. Current value: ${BUILD_WITH_GAZEBO}" >&2
    exit 1
    ;;
esac

set +u
export ARACHNE_ENV_NO_WORKSPACE=1
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
unset ARACHNE_ENV_NO_WORKSPACE
set -u

cd "${ROOT_DIR}"
BUILD_LOG_DIR="${ROOT_DIR}/log/build"
mkdir -p "${BUILD_LOG_DIR}"

arachne_remove_workspace_underlay "${ROOT_DIR}"

if [[ -z "${MAKEFLAGS:-}" && "$(uname -m)" == "aarch64" ]]; then
  export MAKEFLAGS="-j2"
fi

COLCON_ARGS=()
if [[ -n "${ARACHNE_COLCON_PARALLEL_WORKERS:-}" ]]; then
  COLCON_ARGS+=(--parallel-workers "${ARACHNE_COLCON_PARALLEL_WORKERS}")
elif [[ "$(uname -m)" == "aarch64" ]]; then
  COLCON_ARGS+=(--parallel-workers 2)
fi

if [[ -f build/aubo_description/cmake_install.cmake ]] \
  && grep -q 'ament_cmake_symlink_install' build/aubo_description/cmake_install.cmake; then
  echo "Removing stale symlink-install cache for aubo_description."
  rm -rf build/aubo_description install/aubo_description
fi

PACKAGES=(
  lslidar_msgs lslidar_driver \
  aubo_description scout_description dh_ag95_description \
  arachne_sim arachne_gripper arachne_hardware arachne_control arachne_moveit_config \
  arachne_nav arachne_operator arachne_sensors arachne_agent_bridge arachne_description \
  viewpoint_planner_msgs viewpoint_planner
)

has_cmake_package() {
  local package="$1"
  local probe_dir
  local status=0

  probe_dir="$(mktemp -d)"
  cmake -E chdir "${probe_dir}" cmake --find-package \
    -DNAME="${package}" \
    -DCOMPILER_ID=GNU \
    -DLANGUAGE=CXX \
    -DMODE=EXIST >/dev/null 2>&1 || status=$?
  rm -rf "${probe_dir}"
  return "${status}"
}

GAZEBO_DEPS_AVAILABLE=false
if has_cmake_package gz-msgs10 && has_cmake_package gz-transport13; then
  GAZEBO_DEPS_AVAILABLE=true
fi

if [[ "${BUILD_WITH_GAZEBO}" == "true" && "${GAZEBO_DEPS_AVAILABLE}" != "true" ]]; then
  echo "Gazebo build dependencies were not found: gz-msgs10 and/or gz-transport13." >&2
  echo "Install the ROS/Gazebo packages first, or rerun with ARACHNE_BUILD_WITH_GAZEBO=false." >&2
  exit 1
fi

if [[ "${BUILD_WITH_GAZEBO}" == "true" || ( "${BUILD_WITH_GAZEBO}" == "auto" && "${GAZEBO_DEPS_AVAILABLE}" == "true" ) ]]; then
  PACKAGES+=(arachne_gazebo arachne_demo)
else
  echo "Skipping Gazebo demo packages. Set ARACHNE_BUILD_WITH_GAZEBO=true after installing Gazebo deps."
fi

colcon --log-base "${BUILD_LOG_DIR}" build "${COLCON_ARGS[@]}" \
  --base-paths src third_party/Lslidar_ROS2_driver_C16_V4 \
  --packages-select \
  "${PACKAGES[@]}" \
  --cmake-args -DPython3_EXECUTABLE="${ARACHNE_SYSTEM_PYTHON}"
