#!/usr/bin/env bash
# Arachne field helper only; not a stable runtime entrypoint (AGENTS.md).
# Exports Gemini 335 factory calibration (device-burned data, no re-calibration needed)
# from the Orbbec C++ SDK into config/gemini335_intrinsics/factory_calibration.json,
# which gemini335_v4l2_node consumes via the factory_intrinsics_file parameter.
#
# Usage:
#   ./scripts/vision/orbbec_factory_intrinsics.sh [--out <json-path>] [--sdk-dir <dir>] [--keep-sdk]
#
# Requires a connected Gemini 335. The SDK (~18 MB) is downloaded once into build/orbbec_sdk/.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SDK_VERSION="v1.10.27"
SDK_URL="https://github.com/orbbec/OrbbecSDK/releases/download/${SDK_VERSION}/OrbbecSDK_C_C%2B%2B_${SDK_VERSION}_20250925_0549823_linux_arm64_release.zip"
OUT_PATH="${ROOT_DIR}/config/gemini335_intrinsics/factory_calibration.json"
SDK_DIR="${ROOT_DIR}/build/orbbec_sdk"
KEEP_SDK=0

while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT_PATH="$2"; shift 2 ;;
        --sdk-dir) SDK_DIR="$2"; shift 2 ;;
        --keep-sdk) KEEP_SDK=1; shift ;;
        -h|--help)
            sed -n '2,16p' "$0"
            exit 0
            ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

if [ "$(uname -m)" != "aarch64" ]; then
    echo "This helper targets linux arm64 (Jetson); uname -m=$(uname -m)." >&2
    echo "Use the OrbbecSDK release zip for your platform and adjust SDK_URL." >&2
    exit 2
fi

# Locate the SDK include/lib directories (cope with the nested release layout).
find_sdk_root() {
    local dir="$1"
    if [ -f "${dir}/SDK/include/libobsensor/ObSensor.hpp" ] && [ -f "${dir}/SDK/lib/libOrbbecSDK.so" ]; then
        echo "${dir}"
    else
        find "${dir}" -maxdepth 4 -type d -name OrbbecSDK_v* 2>/dev/null | head -1 || true
    fi
}

SDK_ROOT="$(find_sdk_root "${SDK_DIR}")"
if [ -z "${SDK_ROOT}" ] || [ ! -f "${SDK_ROOT}/SDK/lib/libOrbbecSDK.so" ]; then
    echo "OrbbecSDK not found under ${SDK_DIR}; downloading ${SDK_VERSION} (linux arm64) ..."
    mkdir -p "${SDK_DIR}"
    curl -sL --retry 3 -o "${SDK_DIR}/sdk.zip" "${SDK_URL}"
    (cd "${SDK_DIR}" && unzip -o -q sdk.zip)
    if [ "${KEEP_SDK}" -ne 1 ]; then
        rm -f "${SDK_DIR}/sdk.zip"
    fi
    SDK_ROOT="$(find_sdk_root "${SDK_DIR}")"
fi
if [ -z "${SDK_ROOT}" ] || [ ! -f "${SDK_ROOT}/SDK/lib/libOrbbecSDK.so" ]; then
    echo "Failed to locate OrbbecSDK under ${SDK_DIR}. Check the download." >&2
    exit 1
fi
echo "OrbbecSDK: ${SDK_ROOT}"

BIN="${ROOT_DIR}/build/orbbec_dump_factory_intrinsics"
mkdir -p "${ROOT_DIR}/build"
g++ -std=c++17 \
    -I "${SDK_ROOT}/SDK/include" \
    "${ROOT_DIR}/scripts/vision/orbbec_dump_factory_intrinsics.cpp" \
    -L "${SDK_ROOT}/SDK/lib" -lOrbbecSDK \
    -Wl,-rpath,"${SDK_ROOT}/SDK/lib" \
    -o "${BIN}"
echo "Built ${BIN}"

mkdir -p "$(dirname "${OUT_PATH}")"
if ! "${BIN}" "${OUT_PATH}"; then
    RULES="/etc/udev/rules.d/99-obsensor-libusb.rules"
    if [ ! -r /dev/bus/usb ]; then
        echo "Hint: USB access missing (/dev/bus/usb not accessible)." >&2
        echo "  sudo ${SDK_ROOT}/Script/install_udev_rules.sh   # then re-plug the camera" >&2
    elif [ ! -f "${RULES}" ] || ! grep -q "0800" "${RULES}" 2>/dev/null; then
        echo "Hint: camera detected but USB device permission denied (OB_USB_STATUS_ACCESS)." >&2
        echo "  The Orbbec udev rules are not installed. Install them (covers Gemini 335 PID 0x0800)," >&2
        echo "  then re-plug the camera and retry:" >&2
        echo "  sudo ${SDK_ROOT}/Script/install_udev_rules.sh" >&2
    else
        echo "Hint: udev rules present but export still failed; check the camera connection and" >&2
        echo "  that no other process (e.g. the V4L2 driver) is using the device." >&2
    fi
    exit 1
fi
echo "Done. Now relaunch with:"
echo "  ros2 launch arachne_sensors gemini335.launch.py factory_intrinsics_file:=${OUT_PATH}"
