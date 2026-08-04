// Arachne field helper only; not a stable runtime entrypoint (AGENTS.md).
// Reads Gemini 335 factory calibration (device-burned data, no re-calibration needed) via the
// Orbbec C++ SDK and writes config/gemini335_intrinsics/factory_calibration.json, which is
// consumed by gemini335_v4l2_node (param factory_intrinsics_file).
//
// Build + run: scripts/vision/orbbec_factory_intrinsics.sh  (needs a connected Gemini 335)
//   or manually: g++ -std=c++17 -I <sdk>/include orbbec_dump_factory_intrinsics.cpp \
//                    -L <sdk>/lib -lOrbbecSDK -Wl,-rpath,<sdk>/lib -o orbbec_dump_factory_intrinsics
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

#include <libobsensor/ObSensor.hpp>

static std::string fmt(float v) {
    std::ostringstream oss;
    oss << std::setprecision(9) << v;
    return oss.str();
}

static std::string iso_now() {
    std::time_t t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
    return buf;
}

static void print_intrinsic(const char *label, const OBCameraIntrinsic &in) {
    std::cout << "  " << label << " size=" << in.width << "x" << in.height
              << " fx=" << fmt(in.fx) << " fy=" << fmt(in.fy)
              << " cx=" << fmt(in.cx) << " cy=" << fmt(in.cy) << "\n";
}

static std::string distortion_json(const OBCameraDistortion &d) {
    // SDK field order: k1..k6, p1, p2 (matches factory_calibration.json schema)
    return "[" + fmt(d.k1) + ", " + fmt(d.k2) + ", " + fmt(d.k3) + ", " + fmt(d.k4) + ", "
           + fmt(d.k5) + ", " + fmt(d.k6) + ", " + fmt(d.p1) + ", " + fmt(d.p2) + "]";
}

static std::string stream_json(const OBCameraIntrinsic &in, const OBCameraDistortion &dist,
                               const char *model, bool mirrored) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "    \"image_size\": [" << static_cast<int>(in.width) << ", "
        << static_cast<int>(in.height) << "],\n";
    oss << "    \"intrinsic\": {\"fx\": " << fmt(in.fx) << ", \"fy\": " << fmt(in.fy)
        << ", \"cx\": " << fmt(in.cx) << ", \"cy\": " << fmt(in.cy) << "},\n";
    oss << "    \"distortion\": " << distortion_json(dist) << ",\n";
    oss << "    \"distortion_model\": \"" << model << "\",\n";
    oss << "    \"is_mirrored\": " << (mirrored ? "true" : "false") << "\n";
    oss << "  }";
    return oss.str();
}

int main(int argc, char **argv) {
    const char *out_path =
        argc > 1 ? argv[1] : "config/gemini335_intrinsics/factory_calibration.json";
    // libusb needs /dev/bus/usb; check first because OrbbecSDK crashes internally when
    // USB is unavailable (e.g. containers without device access).
    if (access("/dev/bus/usb", R_OK | X_OK) != 0) {
        std::cerr << "USB access unavailable (/dev/bus/usb not accessible). Connect the Gemini 335 "
                     "and install the udev rules (sudo <sdk>/Script/install_udev_rules.sh).\n";
        return 1;
    }
    try {
        ob::Context ctx;
        auto device_list = ctx.queryDeviceList();
        if (device_list->deviceCount() == 0) {
            std::cerr << "No Orbbec device found. Connect the Gemini 335 and retry.\n";
            return 1;
        }
        auto device = device_list->getDevice(0);
        auto info = device->getDeviceInfo();
        if (info != nullptr) {
            std::cout << "Device: " << (info->name() ? info->name() : "?")
                      << "  SN: " << (info->serialNumber() ? info->serialNumber() : "?") << "\n";
        }

        auto calib_list = device->getCalibrationCameraParamList();
        const uint32_t n = calib_list != nullptr ? calib_list->count() : 0;
        if (n == 0) {
            std::cerr << "Device has no calibration parameter list.\n";
            return 1;
        }
        std::cout << "Calibration parameter groups: " << n << "\n";
        for (uint32_t i = 0; i < n; ++i) {
            OBCameraParam p = calib_list->getCameraParam(i);
            std::cout << "  [" << i << "] rgb " << p.rgbIntrinsic.width << "x"
                      << p.rgbIntrinsic.height << " / depth " << p.depthIntrinsic.width << "x"
                      << p.depthIntrinsic.height
                      << " mirrored=" << (p.isMirrored ? "true" : "false") << "\n";
        }

        // Prefer the 640x480 color + 640x480 depth group (matches driver defaults).
        int selected = -1;
        for (uint32_t i = 0; i < n; ++i) {
            OBCameraParam p = calib_list->getCameraParam(i);
            if (p.rgbIntrinsic.width == 640 && p.rgbIntrinsic.height == 480 &&
                p.depthIntrinsic.width == 640 && p.depthIntrinsic.height == 480) {
                selected = static_cast<int>(i);
                break;
            }
        }
        if (selected < 0) {
            std::cerr << "WARNING: no 640x480/640x480 group; using group 0. Verify the output!\n";
            selected = 0;
        }

        OBCameraParam p = calib_list->getCameraParam(static_cast<uint32_t>(selected));
        std::cout << "Selected group [" << selected << "]:\n";
        print_intrinsic("rgb  ", p.rgbIntrinsic);
        print_intrinsic("depth", p.depthIntrinsic);

        std::ofstream out(out_path);
        if (!out) {
            std::cerr << "Cannot write " << out_path << "\n";
            return 1;
        }
        out << "{\n";
        out << "  \"format\": \"arachne_gemini335_factory_calibration_v1\",\n";
        out << "  \"sensor\": \"Gemini 335\",\n";
        out << "  \"exported_at\": \"" << iso_now() << "\",\n";
        out << "  \"sdk_version\": \"" << ob::Version::getMajor() << "." << ob::Version::getMinor()
            << "." << ob::Version::getPatch() << "\",\n";
        out << "  \"source\": \"OrbbecSDK getCalibrationCameraParamList() (device-burned data)\",\n";
        out << "  \"streams\": {\n";
        out << "    \"color\": " << stream_json(p.rgbIntrinsic, p.rgbDistortion,
                                                "modified_brown_conrady", p.isMirrored)
            << ",\n";
        out << "    \"depth\": " << stream_json(p.depthIntrinsic, p.depthDistortion,
                                                "modified_brown_conrady", p.isMirrored)
            << "\n";
        out << "  },\n";
        out << "  \"transform_depth_to_color\": {\n";
        out << "    \"rot\": [" << fmt(p.transform.rot[0]) << ", " << fmt(p.transform.rot[1])
            << ", " << fmt(p.transform.rot[2]) << ", " << fmt(p.transform.rot[3]) << ", "
            << fmt(p.transform.rot[4]) << ", " << fmt(p.transform.rot[5]) << ", "
            << fmt(p.transform.rot[6]) << ", " << fmt(p.transform.rot[7]) << ", "
            << fmt(p.transform.rot[8]) << "],\n";
        out << "    \"trans\": [" << fmt(p.transform.trans[0]) << ", "
            << fmt(p.transform.trans[1]) << ", " << fmt(p.transform.trans[2]) << "]\n";
        out << "  }\n";
        out << "}\n";
        out.close();
        std::cout << "Wrote " << out_path << "\n";
    } catch (const ob::Error &e) {
        // ob::Error is NOT derived from std::exception; catch it separately to avoid terminate.
        const char *msg = e.getMessage();
        std::cerr << "ERROR (Orbbec SDK): " << (msg ? msg : "?") << "\n";
        if (msg != nullptr && std::string(msg).find("OB_USB_STATUS_ACCESS") != std::string::npos) {
            std::cerr << "Hint: USB device permission denied. Install the Orbbec udev rules "
                         "(sudo <sdk>/Script/install_udev_rules.sh), re-plug the camera, and retry.\n";
        }
        return 2;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 2;
    }
    return 0;
}
