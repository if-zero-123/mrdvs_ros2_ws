#include "tools/imu_extrinsic_format.h"
#include "utils/dynamic_link.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string ErrorText(const DcLib& lib, LX_STATE state)
{
  if (lib.DcGetErrorString) {
    const char* message = (lib.DcGetErrorString)(state);
    if (message) {
      return message;
    }
  }
  return "unknown SDK error";
}

void PrintSdkError(const DcLib& lib, const std::string& action, LX_STATE state)
{
  std::cerr << action << " failed, code: " << static_cast<int>(state)
            << ", message: " << ErrorText(lib, state) << std::endl;
}

template <size_t N>
std::string SafeString(const char (&value)[N])
{
  return std::string(value, ::strnlen(value, N));
}

void PrintUsage(const char* program)
{
  std::cout << "Usage: " << program << " [device_index_or_ip]\n"
            << "\n"
            << "Examples:\n"
            << "  " << program << "\n"
            << "  " << program << " 0\n"
            << "  " << program << " 192.168.100.82\n";
}

void PrintDeviceInfo(const LxDeviceInfo& info)
{
  std::cout << "device name: " << SafeString(info.name) << "\n"
            << "device id: " << SafeString(info.id) << "\n"
            << "device ip: " << SafeString(info.ip) << "\n"
            << "device sn: " << SafeString(info.sn) << "\n"
            << "device firmware version: " << SafeString(info.firmware_ver) << "\n"
            << "device algorithm version: " << SafeString(info.algor_ver) << "\n";
}

void PrintYamlArray(const std::string& name, const float* values, size_t count,
                    double scale = 1.0)
{
  std::cout << name << ": [";
  for (size_t i = 0; i < count; ++i) {
    if (i) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(9)
              << static_cast<double>(values[i]) * scale;
  }
  std::cout << "]\n";
}

void PrintYamlArray(const std::string& name, const std::array<double, 9>& values)
{
  std::cout << name << ": [";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(9) << values[i];
  }
  std::cout << "]\n";
}

void PrintIntrinsic(const std::string& title, const LxIntrinsicParameters* calib)
{
  std::cout << "\n" << title << "\n";
  if (!calib) {
    std::cout << "  not available\n";
    return;
  }

  const float* k = calib->intrinsics;
  std::cout << "K: [" << std::fixed << std::setprecision(9)
            << k[0] << ", " << k[1] << ", " << k[2] << ", "
            << k[3] << ", " << k[4] << ", " << k[5] << ", "
            << k[6] << ", " << k[7] << ", " << k[8] << "]\n";
  std::cout << "fx: " << k[0] << "\n"
            << "fy: " << k[4] << "\n"
            << "cx: " << k[2] << "\n"
            << "cy: " << k[5] << "\n"
            << "distortion_model: " << static_cast<int>(calib->distortion_model)
            << "\n";
  std::cout << "D: [";
  for (size_t i = 0; i < calib->num_distortion_coeffs; ++i) {
    if (i) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(9)
              << calib->distortion_coeffs[i];
  }
  std::cout << "]\n";
}

bool ReadIntrinsic(const DcLib& lib, DcHandle handle, int cmd,
                   LxIntrinsicParameters** calib, const std::string& name)
{
  LX_STATE state = (lib.DcGetPtrValue)(handle, cmd, reinterpret_cast<void**>(calib));
  if (state != LX_SUCCESS) {
    PrintSdkError(lib, "DcGetPtrValue(" + name + ")", state);
    return false;
  }
  return *calib != nullptr;
}

bool ReadExtrinsic(const DcLib& lib, DcHandle handle, int cmd, float** values,
                   const std::string& name)
{
  LX_STATE state = (lib.DcGetPtrValue)(handle, cmd, reinterpret_cast<void**>(values));
  if (state != LX_SUCCESS) {
    PrintSdkError(lib, "DcGetPtrValue(" + name + ")", state);
    return false;
  }
  if (!*values) {
    std::cerr << "DcGetPtrValue(" << name << ") returned null." << std::endl;
    return false;
  }
  return true;
}

void PrintRawExtrinsic(const std::string& title, const float* values)
{
  std::cout << "\n" << title << "\n";
  PrintYamlArray("R", values, 9);
  PrintYamlArray("T_m", values + 9, 3, 0.001);
  std::cout << "matrix_m:\n";
  for (int r = 0; r < 3; ++r) {
    std::cout << "  [";
    for (int c = 0; c < 3; ++c) {
      if (c) {
        std::cout << ", ";
      }
      std::cout << std::fixed << std::setprecision(9) << values[r * 3 + c];
    }
    std::cout << ", " << std::fixed << std::setprecision(9)
              << values[9 + r] * 0.001 << "]\n";
  }
  std::cout << "  [0.000000000, 0.000000000, 0.000000000, 1.000000000]\n";
}

std::array<double, 9> Transpose3x3(const float* r)
{
  return {r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]};
}

std::array<double, 3> InverseTranslationMeters(const float* values)
{
  const double tx = values[9] * 0.001;
  const double ty = values[10] * 0.001;
  const double tz = values[11] * 0.001;
  return {
    -(values[0] * tx + values[3] * ty + values[6] * tz),
    -(values[1] * tx + values[4] * ty + values[7] * tz),
    -(values[2] * tx + values[5] * ty + values[8] * tz),
  };
}

void PrintInverseForFastLivo(const float* values)
{
  const auto r_inv = Transpose3x3(values);
  const auto t_inv = InverseTranslationMeters(values);
  std::cout << "\nFAST-LIVO2 candidate Rcl/Pcl if SDK raw 3D extrinsic is RGB->ToF:\n";
  PrintYamlArray("Rcl", r_inv);
  std::cout << "Pcl: [" << std::fixed << std::setprecision(9)
            << t_inv[0] << ", " << t_inv[1] << ", " << t_inv[2] << "]\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc > 2) {
    PrintUsage(argv[0]);
    return 2;
  }

  const std::string target =
    argc == 2 ? std::string(argv[1]) : std::string("0");
  if (target == "-h" || target == "--help") {
    PrintUsage(argv[0]);
    return 0;
  }

  DcLib lib;
  if (!DynamicLink(&lib)) {
    std::cerr << "Load MRDVS SDK dynamic library failed." << std::endl;
    return 1;
  }

  if (lib.DcSetInfoOutput) {
    (lib.DcSetInfoOutput)(1, true, "");
  }

  std::cout << "MRDVS SDK API version: " << (lib.DcGetApiVersion)() << "\n";

  LxDeviceInfo* device_list = nullptr;
  int device_count = 0;
  LX_STATE state = (lib.DcGetDeviceList)(&device_list, &device_count);
  if (state != LX_SUCCESS) {
    PrintSdkError(lib, "DcGetDeviceList", state);
    DisDynamicLink(&lib);
    return 1;
  }

  if (device_count <= 0) {
    std::cerr << "No MRDVS device found." << std::endl;
    DisDynamicLink(&lib);
    return 1;
  }

  std::cout << "found device count: " << device_count << "\n";
  for (int i = 0; i < device_count; ++i) {
    std::cout << "  [" << i << "] " << SafeString(device_list[i].name)
              << " ip=" << SafeString(device_list[i].ip)
              << " sn=" << SafeString(device_list[i].sn) << "\n";
  }

  DcHandle handle = 0;
  LxDeviceInfo opened_info{};
  const LX_OPEN_MODE open_mode = lx_camera_ros::tools::SelectOpenMode(target);
  state = (lib.DcOpenDevice)(open_mode, target.c_str(), &handle, &opened_info);
  if (state != LX_SUCCESS) {
    PrintSdkError(lib, "DcOpenDevice", state);
    if (state == LX_E_CTRL_PERMISS_ERROR) {
      std::cerr << "Device is probably already opened by lx_camera_node or "
                   "another SDK process. Stop that process and retry."
                << std::endl;
    }
    DisDynamicLink(&lib);
    return 1;
  }

  std::cout << "open device success\n";
  PrintDeviceInfo(opened_info);

  LxIntrinsicParameters* rgb_intrinsic = nullptr;
  LxIntrinsicParameters* tof_intrinsic = nullptr;
  ReadIntrinsic(lib, handle, LX_PTR_2D_INTRINSIC_PARAMETERS, &rgb_intrinsic,
                "LX_PTR_2D_INTRINSIC_PARAMETERS");
  ReadIntrinsic(lib, handle, LX_PTR_3D_INTRINSIC_PARAMETERS, &tof_intrinsic,
                "LX_PTR_3D_INTRINSIC_PARAMETERS");
  PrintIntrinsic("RGB intrinsic from LX_PTR_2D_INTRINSIC_PARAMETERS:", rgb_intrinsic);
  PrintIntrinsic("ToF intrinsic from LX_PTR_3D_INTRINSIC_PARAMETERS:", tof_intrinsic);

  float* tof_rgb_extrinsic = nullptr;
  if (ReadExtrinsic(lib, handle, LX_PTR_3D_EXTRIC_PARAM, &tof_rgb_extrinsic,
                    "LX_PTR_3D_EXTRIC_PARAM")) {
    PrintRawExtrinsic(
      "SDK raw ToF/RGB extrinsic from LX_PTR_3D_EXTRIC_PARAM "
      "(translation converted mm->m):",
      tof_rgb_extrinsic);
    PrintInverseForFastLivo(tof_rgb_extrinsic);
  }

  float* imu_extrinsic = nullptr;
  if (ReadExtrinsic(lib, handle, LX_PTR_IMU_EXTRIC_PARAM, &imu_extrinsic,
                    "LX_PTR_IMU_EXTRIC_PARAM")) {
    PrintRawExtrinsic(
      "SDK raw ToF/IMU extrinsic from LX_PTR_IMU_EXTRIC_PARAM "
      "(translation converted mm->m):",
      imu_extrinsic);
  }

  (lib.DcCloseDevice)(handle);
  DisDynamicLink(&lib);
  return 0;
}
