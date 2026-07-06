#include "tools/imu_extrinsic_format.h"
#include "utils/dynamic_link.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
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
            << "  " << program << " 192.168.1.10\n";
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

void WarnSdkError(const DcLib& lib, const std::string& action, LX_STATE state)
{
  std::cerr << "Warning: ";
  PrintSdkError(lib, action, state);
}

void ConfigureImuBeforeRead(const DcLib& lib, DcHandle handle)
{
  LX_STATE state = (lib.DcSetBoolValue)(handle, LX_BOOL_ENABLE_IMU, true);
  if (state != LX_SUCCESS && state != LX_W_NOT_SUPPORT) {
    WarnSdkError(lib, "DcSetBoolValue(LX_BOOL_ENABLE_IMU)", state);
  }

  state = (lib.DcSetIntValue)(handle, LX_INT_IMU_ACCELERATION_LEVEL, 0);
  if (state != LX_SUCCESS && state != LX_W_NOT_SUPPORT) {
    WarnSdkError(lib, "DcSetIntValue(LX_INT_IMU_ACCELERATION_LEVEL)", state);
  }

  state = (lib.DcSetIntValue)(handle, LX_INT_IMU_ANGULAR_RANGE_LEVEL, 0);
  if (state != LX_SUCCESS && state != LX_W_NOT_SUPPORT) {
    WarnSdkError(lib, "DcSetIntValue(LX_INT_IMU_ANGULAR_RANGE_LEVEL)", state);
  }
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
  ConfigureImuBeforeRead(lib, handle);

  float* raw_extrinsic = nullptr;
  state = (lib.DcGetPtrValue)(
    handle, LX_PTR_IMU_EXTRIC_PARAM, reinterpret_cast<void**>(&raw_extrinsic));
  if (state != LX_SUCCESS) {
    PrintSdkError(lib, "DcGetPtrValue(LX_PTR_IMU_EXTRIC_PARAM)", state);
    (lib.DcCloseDevice)(handle);
    DisDynamicLink(&lib);
    return 1;
  }

  if (!raw_extrinsic) {
    std::cerr << "DcGetPtrValue(LX_PTR_IMU_EXTRIC_PARAM) returned null."
              << std::endl;
    (lib.DcCloseDevice)(handle);
    DisDynamicLink(&lib);
    return 1;
  }

  lx_camera_ros::tools::ImuExtrinsicValues values;
  std::copy(raw_extrinsic, raw_extrinsic + values.size(), values.begin());

  std::cout << "\nLX_PTR_IMU_EXTRIC_PARAM:\n"
            << lx_camera_ros::tools::FormatImuExtrinsic(values) << "\n";
  if (lx_camera_ros::tools::IsAllZeroExtrinsic(values)) {
    std::cout << "\nWarning: SDK returned an all-zero IMU extrinsic. This is "
                 "not a valid rotation matrix; the device or firmware may not "
                 "provide this calibration through LX_PTR_IMU_EXTRIC_PARAM."
              << "\n";
  }
  std::cout << "\nNote: SDK only documents this as IMU extrinsic. Confirm the "
               "transform direction and translation unit before using it in "
               "FAST-LIO2."
            << std::endl;

  (lib.DcCloseDevice)(handle);
  DisDynamicLink(&lib);
  return 0;
}
