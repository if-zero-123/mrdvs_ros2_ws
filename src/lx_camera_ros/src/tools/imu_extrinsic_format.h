#ifndef LX_CAMERA_ROS_TOOLS_IMU_EXTRINSIC_FORMAT_H_
#define LX_CAMERA_ROS_TOOLS_IMU_EXTRINSIC_FORMAT_H_

#include "lx_camera_define.h"

#include <array>
#include <string>

namespace lx_camera_ros {
namespace tools {

using ImuExtrinsicValues = std::array<float, 12>;

std::string FormatImuExtrinsic(const ImuExtrinsicValues& values);

bool IsAllZeroExtrinsic(const ImuExtrinsicValues& values);

LX_OPEN_MODE SelectOpenMode(const std::string& target);

}  // namespace tools
}  // namespace lx_camera_ros

#endif  // LX_CAMERA_ROS_TOOLS_IMU_EXTRINSIC_FORMAT_H_
