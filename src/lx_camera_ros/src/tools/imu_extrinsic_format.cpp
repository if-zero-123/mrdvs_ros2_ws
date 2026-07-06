#include "tools/imu_extrinsic_format.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace lx_camera_ros {
namespace tools {
namespace {

std::string FormatNumber(float value)
{
  const double normalized = std::abs(value) < 1e-7 ? 0.0 : value;
  std::ostringstream stream;
  stream << std::setprecision(9) << normalized;
  return stream.str();
}

std::string JoinValues(const ImuExtrinsicValues& values, size_t begin, size_t count)
{
  std::ostringstream stream;
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << FormatNumber(values[begin + i]);
  }
  return stream.str();
}

}  // namespace

std::string FormatImuExtrinsic(const ImuExtrinsicValues& values)
{
  std::ostringstream stream;
  stream << "raw: [" << JoinValues(values, 0, values.size()) << "]\n";
  stream << "rotation matrix:\n";
  for (size_t row = 0; row < 3; ++row) {
    stream << "  [" << JoinValues(values, row * 3, 3) << "]\n";
  }
  stream << "translation: [" << JoinValues(values, 9, 3) << "]\n";
  stream << "yaml candidate (confirm direction/unit before use):\n";
  stream << "r_il: [" << JoinValues(values, 0, 9) << "]\n";
  stream << "t_il: [" << JoinValues(values, 9, 3) << "]";
  return stream.str();
}

bool IsAllZeroExtrinsic(const ImuExtrinsicValues& values)
{
  for (const auto value : values) {
    if (std::abs(value) >= 1e-7) {
      return false;
    }
  }
  return true;
}

LX_OPEN_MODE SelectOpenMode(const std::string& target)
{
  return target.size() < 8 ? OPEN_BY_INDEX : OPEN_BY_IP;
}

}  // namespace tools
}  // namespace lx_camera_ros
