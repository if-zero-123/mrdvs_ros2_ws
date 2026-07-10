#ifndef MRDVS_PREPROCESS_UTILS_H_
#define MRDVS_PREPROCESS_UTILS_H_

#include <cmath>
#include <cstddef>

namespace fast_livo
{

enum class MrdvsPointStatus
{
  kValid,
  kNonFinite,
  kZeroOrNear
};

inline MrdvsPointStatus classifyMrdvsPoint(double x, double y, double z, double blind_m)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
  {
    return MrdvsPointStatus::kNonFinite;
  }

  const double distance_squared = x * x + y * y + z * z;
  if (distance_squared == 0.0)
  {
    return MrdvsPointStatus::kZeroOrNear;
  }

  if (blind_m > 0.0 && distance_squared < blind_m * blind_m)
  {
    return MrdvsPointStatus::kZeroOrNear;
  }

  return MrdvsPointStatus::kValid;
}

inline bool hasEnoughPointsForLidarBuffer(std::size_t point_count)
{
  return point_count >= 2U;
}

template <typename PointCloudDeque, typename HeaderTimeDeque>
inline void clearPairedLidarBuffers(
  PointCloudDeque &point_cloud_buffer,
  HeaderTimeDeque &header_time_buffer,
  bool &lidar_pushed)
{
  point_cloud_buffer.clear();
  header_time_buffer.clear();
  lidar_pushed = false;
}

} // namespace fast_livo

#endif // MRDVS_PREPROCESS_UTILS_H_
