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

  // MRDVS PointCloud2 coordinates are float. Compare in that same precision
  // domain so a coordinate encoded as exactly the configured boundary is not
  // shifted just below a double-precision YAML value during promotion.
  const double blind_at_coordinate_precision = static_cast<double>(static_cast<float>(blind_m));
  if (
    blind_at_coordinate_precision > 0.0 &&
    distance_squared < blind_at_coordinate_precision * blind_at_coordinate_precision)
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
