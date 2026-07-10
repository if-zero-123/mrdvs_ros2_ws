#ifndef MRDVS_TIME_UTILS_H_
#define MRDVS_TIME_UTILS_H_

#include <algorithm>
#include <cmath>

namespace fast_livo
{

enum class MrdvsTimestampStatus
{
  kValid,
  kNonFinite,
  kUnknownUnit,
  kNegative,
  kTooLarge
};

struct MrdvsTimestampResult
{
  MrdvsTimestampStatus status;
  double relative_ms;

  bool valid() const { return status == MrdvsTimestampStatus::kValid; }
};

// The verified MRDVS driver contract defines raw_timestamp as absolute
// microseconds on the same timebase as the cloud header. This parser therefore
// validates their numerical relationship without assuming a Unix epoch or a
// minimum epoch magnitude; a low monotonic timebase is valid.
inline MrdvsTimestampResult parseMrdvsTimestamp(
  double raw_timestamp,
  double cloud_start_sec,
  double max_offset_ms)
{
  if (!std::isfinite(raw_timestamp) || !std::isfinite(cloud_start_sec) || !std::isfinite(max_offset_ms))
  {
    return {MrdvsTimestampStatus::kNonFinite, 0.0};
  }

  if (raw_timestamp <= 0.0 || cloud_start_sec <= 0.0 || max_offset_ms <= 0.0)
  {
    return {MrdvsTimestampStatus::kUnknownUnit, 0.0};
  }

  const double cloud_start_us = cloud_start_sec * 1.0e6;
  const double max_offset_us = max_offset_ms * 1.0e3;
  if (!std::isfinite(cloud_start_us) || !std::isfinite(max_offset_us))
  {
    return {MrdvsTimestampStatus::kUnknownUnit, 0.0};
  }

  const double relative_us = raw_timestamp - cloud_start_us;
  constexpr double kAbsoluteUsRecognitionWindow = 60.0e6;
  const double recognition_window_us = std::max(kAbsoluteUsRecognitionWindow, max_offset_us);
  if (!std::isfinite(relative_us) || std::fabs(relative_us) > recognition_window_us)
  {
    return {MrdvsTimestampStatus::kUnknownUnit, 0.0};
  }

  if (relative_us < 0.0)
  {
    return {MrdvsTimestampStatus::kNegative, 0.0};
  }

  if (relative_us > max_offset_us)
  {
    return {MrdvsTimestampStatus::kTooLarge, 0.0};
  }

  return {MrdvsTimestampStatus::kValid, relative_us / 1.0e3};
}

inline double mrdvsTimestampToRelativeMs(double raw_timestamp, double cloud_start_sec)
{
  if (!std::isfinite(raw_timestamp) || !std::isfinite(cloud_start_sec)) return 0.0;

  const double start_us = cloud_start_sec * 1.0e6;
  const double start_ns = cloud_start_sec * 1.0e9;
  double relative_us = 0.0;

  if (std::fabs(raw_timestamp - start_us) < 60.0e6)
  {
    relative_us = raw_timestamp - start_us;
  }
  else if (std::fabs(raw_timestamp - start_ns) < 60.0e9)
  {
    relative_us = (raw_timestamp - start_ns) / 1.0e3;
  }
  else if (std::fabs(raw_timestamp - cloud_start_sec) < 60.0)
  {
    relative_us = (raw_timestamp - cloud_start_sec) * 1.0e6;
  }
  else if (raw_timestamp >= 0.0 && raw_timestamp < 1.0e6)
  {
    relative_us = raw_timestamp;
  }
  else
  {
    return 0.0;
  }

  if (!std::isfinite(relative_us) || relative_us < 0.0) return 0.0;
  return relative_us / 1.0e3;
}

} // namespace fast_livo

#endif // MRDVS_TIME_UTILS_H_
