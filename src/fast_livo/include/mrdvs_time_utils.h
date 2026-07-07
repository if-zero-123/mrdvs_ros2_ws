#ifndef MRDVS_TIME_UTILS_H_
#define MRDVS_TIME_UTILS_H_

#include <cmath>

namespace fast_livo
{

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
