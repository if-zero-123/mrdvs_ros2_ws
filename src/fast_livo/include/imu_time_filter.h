#ifndef IMU_TIME_FILTER_H_
#define IMU_TIME_FILTER_H_

namespace fast_livo
{

inline bool shouldDropImuTimestamp(double timestamp, double last_timestamp)
{
  return last_timestamp > 0.0 && timestamp <= last_timestamp;
}

inline bool shouldResetImuTimestampStream(double timestamp, double last_timestamp, double max_gap_sec)
{
  return last_timestamp > 0.0 && max_gap_sec > 0.0 && timestamp > last_timestamp + max_gap_sec;
}

} // namespace fast_livo

#endif // IMU_TIME_FILTER_H_
