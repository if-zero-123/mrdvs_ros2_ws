#ifndef IMU_TIME_FILTER_H_
#define IMU_TIME_FILTER_H_

namespace fast_livo
{

inline bool shouldDropImuTimestamp(double timestamp, double last_timestamp)
{
  return last_timestamp > 0.0 && timestamp <= last_timestamp;
}

} // namespace fast_livo

#endif // IMU_TIME_FILTER_H_
