#ifndef IMU_TIME_FILTER_H_
#define IMU_TIME_FILTER_H_

namespace fast_livo
{

enum class ImuTimestampAction
{
  kAccept,
  kDropWithoutReset,
  kResetStream
};

inline bool shouldDropImuTimestamp(double timestamp, double last_timestamp)
{
  return last_timestamp > 0.0 && timestamp <= last_timestamp;
}

inline bool shouldResetImuTimestampStream(double timestamp, double last_timestamp, double max_gap_sec)
{
  return last_timestamp > 0.0 && max_gap_sec > 0.0 &&
         (timestamp > last_timestamp + max_gap_sec || timestamp < last_timestamp - max_gap_sec);
}

inline ImuTimestampAction classifyImuTimestamp(
  double timestamp,
  double last_timestamp,
  double max_gap_sec)
{
  if (shouldResetImuTimestampStream(timestamp, last_timestamp, max_gap_sec))
  {
    return ImuTimestampAction::kResetStream;
  }
  if (shouldDropImuTimestamp(timestamp, last_timestamp))
  {
    return ImuTimestampAction::kDropWithoutReset;
  }
  return ImuTimestampAction::kAccept;
}

} // namespace fast_livo

#endif // IMU_TIME_FILTER_H_
