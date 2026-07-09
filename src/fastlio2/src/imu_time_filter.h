#ifndef FASTLIO2_IMU_TIME_FILTER_H_
#define FASTLIO2_IMU_TIME_FILTER_H_

namespace fastlio2
{

inline bool shouldDropImuTimestamp(double timestamp, double last_timestamp)
{
    return last_timestamp > 0.0 && timestamp <= last_timestamp;
}

} // namespace fastlio2

#endif // FASTLIO2_IMU_TIME_FILTER_H_
