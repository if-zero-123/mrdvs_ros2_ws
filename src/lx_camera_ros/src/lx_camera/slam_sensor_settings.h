#ifndef LX_CAMERA_ROS_SLAM_SENSOR_SETTINGS_H_
#define LX_CAMERA_ROS_SLAM_SENSOR_SETTINGS_H_

namespace lx_camera_ros {

constexpr int kSlamOpticalXyzCoordinate = 0;

inline bool isValidImuAngularRangeLevel(int level)
{
  return level >= 0 && level <= 4;
}

inline bool isRequiredSlamXyzCoordinate(int coordinate)
{
  return coordinate == kSlamOpticalXyzCoordinate;
}

inline bool criticalSensorSettingMatches(int requested, int actual)
{
  return requested == actual;
}

}  // namespace lx_camera_ros

#endif  // LX_CAMERA_ROS_SLAM_SENSOR_SETTINGS_H_
