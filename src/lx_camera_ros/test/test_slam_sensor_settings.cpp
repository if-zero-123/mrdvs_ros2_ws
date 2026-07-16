#include "lx_camera/slam_sensor_settings.h"
#include <gtest/gtest.h>

TEST(SlamSensorSettings, AcceptsOnlySdkAngularRangeLevels)
{
  for (int level = 0; level <= 4; ++level)
    EXPECT_TRUE(lx_camera_ros::isValidImuAngularRangeLevel(level));
  EXPECT_FALSE(lx_camera_ros::isValidImuAngularRangeLevel(-1));
  EXPECT_FALSE(lx_camera_ros::isValidImuAngularRangeLevel(5));
}

TEST(SlamSensorSettings, RequiresReadbackToMatchRequestedValue)
{
  EXPECT_TRUE(lx_camera_ros::criticalSensorSettingMatches(2, 2));
  EXPECT_FALSE(lx_camera_ros::criticalSensorSettingMatches(2, 1));
}

TEST(SlamSensorSettings, RequiresOpticalXyzCoordinateForSlam)
{
  EXPECT_TRUE(lx_camera_ros::isRequiredSlamXyzCoordinate(0));
  EXPECT_FALSE(lx_camera_ros::isRequiredSlamXyzCoordinate(1));
  EXPECT_FALSE(lx_camera_ros::isRequiredSlamXyzCoordinate(-1));
}
