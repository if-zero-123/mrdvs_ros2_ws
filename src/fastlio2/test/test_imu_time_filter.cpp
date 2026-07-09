#include <gtest/gtest.h>

#include "imu_time_filter.h"

TEST(ImuTimeFilter, DropsOnlyNonIncreasingTimestamps)
{
  EXPECT_FALSE(fastlio2::shouldDropImuTimestamp(10.0, -1.0));
  EXPECT_FALSE(fastlio2::shouldDropImuTimestamp(10.005, 10.0));

  EXPECT_TRUE(fastlio2::shouldDropImuTimestamp(10.0, 10.0));
  EXPECT_TRUE(fastlio2::shouldDropImuTimestamp(9.999, 10.0));
}
