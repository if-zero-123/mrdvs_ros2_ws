#include "mrdvs_time_utils.h"
#include "imu_time_filter.h"
#include "lio_update_guard.h"

#include <gtest/gtest.h>

TEST(MrdvsTimeUtils, ConvertsAbsoluteMicrosecondsToFrameRelativeMilliseconds)
{
  const double cloud_start_sec = 1000000.0;
  const double point_timestamp_us = 1000000000000.0 + 12345.0;

  EXPECT_NEAR(fast_livo::mrdvsTimestampToRelativeMs(point_timestamp_us, cloud_start_sec), 12.345, 1e-9);
}

TEST(MrdvsTimeUtils, ConvertsOffsetMicrosecondsToMilliseconds)
{
  const double cloud_start_sec = 1000000.0;
  const double point_offset_us = 2345.0;

  EXPECT_NEAR(fast_livo::mrdvsTimestampToRelativeMs(point_offset_us, cloud_start_sec), 2.345, 1e-9);
}

TEST(MrdvsTimeUtils, RejectsNegativeRelativeTimes)
{
  const double cloud_start_sec = 1000000.0;
  const double timestamp_before_frame_us = 1000000000000.0 - 1.0;

  EXPECT_DOUBLE_EQ(fast_livo::mrdvsTimestampToRelativeMs(timestamp_before_frame_us, cloud_start_sec), 0.0);
}

TEST(ImuTimeFilter, DropsOnlyNonIncreasingTimestamps)
{
  EXPECT_FALSE(fast_livo::shouldDropImuTimestamp(10.0, -1.0));
  EXPECT_FALSE(fast_livo::shouldDropImuTimestamp(10.005, 10.0));

  EXPECT_TRUE(fast_livo::shouldDropImuTimestamp(10.0, 10.0));
  EXPECT_TRUE(fast_livo::shouldDropImuTimestamp(9.999, 10.0));
}

TEST(ImuTimeFilter, ResetsOnLargeForwardJump)
{
  EXPECT_FALSE(fast_livo::shouldResetImuTimestampStream(10.1, -1.0, 0.2));
  EXPECT_FALSE(fast_livo::shouldResetImuTimestampStream(10.19, 10.0, 0.2));

  EXPECT_TRUE(fast_livo::shouldResetImuTimestampStream(10.201, 10.0, 0.2));
}

TEST(LioUpdateGuard, RejectsFramesWithoutEffectiveConstraints)
{
  EXPECT_FALSE(fast_livo::hasUsableLioConstraints(0));
  EXPECT_TRUE(fast_livo::hasUsableLioConstraints(1));
}
