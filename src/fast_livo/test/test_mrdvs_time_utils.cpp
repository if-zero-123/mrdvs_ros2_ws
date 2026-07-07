#include "mrdvs_time_utils.h"

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
