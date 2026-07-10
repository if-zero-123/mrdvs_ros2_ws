#include "mrdvs_time_utils.h"
#include "imu_time_filter.h"
#include "lio_update_guard.h"

#include <gtest/gtest.h>

#include <limits>

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

TEST(MrdvsTimeUtils, ParsesAbsoluteMicrosecondsAtFrameStart)
{
  constexpr double cloud_start_sec = 1000000.0;

  const auto result = fast_livo::parseMrdvsTimestamp(cloud_start_sec * 1.0e6, cloud_start_sec, 200.0);

  EXPECT_EQ(result.status, fast_livo::MrdvsTimestampStatus::kValid);
  EXPECT_TRUE(result.valid());
  EXPECT_DOUBLE_EQ(result.relative_ms, 0.0);
}

TEST(MrdvsTimeUtils, ParsesAbsoluteMicrosecondsWithinFrame)
{
  constexpr double cloud_start_sec = 1000000.0;

  const auto result = fast_livo::parseMrdvsTimestamp(cloud_start_sec * 1.0e6 + 12345.0, cloud_start_sec, 200.0);

  EXPECT_EQ(result.status, fast_livo::MrdvsTimestampStatus::kValid);
  EXPECT_NEAR(result.relative_ms, 12.345, 1e-9);
}

TEST(MrdvsTimeUtils, RejectsAbsoluteTimestampBeforeFrameStart)
{
  constexpr double cloud_start_sec = 1000000.0;

  const auto result = fast_livo::parseMrdvsTimestamp(cloud_start_sec * 1.0e6 - 1.0, cloud_start_sec, 200.0);

  EXPECT_EQ(result.status, fast_livo::MrdvsTimestampStatus::kNegative);
  EXPECT_FALSE(result.valid());
}

TEST(MrdvsTimeUtils, AcceptsConfiguredMaximumAndRejectsOnlyValuesAboveIt)
{
  constexpr double cloud_start_sec = 1000000.0;
  constexpr double cloud_start_us = cloud_start_sec * 1.0e6;

  const auto at_limit = fast_livo::parseMrdvsTimestamp(cloud_start_us + 200000.0, cloud_start_sec, 200.0);
  const auto above_limit = fast_livo::parseMrdvsTimestamp(cloud_start_us + 250000.0, cloud_start_sec, 200.0);

  ASSERT_EQ(at_limit.status, fast_livo::MrdvsTimestampStatus::kValid);
  EXPECT_DOUBLE_EQ(at_limit.relative_ms, 200.0);
  EXPECT_EQ(above_limit.status, fast_livo::MrdvsTimestampStatus::kTooLarge);
  EXPECT_FALSE(above_limit.valid());
}

TEST(MrdvsTimeUtils, RejectsRelativeOrWrongEpochValuesAsUnknownUnit)
{
  constexpr double cloud_start_sec = 1000000.0;

  EXPECT_EQ(
    fast_livo::parseMrdvsTimestamp(0.0, cloud_start_sec, 200.0).status,
    fast_livo::MrdvsTimestampStatus::kUnknownUnit);
  EXPECT_EQ(
    fast_livo::parseMrdvsTimestamp(250000.0, cloud_start_sec, 200.0).status,
    fast_livo::MrdvsTimestampStatus::kUnknownUnit);
  EXPECT_EQ(
    fast_livo::parseMrdvsTimestamp(1.0e9, cloud_start_sec, 200.0).status,
    fast_livo::MrdvsTimestampStatus::kUnknownUnit);
}

TEST(MrdvsTimeUtils, RejectsNonFiniteTimestamps)
{
  constexpr double cloud_start_sec = 1000000.0;

  EXPECT_EQ(
    fast_livo::parseMrdvsTimestamp(std::numeric_limits<double>::quiet_NaN(), cloud_start_sec, 200.0).status,
    fast_livo::MrdvsTimestampStatus::kNonFinite);
  EXPECT_EQ(
    fast_livo::parseMrdvsTimestamp(std::numeric_limits<double>::infinity(), cloud_start_sec, 200.0).status,
    fast_livo::MrdvsTimestampStatus::kNonFinite);
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
