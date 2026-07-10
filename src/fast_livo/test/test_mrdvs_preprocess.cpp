#include "mrdvs_preprocess_utils.h"
#include "preprocess.h"

#include <gtest/gtest.h>

#include <deque>
#include <limits>
#include <stdexcept>

TEST(MrdvsPreprocessUtils, ClassifiesZeroNearBoundaryAndFarPoints)
{
  using fast_livo::MrdvsPointStatus;

  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.0, 0.0, 0.0, 0.19), MrdvsPointStatus::kZeroOrNear);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.18, 0.0, 0.0, 0.19), MrdvsPointStatus::kZeroOrNear);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.19, 0.0, 0.0, 0.19), MrdvsPointStatus::kValid);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.20, 0.0, 0.0, 0.19), MrdvsPointStatus::kValid);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.0, 0.0, 0.0, 0.0), MrdvsPointStatus::kZeroOrNear);
}

TEST(MrdvsPreprocessUtils, RejectsEveryNonFiniteCoordinate)
{
  using fast_livo::MrdvsPointStatus;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_EQ(fast_livo::classifyMrdvsPoint(nan, 1.0, 1.0, 0.19), MrdvsPointStatus::kNonFinite);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(1.0, nan, 1.0, 0.19), MrdvsPointStatus::kNonFinite);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(1.0, 1.0, nan, 0.19), MrdvsPointStatus::kNonFinite);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(inf, 1.0, 1.0, 0.19), MrdvsPointStatus::kNonFinite);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(1.0, inf, 1.0, 0.19), MrdvsPointStatus::kNonFinite);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(1.0, 1.0, inf, 0.19), MrdvsPointStatus::kNonFinite);
}

TEST(MrdvsPreprocessUtils, RequiresAtLeastTwoPointsForLidarBuffer)
{
  EXPECT_FALSE(fast_livo::hasEnoughPointsForLidarBuffer(0));
  EXPECT_FALSE(fast_livo::hasEnoughPointsForLidarBuffer(1));
  EXPECT_TRUE(fast_livo::hasEnoughPointsForLidarBuffer(2));
}

TEST(MrdvsPreprocessUtils, ClearsPairedLidarQueuesAndResetsPushedState)
{
  std::deque<int> point_cloud_buffer{1, 2};
  std::deque<double> header_time_buffer{10.0, 11.0};
  bool lidar_pushed = true;

  fast_livo::clearPairedLidarBuffers(point_cloud_buffer, header_time_buffer, lidar_pushed);

  EXPECT_TRUE(point_cloud_buffer.empty());
  EXPECT_TRUE(header_time_buffer.empty());
  EXPECT_FALSE(lidar_pushed);
}

TEST(PreprocessConfiguration, KeepsBlindAndSquaredBlindSynchronized)
{
  Preprocess preprocess;
  EXPECT_DOUBLE_EQ(preprocess.blind_sqr, preprocess.blind * preprocess.blind);

  preprocess.setBlind(0.19);
  EXPECT_DOUBLE_EQ(preprocess.blind, 0.19);
  EXPECT_DOUBLE_EQ(preprocess.blind_sqr, preprocess.blind * preprocess.blind);

  preprocess.set(false, MRDVS, 0.25, 1);
  EXPECT_DOUBLE_EQ(preprocess.blind, 0.25);
  EXPECT_DOUBLE_EQ(preprocess.blind_sqr, preprocess.blind * preprocess.blind);
}

TEST(PreprocessConfiguration, ValidatesMaximumPointTimeOffset)
{
  Preprocess preprocess;
  EXPECT_DOUBLE_EQ(preprocess.max_point_time_offset_ms, 200.0);

  preprocess.setMaxPointTimeOffsetMs(150.0);
  EXPECT_DOUBLE_EQ(preprocess.max_point_time_offset_ms, 150.0);

  EXPECT_THROW(preprocess.setMaxPointTimeOffsetMs(0.0), std::invalid_argument);
  EXPECT_THROW(preprocess.setMaxPointTimeOffsetMs(-1.0), std::invalid_argument);
  EXPECT_THROW(
    preprocess.setMaxPointTimeOffsetMs(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    preprocess.setMaxPointTimeOffsetMs(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
  EXPECT_DOUBLE_EQ(preprocess.max_point_time_offset_ms, 150.0);
}

TEST(PreprocessMrdvsHandler, KeepsOnlySpatiallyAndTemporallyValidPointsSortedByTime)
{
  constexpr double cloud_start_sec = 1000000.0;
  constexpr double cloud_start_us = cloud_start_sec * 1.0e6;
  pcl::PointCloud<mrdvs_ros::Point> input;

  const auto add_point = [&input](float x, float y, float z, double timestamp, std::uint32_t intensity) {
    mrdvs_ros::Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    point.timestamp = timestamp;
    point.intensity = intensity;
    input.push_back(point);
  };

  add_point(0.0F, 0.0F, 0.0F, cloud_start_us, 1U);
  add_point(0.18F, 0.0F, 0.0F, cloud_start_us + 1000.0, 2U);
  add_point(0.195F, 0.0F, 0.0F, cloud_start_us + 50000.0, 3U);
  add_point(0.20F, 0.0F, 0.0F, cloud_start_us + 12345.0, 4U);
  add_point(0.21F, 0.0F, 0.0F, 0.0, 5U);
  add_point(0.22F, 0.0F, 0.0F, cloud_start_us + 250000.0, 6U);
  add_point(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, cloud_start_us + 2000.0, 7U);
  add_point(0.23F, 0.0F, 0.0F, cloud_start_us - 1.0, 8U);

  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(input, *message);
  message->header.stamp.sec = static_cast<std::int32_t>(cloud_start_sec);
  message->header.stamp.nanosec = 0U;

  Preprocess preprocess;
  preprocess.set(false, MRDVS, 0.19, 1);
  preprocess.setMaxPointTimeOffsetMs(200.0);
  PointCloudXYZI::Ptr output(new PointCloudXYZI());

  preprocess.process(message, output);

  ASSERT_EQ(output->size(), 2U);
  EXPECT_EQ(output->points[0].intensity, 4.0F);
  EXPECT_NEAR(output->points[0].curvature, 12.345, 1e-6);
  EXPECT_EQ(output->points[1].intensity, 3.0F);
  EXPECT_NEAR(output->points[1].curvature, 50.0, 1e-6);
}
