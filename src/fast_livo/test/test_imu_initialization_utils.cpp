#include "imu_initialization_utils.h"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <stdexcept>

namespace
{

constexpr double kGravity = 9.81;
constexpr double kMaxGyroNorm = 0.10;
constexpr double kMaxAccNormError = 0.75;

fast_livo::ImuInitializationConfig defaultConfig(int required_samples = 3)
{
  return {required_samples, kGravity, kMaxGyroNorm, kMaxAccNormError};
}

const Eigen::Vector3d kStaticAcc(0.0, 0.0, 9.70);
const Eigen::Vector3d kStaticGyro(0.01, -0.02, 0.005);

} // namespace

TEST(ImuInitializationAccumulator, RequiresConsecutiveStationarySamplesAndComputesMeans)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig());

  EXPECT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.01, -0.02, 0.005)));
  EXPECT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.02, -0.01, 0.005)));
  EXPECT_FALSE(accumulator.ready());
  EXPECT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.00, -0.03, 0.005)));

  EXPECT_TRUE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 3);
  EXPECT_TRUE(accumulator.meanAcc().isApprox(kStaticAcc, 1e-12));
  EXPECT_TRUE(accumulator.meanGyro().isApprox(Eigen::Vector3d(0.01, -0.02, 0.005), 1e-12));
}

TEST(ImuInitializationAccumulator, GyroscopeMotionResetsTheWholeWindow)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig());

  ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  EXPECT_FALSE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.20, 0.0, 0.0)));

  EXPECT_FALSE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 0);
  EXPECT_TRUE(accumulator.meanAcc().isZero(0.0));
  EXPECT_TRUE(accumulator.meanGyro().isZero(0.0));
}

TEST(ImuInitializationAccumulator, IncludesGyroscopeAndAccelerationThresholdBoundaries)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig(4));

  EXPECT_TRUE(accumulator.addSample(
    Eigen::Vector3d(0.0, 0.0, kGravity + kMaxAccNormError),
    Eigen::Vector3d(kMaxGyroNorm, 0.0, 0.0)));
  EXPECT_EQ(accumulator.sampleCount(), 1);

  EXPECT_FALSE(accumulator.addSample(
    kStaticAcc,
    Eigen::Vector3d(kMaxGyroNorm + 1e-6, 0.0, 0.0)));
  EXPECT_EQ(accumulator.sampleCount(), 0);

  ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  EXPECT_FALSE(accumulator.addSample(
    Eigen::Vector3d(0.0, 0.0, kGravity + kMaxAccNormError + 1e-6),
    Eigen::Vector3d::Zero()));
  EXPECT_EQ(accumulator.sampleCount(), 0);
}

TEST(ImuInitializationAccumulator, RejectsEveryNonFiniteSensorComponentAndResets)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig());
  const std::array<double, 2> invalid_values{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()};

  for (const double invalid_value : invalid_values)
  {
    for (Eigen::Index component = 0; component < 3; ++component)
    {
      Eigen::Vector3d invalid_acc = kStaticAcc;
      invalid_acc[component] = invalid_value;
      ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
      EXPECT_FALSE(accumulator.addSample(invalid_acc, kStaticGyro));
      EXPECT_EQ(accumulator.sampleCount(), 0);

      Eigen::Vector3d invalid_gyro = kStaticGyro;
      invalid_gyro[component] = invalid_value;
      ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
      EXPECT_FALSE(accumulator.addSample(kStaticAcc, invalid_gyro));
      EXPECT_EQ(accumulator.sampleCount(), 0);
    }
  }
}

TEST(ImuInitializationAccumulator, BecomesReadyAtExactlySixHundredSamplesAcrossBatches)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig(600));

  for (int sample = 0; sample < 300; ++sample)
  {
    ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  }
  EXPECT_FALSE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 300);

  for (int sample = 300; sample < 599; ++sample)
  {
    ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  }
  EXPECT_FALSE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 599);

  EXPECT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  EXPECT_TRUE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 600);
}

TEST(ImuInitializationAccumulator, MotionAfterReadyInTheSameBatchCancelsReadiness)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig());
  for (int sample = 0; sample < 3; ++sample)
  {
    ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  }
  ASSERT_TRUE(accumulator.ready());

  EXPECT_FALSE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.20, 0.0, 0.0)));
  EXPECT_FALSE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 0);
}

TEST(ImuInitializationAccumulator, InvalidSampleAfterReadyInTheSameBatchCancelsReadiness)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig());
  for (int sample = 0; sample < 3; ++sample)
  {
    ASSERT_TRUE(accumulator.addSample(kStaticAcc, kStaticGyro));
  }
  ASSERT_TRUE(accumulator.ready());

  Eigen::Vector3d invalid_acc = kStaticAcc;
  invalid_acc.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(accumulator.addSample(invalid_acc, kStaticGyro));
  EXPECT_FALSE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 0);
}

TEST(ImuInitializationAccumulator, FreezesCompletedWindowForAdditionalStationarySamples)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig());
  ASSERT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.01, 0.00, 0.00)));
  ASSERT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.02, 0.00, 0.00)));
  ASSERT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.03, 0.00, 0.00)));
  ASSERT_TRUE(accumulator.ready());
  const Eigen::Vector3d completed_mean_acc = accumulator.meanAcc();
  const Eigen::Vector3d completed_mean_gyro = accumulator.meanGyro();

  EXPECT_TRUE(accumulator.addSample(
    Eigen::Vector3d(0.0, 0.0, 9.90),
    Eigen::Vector3d(0.09, 0.0, 0.0)));

  EXPECT_TRUE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 3);
  EXPECT_TRUE(accumulator.meanAcc().isApprox(completed_mean_acc, 0.0));
  EXPECT_TRUE(accumulator.meanGyro().isApprox(completed_mean_gyro, 0.0));
}

TEST(ImuInitializationAccumulator, ResetClearsMeansAndKeepsWindowsIndependent)
{
  fast_livo::ImuInitializationAccumulator accumulator(defaultConfig(2));
  ASSERT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.01, 0.0, 0.0)));
  ASSERT_TRUE(accumulator.addSample(kStaticAcc, Eigen::Vector3d(0.03, 0.0, 0.0)));
  ASSERT_TRUE(accumulator.ready());

  accumulator.reset();

  EXPECT_FALSE(accumulator.ready());
  EXPECT_EQ(accumulator.sampleCount(), 0);
  EXPECT_TRUE(accumulator.meanAcc().isZero(0.0));
  EXPECT_TRUE(accumulator.meanGyro().isZero(0.0));

  const Eigen::Vector3d second_acc(0.0, 0.0, 9.60);
  const Eigen::Vector3d second_gyro(-0.02, 0.01, 0.0);
  ASSERT_TRUE(accumulator.addSample(second_acc, second_gyro));
  ASSERT_TRUE(accumulator.addSample(second_acc, second_gyro));
  EXPECT_TRUE(accumulator.meanAcc().isApprox(second_acc, 1e-12));
  EXPECT_TRUE(accumulator.meanGyro().isApprox(second_gyro, 1e-12));
}

TEST(ImuInitializationAccumulator, RejectsInvalidConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_THROW(fast_livo::ImuInitializationAccumulator({0, kGravity, 0.10, 0.75}), std::invalid_argument);
  EXPECT_THROW(fast_livo::ImuInitializationAccumulator({-1, kGravity, 0.10, 0.75}), std::invalid_argument);

  for (const double invalid_gravity : {0.0, -kGravity, nan, inf})
  {
    EXPECT_THROW(
      fast_livo::ImuInitializationAccumulator({3, invalid_gravity, 0.10, 0.75}),
      std::invalid_argument);
  }
  for (const double invalid_threshold : {-0.01, nan, inf})
  {
    EXPECT_THROW(
      fast_livo::ImuInitializationAccumulator({3, kGravity, invalid_threshold, 0.75}),
      std::invalid_argument);
    EXPECT_THROW(
      fast_livo::ImuInitializationAccumulator({3, kGravity, 0.10, invalid_threshold}),
      std::invalid_argument);
  }
}

TEST(ImuInitializationEstimate, SetsGravityGyroscopeBiasAndMeanAccelerationNorm)
{
  const Eigen::Vector3d mean_acc(0.0, 0.0, 9.70);
  const Eigen::Vector3d mean_gyro(0.01, -0.02, 0.005);

  const auto estimate =
    fast_livo::makeImuInitializationEstimate(mean_acc, mean_gyro, kGravity);

  EXPECT_TRUE(estimate.gravity.isApprox(Eigen::Vector3d(0.0, 0.0, -kGravity), 1e-12));
  EXPECT_TRUE(estimate.gyro_bias.isApprox(mean_gyro, 0.0));
  EXPECT_NEAR(estimate.mean_acc_norm, 9.70, 1e-12);
}

TEST(ImuInitializationEstimate, RejectsZeroAndNonFiniteMeanAcceleration)
{
  const Eigen::Vector3d mean_gyro(0.01, -0.02, 0.005);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_THROW(
    fast_livo::makeImuInitializationEstimate(Eigen::Vector3d::Zero(), mean_gyro, kGravity),
    std::invalid_argument);
  EXPECT_THROW(
    fast_livo::makeImuInitializationEstimate(Eigen::Vector3d(nan, 0.0, kGravity), mean_gyro, kGravity),
    std::invalid_argument);
  EXPECT_THROW(
    fast_livo::makeImuInitializationEstimate(Eigen::Vector3d(0.0, inf, kGravity), mean_gyro, kGravity),
    std::invalid_argument);
}

TEST(ImuInitializationWatermarks, AdvancesLioAndPropagationTimesTogether)
{
  double last_lio_update_time = 10.0;
  double last_prop_end_time = 11.0;

  fast_livo::advanceImuInitializationWatermarks(
    12.5, last_lio_update_time, last_prop_end_time);

  EXPECT_DOUBLE_EQ(last_lio_update_time, 12.5);
  EXPECT_DOUBLE_EQ(last_prop_end_time, 12.5);
}
