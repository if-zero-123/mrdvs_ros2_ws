#ifndef IMU_INITIALIZATION_UTILS_H_
#define IMU_INITIALIZATION_UTILS_H_

#include <Eigen/Core>

#include <cmath>
#include <stdexcept>

namespace fast_livo
{

struct ImuInitializationConfig
{
  int required_samples;
  double gravity_magnitude;
  double max_gyro_norm;
  double max_acc_norm_error;
};

class ImuInitializationAccumulator
{
public:
  explicit ImuInitializationAccumulator(const ImuInitializationConfig &config)
  : config_(config)
  {
    if (config_.required_samples <= 0)
    {
      throw std::invalid_argument("IMU initialization requires a positive sample count");
    }
    if (!std::isfinite(config_.gravity_magnitude) || config_.gravity_magnitude <= 0.0)
    {
      throw std::invalid_argument("IMU initialization gravity magnitude must be finite and positive");
    }
    if (!std::isfinite(config_.max_gyro_norm) || config_.max_gyro_norm < 0.0)
    {
      throw std::invalid_argument("IMU initialization gyro threshold must be finite and non-negative");
    }
    if (!std::isfinite(config_.max_acc_norm_error) || config_.max_acc_norm_error < 0.0)
    {
      throw std::invalid_argument("IMU initialization acceleration threshold must be finite and non-negative");
    }
  }

  bool addSample(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
  {
    if (!acc.allFinite() || !gyro.allFinite())
    {
      reset();
      return false;
    }

    const double acc_norm = acc.norm();
    const double gyro_norm = gyro.norm();
    if (!std::isfinite(acc_norm) || !std::isfinite(gyro_norm) ||
      gyro_norm > config_.max_gyro_norm ||
      std::abs(acc_norm - config_.gravity_magnitude) > config_.max_acc_norm_error)
    {
      reset();
      return false;
    }

    if (ready()) return true;

    ++sample_count_;
    mean_acc_ += (acc - mean_acc_) / static_cast<double>(sample_count_);
    mean_gyro_ += (gyro - mean_gyro_) / static_cast<double>(sample_count_);
    return true;
  }

  void reset()
  {
    sample_count_ = 0;
    mean_acc_.setZero();
    mean_gyro_.setZero();
  }

  bool ready() const { return sample_count_ == config_.required_samples; }
  int sampleCount() const { return sample_count_; }
  const Eigen::Vector3d &meanAcc() const { return mean_acc_; }
  const Eigen::Vector3d &meanGyro() const { return mean_gyro_; }

private:
  ImuInitializationConfig config_;
  int sample_count_ = 0;
  Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyro_ = Eigen::Vector3d::Zero();
};

struct ImuInitializationEstimate
{
  Eigen::Vector3d gravity;
  Eigen::Vector3d gyro_bias;
  double mean_acc_norm;
};

inline ImuInitializationEstimate makeImuInitializationEstimate(
  const Eigen::Vector3d &mean_acc,
  const Eigen::Vector3d &mean_gyro,
  double gravity_magnitude)
{
  const double mean_acc_norm = mean_acc.norm();
  if (!mean_acc.allFinite() || !std::isfinite(mean_acc_norm) || mean_acc_norm <= 0.0)
  {
    throw std::invalid_argument("IMU initialization mean acceleration must be finite and non-zero");
  }
  return {
    -mean_acc / mean_acc_norm * gravity_magnitude,
    mean_gyro,
    mean_acc_norm};
}

inline void advanceImuInitializationWatermarks(
  double initialization_end_time,
  double &last_lio_update_time,
  double &last_prop_end_time)
{
  last_lio_update_time = initialization_end_time;
  last_prop_end_time = initialization_end_time;
}

} // namespace fast_livo

#endif // IMU_INITIALIZATION_UTILS_H_
