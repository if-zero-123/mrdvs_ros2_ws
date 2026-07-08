#include <gtest/gtest.h>

#include "map_builder/lidar_processor.h"

TEST(LidarProcessorMath, ImuRotationJacobianUsesLidarToImuTranslation)
{
  State state;
  state.r_wi = Eigen::AngleAxisd(0.2, V3D::UnitZ()).toRotationMatrix();
  state.t_wi = V3D(10.0, 20.0, 30.0);
  state.r_il = Eigen::AngleAxisd(0.1, V3D::UnitY()).toRotationMatrix();
  state.t_il = V3D(0.1, -0.2, 0.3);

  const V3D lidar_point(1.0, 2.0, 3.0);
  const V3D norm_vec(0.3, -0.5, 0.8);

  const Eigen::Matrix<double, 1, 3> expected =
      -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * lidar_point + state.t_il);
  const Eigen::Matrix<double, 1, 3> old_formula =
      -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * lidar_point + state.t_wi);

  const Eigen::Matrix<double, 1, 3> actual =
      LidarProcessor::computeImuRotationJacobian(norm_vec, state, lidar_point);

  EXPECT_TRUE(actual.isApprox(expected, 1e-12));
  EXPECT_FALSE(actual.isApprox(old_formula, 1e-6));
}
