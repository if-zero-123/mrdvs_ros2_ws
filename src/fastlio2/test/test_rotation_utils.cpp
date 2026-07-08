#include <gtest/gtest.h>

#include "map_builder/commons.h"

TEST(RotationUtils, OrthonormalizesTruncatedCalibrationMatrix)
{
  M3D truncated;
  truncated << 0.999802, -0.007834, 0.018309,
      0.009201, 0.997081, -0.075789,
      -0.017662, 0.075943, 0.996956;

  const double original_error = (truncated.transpose() * truncated - M3D::Identity()).cwiseAbs().maxCoeff();
  EXPECT_GT(original_error, 1e-7);

  const M3D normalized = orthonormalizeRotationMatrix(truncated);

  EXPECT_NEAR(normalized.determinant(), 1.0, 1e-12);
  EXPECT_LT((normalized.transpose() * normalized - M3D::Identity()).cwiseAbs().maxCoeff(), 1e-12);
}
