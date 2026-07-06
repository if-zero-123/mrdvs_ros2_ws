#include "tools/imu_extrinsic_format.h"

#include <gtest/gtest.h>

#include <array>
#include <string>

TEST(ImuExtrinsicFormat, FormatsRawMatrixTranslationAndYaml)
{
  const std::array<float, 12> values = {
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, -1.0f,
    0.0f, 1.0f, 0.0f,
    12.5f, -3.0f, 0.25f};

  const std::string text = lx_camera_ros::tools::FormatImuExtrinsic(values);

  EXPECT_NE(text.find("raw: [1, 0, 0, 0, 0, -1, 0, 1, 0, 12.5, -3, 0.25]"),
            std::string::npos);
  EXPECT_NE(text.find("rotation matrix:"), std::string::npos);
  EXPECT_NE(text.find("[0, 0, -1]"), std::string::npos);
  EXPECT_NE(text.find("translation: [12.5, -3, 0.25]"), std::string::npos);
  EXPECT_NE(text.find("r_il: [1, 0, 0, 0, 0, -1, 0, 1, 0]"),
            std::string::npos);
  EXPECT_NE(text.find("t_il: [12.5, -3, 0.25]"), std::string::npos);
}

TEST(ImuExtrinsicFormat, SelectsOpenModeByTargetShape)
{
  EXPECT_EQ(lx_camera_ros::tools::SelectOpenMode("0"), OPEN_BY_INDEX);
  EXPECT_EQ(lx_camera_ros::tools::SelectOpenMode("192.168.1.10"), OPEN_BY_IP);
}

TEST(ImuExtrinsicFormat, DetectsAllZeroExtrinsic)
{
  lx_camera_ros::tools::ImuExtrinsicValues zeros{};
  EXPECT_TRUE(lx_camera_ros::tools::IsAllZeroExtrinsic(zeros));

  lx_camera_ros::tools::ImuExtrinsicValues identity = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f};
  EXPECT_FALSE(lx_camera_ros::tools::IsAllZeroExtrinsic(identity));
}
