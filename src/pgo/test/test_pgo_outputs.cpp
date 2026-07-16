#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "pgo_outputs.h"

namespace
{
constexpr double kTolerance = 1e-9;

TEST(PgoOutputs, CorrectsRealtimeOdometryIntoMapFrame)
{
    nav_msgs::msg::Odometry local_odom;
    local_odom.header.frame_id = "lio_local";
    local_odom.header.stamp.sec = 12;
    local_odom.header.stamp.nanosec = 345;
    local_odom.child_frame_id = "mrdvs_imu";
    local_odom.pose.pose.position.x = 1.0;
    local_odom.pose.pose.position.y = 0.0;
    local_odom.pose.pose.position.z = 2.0;
    local_odom.pose.pose.orientation.w = 1.0;
    local_odom.twist.twist.linear.x = 0.4;

    const M3D offset_r =
        Eigen::AngleAxisd(M_PI_2, V3D::UnitZ()).toRotationMatrix();
    const V3D offset_t(10.0, 0.0, 0.0);

    const auto optimized = pgo_outputs::makeOptimizedOdometry(
        local_odom, "map", offset_r, offset_t);

    EXPECT_EQ(optimized.header.frame_id, "map");
    EXPECT_EQ(optimized.header.stamp, local_odom.header.stamp);
    EXPECT_EQ(optimized.child_frame_id, "mrdvs_imu");
    EXPECT_NEAR(optimized.pose.pose.position.x, 10.0, kTolerance);
    EXPECT_NEAR(optimized.pose.pose.position.y, 1.0, kTolerance);
    EXPECT_NEAR(optimized.pose.pose.position.z, 2.0, kTolerance);
    EXPECT_NEAR(optimized.pose.pose.orientation.z, std::sqrt(0.5), kTolerance);
    EXPECT_NEAR(optimized.pose.pose.orientation.w, std::sqrt(0.5), kTolerance);
    EXPECT_DOUBLE_EQ(optimized.twist.twist.linear.x, 0.4);
}

TEST(PgoOutputs, BuildsPathFromEveryOptimizedKeyPose)
{
    std::vector<KeyPoseWithCloud> key_poses(2);
    key_poses[0].r_global.setIdentity();
    key_poses[0].t_global = V3D(1.0, 2.0, 3.0);
    key_poses[1].r_global =
        Eigen::AngleAxisd(M_PI, V3D::UnitX()).toRotationMatrix();
    key_poses[1].t_global = V3D(4.0, 5.0, 6.0);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 20;
    stamp.nanosec = 30;
    const auto path = pgo_outputs::makeOptimizedPath(
        key_poses, "map", stamp);

    ASSERT_EQ(path.poses.size(), 2U);
    EXPECT_EQ(path.header.frame_id, "map");
    EXPECT_EQ(path.header.stamp, stamp);
    EXPECT_EQ(path.poses[0].header.frame_id, "map");
    EXPECT_EQ(path.poses[0].header.stamp, stamp);
    EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(path.poses[1].pose.position.y, 5.0);
    EXPECT_NEAR(std::abs(path.poses[1].pose.orientation.x), 1.0, kTolerance);
    EXPECT_NEAR(path.poses[1].pose.orientation.w, 0.0, kTolerance);
}

KeyPoseWithCloud makeKeyPoseWithOnePoint(double pose_x, float point_x)
{
    KeyPoseWithCloud key_pose;
    key_pose.r_global.setIdentity();
    key_pose.t_global = V3D(pose_x, 0.0, 0.0);
    key_pose.body_cloud = CloudType::Ptr(new CloudType);
    PointType point;
    point.x = point_x;
    point.y = 0.0F;
    point.z = 0.0F;
    point.intensity = 1.0F;
    key_pose.body_cloud->push_back(point);
    return key_pose;
}

TEST(PgoOutputs, IncrementalMapOnlyAppendsNewKeyPoses)
{
    pgo_outputs::OptimizedMapAssembler assembler;
    std::vector<KeyPoseWithCloud> key_poses;
    key_poses.push_back(makeKeyPoseWithOnePoint(1.0, 0.0F));

    auto map = assembler.update(key_poses, false, 0.0);
    ASSERT_EQ(map->size(), 1U);
    EXPECT_NEAR(map->points[0].x, 1.0, kTolerance);

    key_poses.push_back(makeKeyPoseWithOnePoint(2.0, 0.0F));
    map = assembler.update(key_poses, false, 0.0);
    ASSERT_EQ(map->size(), 2U);
    EXPECT_NEAR(map->points[0].x, 1.0, kTolerance);
    EXPECT_NEAR(map->points[1].x, 2.0, kTolerance);
}

TEST(PgoOutputs, LoopCorrectionRebuildsEveryHistoricalKeyPose)
{
    pgo_outputs::OptimizedMapAssembler assembler;
    std::vector<KeyPoseWithCloud> key_poses;
    key_poses.push_back(makeKeyPoseWithOnePoint(1.0, 0.0F));
    key_poses.push_back(makeKeyPoseWithOnePoint(2.0, 0.0F));
    assembler.update(key_poses, false, 0.0);

    key_poses[0].t_global.x() = 10.0;
    key_poses[1].t_global.x() = 20.0;
    const auto rebuilt_map = assembler.update(key_poses, true, 0.0);

    ASSERT_EQ(rebuilt_map->size(), 2U);
    EXPECT_NEAR(rebuilt_map->points[0].x, 10.0, kTolerance);
    EXPECT_NEAR(rebuilt_map->points[1].x, 20.0, kTolerance);
}

TEST(PgoOutputs, MapResolutionDownsamplesPointsInTheSameVoxel)
{
    pgo_outputs::OptimizedMapAssembler assembler;
    std::vector<KeyPoseWithCloud> key_poses;
    key_poses.push_back(makeKeyPoseWithOnePoint(0.0, 0.05F));
    PointType nearby_point;
    nearby_point.x = 0.1F;
    nearby_point.y = 0.0F;
    nearby_point.z = 0.0F;
    nearby_point.intensity = 1.0F;
    key_poses[0].body_cloud->push_back(nearby_point);

    const auto filtered_map = assembler.update(key_poses, false, 0.5);

    EXPECT_EQ(filtered_map->size(), 1U);
}

TEST(PgoOutputs, BuildsMapOriginAndRealtimePositionLabels)
{
    nav_msgs::msg::Odometry optimized_odom;
    optimized_odom.header.frame_id = "map";
    optimized_odom.header.stamp.sec = 42;
    optimized_odom.child_frame_id = "mrdvs_imu";
    optimized_odom.pose.pose.position.x = 1.23456;
    optimized_odom.pose.pose.position.y = -2.34567;
    optimized_odom.pose.pose.position.z = 0.45644;
    optimized_odom.pose.pose.orientation.w = 1.0;

    const auto markers = pgo_outputs::makePoseMarkers(optimized_odom);

    ASSERT_EQ(markers.markers.size(), 2U);
    const auto &origin_marker = markers.markers[0];
    EXPECT_EQ(origin_marker.header, optimized_odom.header);
    EXPECT_EQ(origin_marker.ns, "pgo_pose_labels");
    EXPECT_EQ(origin_marker.id, 0);
    EXPECT_EQ(origin_marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    EXPECT_EQ(origin_marker.text, "map origin\n(0.000, 0.000, 0.000) m");
    EXPECT_DOUBLE_EQ(origin_marker.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(origin_marker.pose.position.y, 0.0);

    const auto &position_marker = markers.markers[1];
    EXPECT_EQ(position_marker.header, optimized_odom.header);
    EXPECT_EQ(position_marker.ns, "pgo_pose_labels");
    EXPECT_EQ(position_marker.id, 1);
    EXPECT_EQ(position_marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    EXPECT_EQ(position_marker.text, "x: 1.235 m\ny: -2.346 m\nz: 0.456 m");
    EXPECT_DOUBLE_EQ(position_marker.pose.position.x, 1.23456);
    EXPECT_DOUBLE_EQ(position_marker.pose.position.y, -2.34567);
    EXPECT_DOUBLE_EQ(position_marker.pose.position.z, 0.95644);
}
}  // namespace
