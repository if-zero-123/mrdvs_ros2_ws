#include "pgo_outputs.h"

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace pgo_outputs
{
namespace
{
geometry_msgs::msg::Pose makePose(const M3D &rotation, const V3D &translation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();

    const Eigen::Quaterniond quaternion(rotation);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
}
}  // namespace

OptimizedMapAssembler::OptimizedMapAssembler()
    : map_(new CloudType)
{
}

CloudType::ConstPtr OptimizedMapAssembler::update(
    const std::vector<KeyPoseWithCloud> &key_poses,
    bool rebuild,
    double resolution)
{
    if (rebuild || key_poses.size() < assembled_key_pose_count_)
    {
        map_->clear();
        assembled_key_pose_count_ = 0;
    }

    for (std::size_t i = assembled_key_pose_count_; i < key_poses.size(); ++i)
    {
        const KeyPoseWithCloud &key_pose = key_poses[i];
        if (!key_pose.body_cloud || key_pose.body_cloud->empty())
            continue;

        CloudType transformed_cloud;
        pcl::transformPointCloud(
            *key_pose.body_cloud,
            transformed_cloud,
            key_pose.t_global,
            Eigen::Quaterniond(key_pose.r_global));
        *map_ += transformed_cloud;
    }
    assembled_key_pose_count_ = key_poses.size();

    if (resolution > 0.0 && !map_->empty())
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(map_);
        CloudType filtered_map;
        voxel_grid.filter(filtered_map);
        map_->swap(filtered_map);
    }
    return map_;
}

nav_msgs::msg::Odometry makeOptimizedOdometry(
    const nav_msgs::msg::Odometry &local_odom,
    const std::string &map_frame,
    const M3D &offset_r,
    const V3D &offset_t)
{
    const Eigen::Quaterniond local_quaternion(
        local_odom.pose.pose.orientation.w,
        local_odom.pose.pose.orientation.x,
        local_odom.pose.pose.orientation.y,
        local_odom.pose.pose.orientation.z);
    const M3D local_r = local_quaternion.normalized().toRotationMatrix();
    const V3D local_t(
        local_odom.pose.pose.position.x,
        local_odom.pose.pose.position.y,
        local_odom.pose.pose.position.z);

    nav_msgs::msg::Odometry optimized_odom = local_odom;
    optimized_odom.header.frame_id = map_frame;
    optimized_odom.pose.pose = makePose(
        offset_r * local_r,
        offset_r * local_t + offset_t);
    return optimized_odom;
}

nav_msgs::msg::Path makeOptimizedPath(
    const std::vector<KeyPoseWithCloud> &key_poses,
    const std::string &map_frame,
    const builtin_interfaces::msg::Time &stamp)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame;
    path.header.stamp = stamp;
    path.poses.reserve(key_poses.size());

    for (const KeyPoseWithCloud &key_pose : key_poses)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose = makePose(key_pose.r_global, key_pose.t_global);
        path.poses.push_back(pose);
    }
    return path;
}
}  // namespace pgo_outputs
