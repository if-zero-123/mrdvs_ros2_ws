#include "pgo_outputs.h"

#include <Eigen/Geometry>
#include <iomanip>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>

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

visualization_msgs::msg::MarkerArray makePoseMarkers(
    const nav_msgs::msg::Odometry &optimized_odom)
{
    visualization_msgs::msg::Marker origin_marker;
    origin_marker.header = optimized_odom.header;
    origin_marker.ns = "pgo_pose_labels";
    origin_marker.id = 0;
    origin_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    origin_marker.action = visualization_msgs::msg::Marker::ADD;
    origin_marker.pose.position.z = 0.25;
    origin_marker.pose.orientation.w = 1.0;
    origin_marker.scale.z = 0.25;
    origin_marker.color.r = 1.0F;
    origin_marker.color.g = 1.0F;
    origin_marker.color.b = 1.0F;
    origin_marker.color.a = 0.9F;
    origin_marker.text = "map origin\n(0.000, 0.000, 0.000) m";

    visualization_msgs::msg::Marker position_marker;
    position_marker.header = optimized_odom.header;
    position_marker.ns = "pgo_pose_labels";
    position_marker.id = 1;
    position_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    position_marker.action = visualization_msgs::msg::Marker::ADD;
    position_marker.pose.position = optimized_odom.pose.pose.position;
    position_marker.pose.position.z += 1.0;
    position_marker.pose.orientation.w = 1.0;
    position_marker.scale.z = 0.3;
    position_marker.color.r = 1.0F;
    position_marker.color.g = 0.85F;
    position_marker.color.b = 0.1F;
    position_marker.color.a = 1.0F;

    std::ostringstream text;
    text << std::fixed << std::setprecision(3)
         << "x: " << optimized_odom.pose.pose.position.x << " m\n"
         << "y: " << optimized_odom.pose.pose.position.y << " m\n"
         << "z: " << optimized_odom.pose.pose.position.z << " m";
    position_marker.text = text.str();

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(origin_marker);
    markers.markers.push_back(position_marker);
    return markers;
}
}  // namespace pgo_outputs
