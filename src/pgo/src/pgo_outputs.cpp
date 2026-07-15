#include "pgo_outputs.h"

#include <Eigen/Geometry>

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
