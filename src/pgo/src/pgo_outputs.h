#pragma once

#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "pgos/simple_pgo.h"

namespace pgo_outputs
{
nav_msgs::msg::Odometry makeOptimizedOdometry(
    const nav_msgs::msg::Odometry &local_odom,
    const std::string &map_frame,
    const M3D &offset_r,
    const V3D &offset_t);

nav_msgs::msg::Path makeOptimizedPath(
    const std::vector<KeyPoseWithCloud> &key_poses,
    const std::string &map_frame,
    const builtin_interfaces::msg::Time &stamp);
}  // namespace pgo_outputs
