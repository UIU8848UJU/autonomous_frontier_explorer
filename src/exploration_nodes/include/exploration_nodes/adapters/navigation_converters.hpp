#pragma once

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation_core/path_utils.hpp"
#include "robot_geometry_core/types/footprint.hpp"
#include "robot_geometry_core/types/pose2d.hpp"

namespace exploration
{
namespace adapters
{

/// 将 ROS 位姿转换为二维导航核心位姿。
robot_geometry_core::Pose2D toCorePose(const geometry_msgs::msg::Pose & pose);

/// 将 Nav2 路径转换为不依赖 ROS 的路径快照。
navigation::navigation_core::Path2D toCorePath(const nav_msgs::msg::Path & path);

/// 将 ROS footprint 点转换为纯几何 footprint。
robot_geometry_core::Footprint toCoreFootprint(
    const std::vector<geometry_msgs::msg::Point> & footprint);

/// 将 Nav2 cost 阈值转换为 OccupancyGrid 的占用率阈值。
std::int8_t occupancyThresholdFromNav2Cost(unsigned char cost);

}  // 命名空间 adapters
}  // 命名空间 exploration
