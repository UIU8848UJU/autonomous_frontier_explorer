#pragma once

#include <vector>

#include "robot_geometry_core/types/pose2d.hpp"

namespace navigation
{
namespace navigation_core
{

/// 不依赖 ROS 的二维路径表示。
struct Path2D
{
    std::vector<robot_geometry_core::Pose2D> poses;
};

/// 计算路径相邻位姿之间的二维长度。
double pathLengthM(const Path2D & path);

}  // namespace navigation_core
}  // namespace navigation
