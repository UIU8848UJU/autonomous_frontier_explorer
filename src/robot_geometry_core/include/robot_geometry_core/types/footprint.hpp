#pragma once

#include <vector>

#include "robot_geometry_core/types/point2d.hpp"

namespace robot_geometry_core
{

/// 机器人在自身坐标系中的二维 footprint。
struct Footprint
{
    std::vector<Point2D> points;
};

}  // 命名空间 robot_geometry_core
