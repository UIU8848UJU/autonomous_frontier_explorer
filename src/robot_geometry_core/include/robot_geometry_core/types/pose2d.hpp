#pragma once

namespace robot_geometry_core
{

/// 二维机器人位姿；导航核心只关心平面位置和偏航角。
struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

}  // 命名空间 robot_geometry_core
