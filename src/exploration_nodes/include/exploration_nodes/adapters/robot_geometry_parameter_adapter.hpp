#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_geometry_core/robot_geometry_provider.hpp"

namespace exploration
{
namespace adapters
{

/// ROS 参数规范化后的机器人二维几何配置。
struct RobotGeometryParameters
{
    std::string shape{"circle"};
    double radius{0.1};
    double padding{0.0};
    std::vector<double> footprint_xy;
    std::string frame_id{"base_link"};
    std::string source{"configured"};
    std::uint64_t revision{0U};
};

/// 旧参数名只负责兼容输入；所有几何解释仍集中在本适配器内。
struct LegacyRobotGeometryParameterNames
{
    std::string radius;
    std::string padding;
};

/// 声明统一几何参数以及当前节点的旧半径参数别名。
void declareRobotGeometryParameters(
    rclcpp::Node & node,
    const LegacyRobotGeometryParameterNames & legacy_names,
    const RobotGeometryParameters & defaults = {});

/// 从 ROS 参数读取并形成与 ROS 无关的中间配置。
RobotGeometryParameters loadRobotGeometryParameters(
    const rclcpp::Node & node,
    const LegacyRobotGeometryParameterNames & legacy_names);

/// 根据规范化配置创建静态 Provider；非法配置会抛出异常并阻止节点带病启动。
std::shared_ptr<const robot_geometry_core::IRobotGeometryProvider>
makeRobotGeometryProvider(const RobotGeometryParameters & params);

}  // 命名空间 adapters
}  // 命名空间 exploration
