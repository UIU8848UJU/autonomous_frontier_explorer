#include "exploration_nodes/adapters/navigation_converters.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace exploration
{
namespace adapters
{

robot_geometry_core::Pose2D toCorePose(const geometry_msgs::msg::Pose & pose)
{
    const double sin_yaw = 2.0 * (pose.orientation.w * pose.orientation.z +
        pose.orientation.x * pose.orientation.y);
    const double cos_yaw = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z);
    return robot_geometry_core::Pose2D{
        pose.position.x,
        pose.position.y,
        std::atan2(sin_yaw, cos_yaw)};
}

navigation::navigation_core::Path2D toCorePath(const nav_msgs::msg::Path & path)
{
    navigation::navigation_core::Path2D result;
    result.poses.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
        result.poses.push_back(toCorePose(pose.pose));
    }
    return result;
}

robot_geometry_core::Footprint toCoreFootprint(
    const std::vector<geometry_msgs::msg::Point> & footprint)
{
    robot_geometry_core::Footprint result;
    result.points.reserve(footprint.size());
    for (const auto & point : footprint) {
        result.points.push_back({point.x, point.y});
    }
    return result;
}

std::int8_t occupancyThresholdFromNav2Cost(unsigned char cost)
{
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        return 51;
    }
    const auto threshold = static_cast<int>(std::ceil(
        static_cast<double>(cost) * 50.0 / 252.0));
    return static_cast<std::int8_t>(std::clamp(threshold, 1, 100));
}

}  // 命名空间 adapters
}  // 命名空间 exploration
