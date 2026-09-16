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
    // Nav2 的 Costmap2D 发布为 OccupancyGrid 时使用 0~100 表示代价：
    // 0 表示空闲，99 表示 inscribed/inflated obstacle，100 表示致命障碍。
    // 这里必须和 grid_map_ros::CostmapAdapter 的转换保持同一量纲，不能再
    // 把阈值压缩到 0~50，否则 path_cost_threshold=253 会错误地变成 51。
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        return 100;
    }
    const auto threshold = static_cast<int>(std::ceil(
        static_cast<double>(cost) * 99.0 /
        static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)));
    return static_cast<std::int8_t>(std::clamp(threshold, 1, 99));
}

}  // 命名空间 adapters
}  // 命名空间 exploration
