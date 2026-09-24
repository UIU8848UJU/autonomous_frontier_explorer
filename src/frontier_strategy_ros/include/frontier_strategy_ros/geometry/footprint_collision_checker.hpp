#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "grid_map_ros/costmap_adapter.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "robot_geometry_core/footprint_collision_checker.hpp"

namespace frontier_strategy
{

/// ROS 适配层使用的 footprint 配置，负责承载 ROS 参数和消息类型。
struct FootprintCollisionCheckerConfig
{
    bool enabled{true};
    bool allow_unknown{false};
    unsigned char cost_threshold{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
    std::vector<geometry_msgs::msg::Point> footprint;
};

using FootprintCollisionCheckResult = robot_geometry_core::FootprintCollisionResult;

/// 将 ROS/Nav2 footprint 参数转换为纯几何核心调用。
class FootprintCollisionChecker
{
public:
    static std::vector<geometry_msgs::msg::Point> makeCircularFootprint(
        double robot_radius,
        double footprint_padding)
    {
        const auto core_footprint = robot_geometry_core::makeCircularFootprint(
            robot_radius, footprint_padding);
        return toRosFootprint(core_footprint);
    }

    /// 将纯 Core footprint 转为 ROS Point 数组。
    static std::vector<geometry_msgs::msg::Point> toRosFootprint(
        const robot_geometry_core::Footprint & core_footprint)
    {
        std::vector<geometry_msgs::msg::Point> footprint;
        footprint.reserve(core_footprint.points.size());
        for (const auto & point : core_footprint.points) {
            geometry_msgs::msg::Point ros_point;
            ros_point.x = point.x;
            ros_point.y = point.y;
            footprint.push_back(ros_point);
        }
        return footprint;
    }

    static FootprintCollisionCheckResult checkPose(
        const grid_map_ros::CostmapAdapter & costmap,
        const geometry_msgs::msg::Pose & pose,
        const FootprintCollisionCheckerConfig & config)
    {
        return checkWorldPoint(
            costmap,
            pose.position.x,
            pose.position.y,
            yawFromQuaternion(pose.orientation),
            config);
    }

    static FootprintCollisionCheckResult checkWorldPoint(
        const grid_map_ros::CostmapAdapter & costmap,
        double x,
        double y,
        double yaw,
        const FootprintCollisionCheckerConfig & config)
    {
        robot_geometry_core::FootprintCollisionConfig core_config;
        core_config.enabled = config.enabled;
        core_config.allow_unknown = config.allow_unknown;
        core_config.occupied_threshold = occupancyThreshold(config.cost_threshold);
        core_config.footprint.points.reserve(config.footprint.size());
        for (const auto & point : config.footprint) {
            core_config.footprint.points.push_back({point.x, point.y});
        }
        return robot_geometry_core::checkFootprint(
            costmap.gridMap(),
            robot_geometry_core::Pose2D{x, y, yaw},
            core_config);
    }

private:
    static std::int8_t occupancyThreshold(unsigned char nav2_cost_threshold)
    {
        // CostmapAdapter 将 51~100 的占用概率映射为致命 cost，因此反向转换时使用相同边界。
        if (nav2_cost_threshold >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            return 51;
        }
        const auto threshold = static_cast<int>(std::ceil(
            static_cast<double>(nav2_cost_threshold) * 50.0 / 252.0));
        return static_cast<std::int8_t>(std::clamp(threshold, 1, 100));
    }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
    {
        const double sin_yaw = 2.0 * (quaternion.w * quaternion.z +
            quaternion.x * quaternion.y);
        const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y +
            quaternion.z * quaternion.z);
        return std::atan2(sin_yaw, cos_yaw);
    }
};

}  // namespace frontier_strategy
