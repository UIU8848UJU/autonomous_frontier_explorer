#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "frontier_explorer_core/costmap/costmap_adapter.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace frontier_explorer
{

struct FootprintCollisionCheckerConfig
{
    bool enabled{true};
    bool allow_unknown{false};
    unsigned char cost_threshold{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
    std::vector<geometry_msgs::msg::Point> footprint;
};

struct FootprintCollisionCheckResult
{
    bool valid{false};
    std::string reason;
    double max_cost{0.0};
};

/// @brief: 共享 robot footprint 落脚检查，供 frontier 硬过滤和 NavigationNode 执行检查复用
class FootprintCollisionChecker
{
public:
    static std::vector<geometry_msgs::msg::Point> makeCircularFootprint(
        double robot_radius,
        double footprint_padding)
    {
        auto footprint = nav2_costmap_2d::makeFootprintFromRadius(std::max(0.01, robot_radius));
        if (footprint_padding > 0.0) {
            nav2_costmap_2d::padFootprint(footprint, footprint_padding);
        }
        return footprint;
    }

    static FootprintCollisionCheckResult checkPose(
        const CostmapAdapter & costmap,
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
        const CostmapAdapter & costmap,
        double x,
        double y,
        double yaw,
        const FootprintCollisionCheckerConfig & config)
    {
        FootprintCollisionCheckResult result;
        if (!config.enabled) {
            result.valid = true;
            result.reason = "footprint_check_disabled";
            return result;
        }
        if (!costmap.isReady()) {
            result.reason = "footprint_costmap_unavailable";
            return result;
        }
        if (config.footprint.empty()) {
            result.reason = "footprint_empty";
            return result;
        }

        const double cos_th = std::cos(yaw);
        const double sin_th = std::sin(yaw);
        std::vector<geometry_msgs::msg::Point> oriented;
        oriented.reserve(config.footprint.size());
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        for (const auto & point : config.footprint) {
            geometry_msgs::msg::Point transformed;
            transformed.x = x + point.x * cos_th - point.y * sin_th;
            transformed.y = y + point.x * sin_th + point.y * cos_th;
            oriented.push_back(transformed);
            min_x = std::min(min_x, transformed.x);
            min_y = std::min(min_y, transformed.y);
            max_x = std::max(max_x, transformed.x);
            max_y = std::max(max_y, transformed.y);
        }

        unsigned int min_mx = 0U;
        unsigned int min_my = 0U;
        unsigned int max_mx = 0U;
        unsigned int max_my = 0U;
        if (!costmap.worldToMap(min_x, min_y, min_mx, min_my) ||
            !costmap.worldToMap(max_x, max_y, max_mx, max_my))
        {
            result.reason = "footprint_out_of_costmap";
            return result;
        }

        const auto start_x = std::min(min_mx, max_mx);
        const auto end_x = std::max(min_mx, max_mx);
        const auto start_y = std::min(min_my, max_my);
        const auto end_y = std::max(min_my, max_my);
        bool sampled = false;
        for (unsigned int my = start_y; my <= end_y; ++my) {
            for (unsigned int mx = start_x; mx <= end_x; ++mx) {
                double wx = 0.0;
                double wy = 0.0;
                costmap.mapToWorld(mx, my, wx, wy);
                if (!pointInPolygon(wx, wy, oriented)) {
                    continue;
                }
                sampled = true;
                const auto cost = costmap.getCost(mx, my);
                result.max_cost = std::max(result.max_cost, static_cast<double>(cost));
                if (cost == nav2_costmap_2d::NO_INFORMATION && !config.allow_unknown) {
                    result.reason = "footprint_over_unknown";
                    return result;
                }
                if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= config.cost_threshold) {
                    result.reason = "footprint_collision";
                    return result;
                }
            }
        }

        if (!sampled) {
            result.reason = "footprint_not_sampled";
            return result;
        }

        result.valid = true;
        result.reason = "footprint_valid";
        return result;
    }

private:
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
    {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static bool pointInPolygon(
        double x,
        double y,
        const std::vector<geometry_msgs::msg::Point> & polygon)
    {
        if (polygon.size() < 3U) {
            return false;
        }

        bool inside = false;
        for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
            const auto & pi = polygon[i];
            const auto & pj = polygon[j];
            const bool intersects =
                ((pi.y > y) != (pj.y > y)) &&
                (x < (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1e-12) + pi.x);
            if (intersects) {
                inside = !inside;
            }
        }
        return inside;
    }
};

}  // namespace frontier_explorer
