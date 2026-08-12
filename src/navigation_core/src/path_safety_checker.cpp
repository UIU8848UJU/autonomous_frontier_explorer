#include "navigation_core/path_safety_checker.hpp"

#include <algorithm>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_explorer
{
namespace navigation_core
{

PathSafetyChecker::PathSafetyChecker(const CostmapAdapter & costmap)
: costmap_(costmap)
{
}

void PathSafetyChecker::configure(const PathSafetyCheckerConfig & config)
{
    config_ = config;
}

bool PathSafetyChecker::isSafe(
    const nav_msgs::msg::Path & path,
    std::string & reason,
    double & max_path_cost) const
{
    max_path_cost = 0.0;
    if (!config_.enabled) {
        reason = "path_check_disabled";
        return true;
    }
    if (path.poses.empty()) {
        reason = "path_empty";
        return false;
    }
    if (!costmap_.isReady()) {
        reason = "path_costmap_unavailable";
        return false;
    }

    for (const auto & pose : path.poses) {
        unsigned int mx = 0U;
        unsigned int my = 0U;
        if (!costmap_.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
            reason = "path_out_of_costmap";
            return false;
        }

        const auto cost = costmap_.getCost(mx, my);
        max_path_cost = std::max(max_path_cost, static_cast<double>(cost));
        if (cost == nav2_costmap_2d::NO_INFORMATION && !config_.allow_unknown) {
            reason = "path_crosses_unknown";
            return false;
        }
        if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= config_.cost_threshold) {
            reason = "path_crosses_high_cost";
            return false;
        }
    }

    reason = "path_safe";
    return true;
}

}  // namespace navigation_core
}  // namespace frontier_explorer
