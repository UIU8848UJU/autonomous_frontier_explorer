#include "navigation_core/path_safety_checker.hpp"

#include <algorithm>

namespace navigation
{
namespace navigation_core
{

void PathSafetyChecker::configure(const PathSafetyCheckerConfig & config)
{
    config_ = config;
}

bool PathSafetyChecker::isSafe(
    const Path2D & path,
    const grid_map_core::GridMap & map,
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
    if (!map.isReady()) {
        reason = "path_costmap_unavailable";
        return false;
    }

    for (const auto & pose : path.poses) {
        grid_map_core::GridCell cell;
        if (!map.worldToMap(pose.x, pose.y, cell)) {
            reason = "path_out_of_costmap";
            return false;
        }

        const auto value = map.value(
            static_cast<unsigned int>(cell.col),
            static_cast<unsigned int>(cell.row));
        max_path_cost = std::max(
            max_path_cost,
            static_cast<double>(std::max<std::int8_t>(0, value)));
        if (value < 0 && !config_.allow_unknown) {
            reason = "path_crosses_unknown";
            return false;
        }
        if (value >= config_.occupied_threshold) {
            reason = "path_crosses_high_cost";
            return false;
        }
    }

    reason = "path_safe";
    return true;
}

}  // namespace navigation_core
}  // namespace navigation
