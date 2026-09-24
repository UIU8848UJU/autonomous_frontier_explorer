#include "frontier_strategy_ros/adaptation/frontier_map_parameter_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_strategy
{
namespace
{
constexpr double kResolutionEpsilon = 1e-9;

int distance_to_cells(double distance_m, double resolution, int minimum_cells)
{
    if (distance_m <= 0.0) {
        return minimum_cells;
    }

    // 减去极小量，避免 0.15 / 0.05 的浮点误差被错误上取整为 4。
    const auto cells = static_cast<int>(std::ceil(
        distance_m / resolution - kResolutionEpsilon));
    return std::max(minimum_cells, cells);
}
}  // 命名空间

FrontierStrategyParams adapt_strategy_params_to_map_resolution(
    const FrontierStrategyParams & base_params,
    double map_resolution)
{
    if (!base_params.map_adaptation.enabled ||
        !std::isfinite(map_resolution) ||
        map_resolution <= kResolutionEpsilon)
    {
        return base_params;
    }

    auto adapted = base_params;
    const auto & metric = base_params.map_adaptation;
    adapted.runtime.obstacle_search_radius_cells = distance_to_cells(
        metric.obstacle_clearance_m, map_resolution, 0);
    adapted.runtime.min_frontier_cluster_size = distance_to_cells(
        metric.min_frontier_length_m, map_resolution, 1);
    adapted.pruner.min_cluster_size = static_cast<std::size_t>(
        adapted.runtime.min_frontier_cluster_size);
    adapted.selection.small_cluster_size_threshold = static_cast<std::size_t>(std::max(
        adapted.runtime.min_frontier_cluster_size + 1,
        distance_to_cells(metric.small_frontier_length_m, map_resolution, 1)));
    adapted.pruner.candidate_unknown_margin_cells = distance_to_cells(
        metric.candidate_unknown_margin_m, map_resolution, 0);
    adapted.pruner.candidate_goal_inset_cells = distance_to_cells(
        metric.candidate_goal_inset_m, map_resolution, 0);
    adapted.pruner.cleanup_goal_inset_cells = distance_to_cells(
        metric.cleanup_goal_inset_m, map_resolution, 0);
    return adapted;
}

}  // 命名空间 frontier_strategy
