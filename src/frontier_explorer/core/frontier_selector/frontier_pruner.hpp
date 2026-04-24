#pragma once

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_decision_types.hpp"
#include "types.h"

namespace frontier_explorer
{

struct FrontierPruningContext
{
    std::optional<GridCell> last_goal;

    const std::unordered_map<GridCell, int, GridCellHash> * failed_goal_counts{nullptr};
    const std::unordered_set<GridCell, GridCellHash> * goal_blacklist{nullptr};

    const std::unordered_map<GridCell, int, GridCellHash> * failed_cluster_counts{nullptr};
    const std::unordered_set<GridCell, GridCellHash> * cluster_blacklist{nullptr};
};

class FrontierPruner
{
public:
    FrontierPruner(
        double min_goal_distance_m,
        int max_retry_count,
        int max_cluster_retry_count,
        std::size_t min_cluster_size,
        int unknown_margin_cells,
        double max_unknown_ratio);

    std::vector<FrontierSelectionCandidate> prune_clusters(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid * map,
        const FrontierPruningContext & context,
        std::vector<GridCell> * failed_cluster_ids = nullptr) const;

private:
    double distance_in_meters(
        const GridCell & a,
        const GridCell & b,
        double resolution) const;

    bool is_same_as_last_goal(
        const GridCell & goal,
        const std::optional<GridCell> & last_goal) const;

    bool should_skip_goal(
        const GridCell & goal,
        const FrontierPruningContext & context) const;

    int retry_count_of_goal(
        const GridCell & goal,
        const FrontierPruningContext & context) const;

    std::optional<GridCell> find_fallback_goal_in_cluster(
        const FrontierCluster & cluster,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid * map,
        const FrontierPruningContext & context) const;

    bool pass_map_candidate_constraints(
        const GridCell & cell,
        const nav_msgs::msg::OccupancyGrid * map) const;

private:
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::size_t min_cluster_size_{1U};
    int unknown_margin_cells_{2};
    double max_unknown_ratio_{0.4};
};

}  // namespace frontier_explorer
