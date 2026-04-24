#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_decision_types.hpp"
#include "frontier_pruner.hpp"
#include "frontier_scorer.hpp"
#include "types.h"

namespace frontier_explorer
{

class FrontierSelector
{
public:
    FrontierSelector(
        double min_goal_distance_m,
        int max_retry_count = 2,
        const FrontierScoringWeights & scoring_weights = FrontierScoringWeights{},
        std::size_t min_cluster_size = 1U,
        int max_cluster_retry_count = 3,
        int candidate_unknown_margin_cells = 2,
        double candidate_max_unknown_ratio = 0.4);

    FrontierSelector(
        double min_goal_distance_m,
        int max_retry_count,
        const std::string & strategy_name);

    // Backward-compatibility ctor. Deprecated and no longer used as primary selection path.
    FrontierSelector(
        double min_goal_distance_m,
        int max_retry_count,
        const std::string & strategy_name,
        std::size_t min_cluster_size,
        int max_cluster_retry_count = 3,
        int candidate_unknown_margin_cells = 2,
        double candidate_max_unknown_ratio = 0.4);

    std::optional<GridCell> choose_best_frontier(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution);

    std::optional<GridCell> choose_best_frontier(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid & map);

    void set_last_goal(const GridCell & goal);
    void mark_goal_failed(const GridCell & goal);
    void mark_goal_succeeded(const GridCell & goal);

    bool is_same_as_last_goal(const GridCell & goal) const;
    bool should_skip_goal(const GridCell & goal) const;
    void clear_history();

    std::optional<GridCell> find_fallback_goal_in_cluster(
        const FrontierCluster & cluster,
        const GridCell & robot_grid,
        double resolution) const;

    void mark_cluster_failed(const GridCell & cluster_id);
    void mark_cluster_succeeded(const GridCell & cluster_id);

private:
    static FrontierScoringWeights scoring_weights_from_strategy(
        const std::string & strategy_name);

    std::optional<GridCell> choose_best_frontier_impl(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid * map);

    double distance_in_meters(
        const GridCell & a,
        const GridCell & b,
        double resolution) const;

private:
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::size_t min_cluster_size_{1U};

    FrontierPruner pruner_;
    FrontierScorer scorer_;

    std::optional<GridCell> last_goal_grid_;
    std::unordered_map<GridCell, int, GridCellHash> failed_goal_counts_;
    std::unordered_set<GridCell, GridCellHash> blacklist_;
    std::unordered_map<GridCell, int, GridCellHash> failed_cluster_counts_;
    std::unordered_set<GridCell, GridCellHash> cluster_blacklist_;
};

}  // namespace frontier_explorer
