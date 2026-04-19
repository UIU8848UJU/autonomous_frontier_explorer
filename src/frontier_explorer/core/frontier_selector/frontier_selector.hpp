#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "frontier_selection_strategy.hpp"
#include "types.h"

namespace frontier_explorer
{

class FrontierSelector
{
public:
    FrontierSelector(
        double min_goal_distance_m,
        int max_retry_count = 2,
        const std::string & strategy_name = "nearest");

    std::optional<GridCell> choose_best_frontier(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution);

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
    double distance_in_meters(
        const GridCell & a,
        const GridCell & b,
        double resolution) const;

private:
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::string selection_strategy_name_{"nearest"};
    std::unique_ptr<IFrontierSelectionStrategy> strategy_;

    std::optional<GridCell> last_goal_grid_;
    std::unordered_map<GridCell, int, GridCellHash> failed_goal_counts_;
    std::unordered_set<GridCell, GridCellHash> blacklist_;
    /// @note:暂时拉黑整个组防止局部来回循环，后续增加多种选择策略
    std::unordered_map<GridCell, int, GridCellHash> failed_cluster_counts_;
    std::unordered_set<GridCell, GridCellHash> cluster_blacklist_;
};

}  // namespace frontier_explorer
