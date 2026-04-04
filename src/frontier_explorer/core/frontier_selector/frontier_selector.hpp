#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"

namespace frontier_explorer
{

class FrontierSelector
{
public:
    FrontierSelector(double min_goal_distance_m, int max_retry_count = 2);

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
private:
    double distance_in_meters(
        const GridCell & a,
        const GridCell & b,
        double resolution) const;

private:
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};

    std::optional<GridCell> last_goal_grid_;
    std::unordered_map<GridCell, int, GridCellHash> failed_goal_counts_;
    std::unordered_set<GridCell, GridCellHash> blacklist_;
};

}  // namespace frontier_explorer