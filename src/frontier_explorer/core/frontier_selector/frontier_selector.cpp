#include "frontier_selector.hpp"

#include <cmath>
#include <limits>

namespace frontier_explorer
{

FrontierSelector::FrontierSelector(
    double min_goal_distance_m,
    int max_retry_count)
: min_goal_distance_m_(min_goal_distance_m),
  max_retry_count_(max_retry_count)
{
}

double FrontierSelector::distance_in_meters( 
    const GridCell & a,
    const GridCell & b,
    double resolution) const
{
    const double dr = static_cast<double>(a.row - b.row);
    const double dc = static_cast<double>(a.col - b.col);
    return std::sqrt(dr * dr + dc * dc) * resolution;
}

bool FrontierSelector::is_same_as_last_goal(const GridCell & goal) const
{
    return last_goal_grid_.has_value() && last_goal_grid_.value() == goal;
}

bool FrontierSelector::should_skip_goal(const GridCell & goal) const
{
    if (blacklist_.count(goal) > 0) {
        return true;
    }

    const auto it = failed_goal_counts_.find(goal);
    if (it != failed_goal_counts_.end() && it->second >= max_retry_count_) {
        return true;
    }

    return false;
}

std::optional<GridCell> FrontierSelector::choose_best_frontier(
  const std::vector<FrontierCluster> & clusters,
  const GridCell & robot_grid,
  double resolution)
{

    double best_dist = std::numeric_limits<double>::max();
    std::optional<GridCell> best_cell;

    for (const auto & cluster : clusters) {

        /// @note 暂定为这样黑名单组
        if (cluster_blacklist_.count(cluster.centroid) > 0) {
            continue;
        } 

        GridCell candidate = cluster.centroid;
        double dist_m = distance_in_meters(robot_grid, candidate, resolution);

        bool centroid_invalid = false;

        if (should_skip_goal(candidate)) 
        {
            RCLCPP_INFO(
                rclcpp::get_logger("frontier_selector"),
                    "Centroid skip=(%d,%d), reason=blacklist_or_retry",
                    candidate.row, candidate.col);
            centroid_invalid = true;
        } 
        else if (dist_m < min_goal_distance_m_) 
        {
            RCLCPP_INFO(
                rclcpp::get_logger("frontier_selector"),
                    "Centroid skip=(%d,%d), reason=too_close dist=%.3f",
                    candidate.row, candidate.col, dist_m);
            centroid_invalid = true;
        } 
        else if (is_same_as_last_goal(candidate)) 
        {
            RCLCPP_INFO(
                rclcpp::get_logger("frontier_selector"),
                    "Centroid skip=(%d,%d), reason=same_as_last_goal",
                    candidate.row, candidate.col);
            centroid_invalid = true;
        }

        if (centroid_invalid) {
            const auto fallback = find_fallback_goal_in_cluster(
                cluster, robot_grid, resolution);

            if (!fallback.has_value()) {

                RCLCPP_INFO(
                    rclcpp::get_logger("frontier_selector"),
                    "Cluster fallback failed for centroid=(%d,%d)",
                    candidate.row, candidate.col);
                    mark_cluster_failed(cluster.centroid);
                continue;
            }

            candidate = fallback.value();
            dist_m = distance_in_meters(robot_grid, candidate, resolution);

            RCLCPP_INFO(
                rclcpp::get_logger("frontier_selector"), "Use fallback goal=(%d,%d), dist=%.3f",
                    candidate.row, candidate.col, dist_m);
        }

        if (dist_m < best_dist) {
            best_dist = dist_m;
            best_cell = candidate;
        }
    }

    return best_cell;
}
void FrontierSelector::set_last_goal(const GridCell & goal)
{
    last_goal_grid_ = goal;
}

void FrontierSelector::mark_goal_failed(const GridCell & goal)
{
    auto & count = failed_goal_counts_[goal];
    ++count;

    if (count >= max_retry_count_) {
        blacklist_.insert(goal);
    }
}

void FrontierSelector::mark_goal_succeeded(const GridCell & goal)
{
    failed_goal_counts_.erase(goal);
    blacklist_.erase(goal);
}

void FrontierSelector::clear_history()
{
    last_goal_grid_.reset();
    failed_goal_counts_.clear();
    blacklist_.clear();
}

std::optional<GridCell> FrontierSelector::find_fallback_goal_in_cluster(
  const FrontierCluster & cluster,
  const GridCell & robot_grid,
  double resolution) const
{
    double best_dist = std::numeric_limits<double>::max();
    std::optional<GridCell> best_cell;

    for (const auto & cell : cluster.cells) {
        if (should_skip_goal(cell)) {
            continue;
        }

        if (is_same_as_last_goal(cell)) {
            continue;
        }

        const double dist_m = distance_in_meters(robot_grid, cell, resolution);
        if (dist_m < min_goal_distance_m_) {
            continue;
        }

        if (dist_m < best_dist) {
            best_dist = dist_m;
            best_cell = cell;
        }
    }

    return best_cell;
}

void FrontierSelector::mark_cluster_failed(const GridCell & cluster_id)
{
    auto & count = failed_cluster_counts_[cluster_id];
    ++count;
    if (count >= max_cluster_retry_count_) {
        cluster_blacklist_.insert(cluster_id);
    }
}

void FrontierSelector::mark_cluster_succeeded(const GridCell & cluster_id)
{
    if(failed_cluster_counts_.count(cluster_id)) failed_cluster_counts_.erase(cluster_id);
    if(cluster_blacklist_.count(cluster_id))cluster_blacklist_.erase(cluster_id);
}

}  // namespace frontier_explorer