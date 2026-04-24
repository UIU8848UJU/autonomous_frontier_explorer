#include "frontier_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"

namespace frontier_explorer
{

FrontierSelector::FrontierSelector(
    double min_goal_distance_m,
    int max_retry_count,
    const FrontierScoringWeights & scoring_weights,
    std::size_t min_cluster_size,
    int max_cluster_retry_count,
    int candidate_unknown_margin_cells,
    double candidate_max_unknown_ratio)
: min_goal_distance_m_(min_goal_distance_m),
  max_retry_count_(max_retry_count),
  max_cluster_retry_count_(max_cluster_retry_count),
  min_cluster_size_(std::max<std::size_t>(1U, min_cluster_size)),
  pruner_(
      min_goal_distance_m_,
      max_retry_count_,
      max_cluster_retry_count_,
      min_cluster_size_,
      candidate_unknown_margin_cells,
      candidate_max_unknown_ratio),
  scorer_(scoring_weights, max_retry_count_)
{
}

FrontierSelector::FrontierSelector(
    double min_goal_distance_m,
    int max_retry_count,
    const std::string & strategy_name)
: FrontierSelector(
      min_goal_distance_m,
      max_retry_count,
      strategy_name,
      1U,
      3,
      2,
      0.4)
{
}

FrontierSelector::FrontierSelector(
    double min_goal_distance_m,
    int max_retry_count,
    const std::string & strategy_name,
    std::size_t min_cluster_size,
    int max_cluster_retry_count,
    int candidate_unknown_margin_cells,
    double candidate_max_unknown_ratio)
: FrontierSelector(
      min_goal_distance_m,
      max_retry_count,
      scoring_weights_from_strategy(strategy_name),
      min_cluster_size,
      max_cluster_retry_count,
      candidate_unknown_margin_cells,
      candidate_max_unknown_ratio)
{
    RCLCPP_WARN(
        rclcpp::get_logger("frontier_selector"),
        "selection_strategy='%s' is deprecated. Use frontier_decision weights instead.",
        strategy_name.c_str());
}

FrontierScoringWeights FrontierSelector::scoring_weights_from_strategy(
    const std::string & strategy_name)
{
    FrontierScoringWeights weights;

    if (strategy_name == "nearest") {
        weights.weight_distance = 1.0;
        weights.weight_cluster_size = 0.0;
        return weights;
    }

    if (strategy_name == "largest_cluster") {
        weights.weight_distance = 0.0;
        weights.weight_cluster_size = 1.0;
        return weights;
    }

    // Default hybrid-like balance.
    weights.weight_distance = 1.0;
    weights.weight_cluster_size = 1.0;
    return weights;
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
    return choose_best_frontier_impl(clusters, robot_grid, resolution, nullptr);
}

std::optional<GridCell> FrontierSelector::choose_best_frontier(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const nav_msgs::msg::OccupancyGrid & map)
{
    return choose_best_frontier_impl(clusters, robot_grid, resolution, &map);
}

std::optional<GridCell> FrontierSelector::choose_best_frontier_impl(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const nav_msgs::msg::OccupancyGrid * map)
{
    const FrontierPruningContext context{
        last_goal_grid_,
        &failed_goal_counts_,
        &blacklist_,
        &failed_cluster_counts_,
        &cluster_blacklist_};

    std::vector<GridCell> fallback_failed_clusters;
    auto valid_candidates = pruner_.prune_clusters(
        clusters,
        robot_grid,
        resolution,
        map,
        context,
        &fallback_failed_clusters);

    for (const auto & cluster_id : fallback_failed_clusters) {
        mark_cluster_failed(cluster_id);
    }

    if (valid_candidates.empty()) {
        return std::nullopt;
    }

    const auto scored_candidates = scorer_.score_candidates(valid_candidates, last_goal_grid_);
    if (scored_candidates.empty()) {
        return std::nullopt;
    }

    const auto best_it = std::max_element(
        scored_candidates.begin(),
        scored_candidates.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            if (lhs.total_score == rhs.total_score) {
                return lhs.candidate.distance_m > rhs.candidate.distance_m;
            }
            return lhs.total_score < rhs.total_score;
        });

    if (best_it == scored_candidates.end()) {
        return std::nullopt;
    }

    mark_cluster_succeeded(best_it->candidate.cluster_centroid);
    return best_it->candidate.goal;
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
    failed_cluster_counts_.clear();
    cluster_blacklist_.clear();
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
    failed_cluster_counts_.erase(cluster_id);
    cluster_blacklist_.erase(cluster_id);
}

}  // namespace frontier_explorer
