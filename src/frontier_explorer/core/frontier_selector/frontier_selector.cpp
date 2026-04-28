#include "frontier_selector.hpp"

#include <algorithm>
#include <sstream>

namespace frontier_explorer
{

FrontierSelector::FrontierSelector(
    double min_goal_distance_m,
    int max_retry_count,
    const FrontierScoringWeights & scoring_weights,
    std::size_t min_cluster_size,
    int max_cluster_retry_count,
    int candidate_unknown_margin_cells,
    bool defer_small_clusters,
    std::size_t small_cluster_size_threshold,
    const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("selector")),
  min_goal_distance_m_(min_goal_distance_m),
  max_retry_count_(max_retry_count),
  max_cluster_retry_count_(max_cluster_retry_count),
  min_cluster_size_(std::max<std::size_t>(1U, min_cluster_size)),
  defer_small_clusters_(defer_small_clusters),
  small_cluster_size_threshold_(
      std::max<std::size_t>(min_cluster_size_, small_cluster_size_threshold)),
  pruner_(
      min_goal_distance_m_,
      max_retry_count_,
      max_cluster_retry_count_,
      min_cluster_size_,
      candidate_unknown_margin_cells,
      logger_),
  scorer_(scoring_weights, max_retry_count_, logger_)
{
}

std::optional<GridCell> FrontierSelector::choose_best_frontier(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const CostmapAdapter & frontier_costmap,
    const CostmapAdapter * safety_costmap)
{
    last_scored_candidates_.clear();

    const FrontierPruningContext context{
        state_.last_goal_grid,
        &state_.failed_goal_counts,
        &state_.blacklist,
        &state_.failed_cluster_counts,
        &state_.cluster_blacklist};

    std::vector<GridCell> fallback_failed_clusters;
    auto valid_candidates = pruner_.prune_clusters(
        clusters,
        robot_grid,
        resolution,
        &frontier_costmap,
        safety_costmap,
        context,
        &fallback_failed_clusters);

    for (const auto & cluster_id : fallback_failed_clusters) {
        mark_cluster_failed(cluster_id);
    }

    if (valid_candidates.empty()) {
        RCLCPP_WARN(logger_, "No valid frontier candidates after pruning.");
        return std::nullopt;
    }

    std::vector<FrontierCandidate> normal_candidates;
    std::vector<FrontierCandidate> small_candidates;
    normal_candidates.reserve(valid_candidates.size());
    small_candidates.reserve(valid_candidates.size());

    if (defer_small_clusters_) {
        for (const auto & candidate : valid_candidates) {
            if (candidate.cluster_size < small_cluster_size_threshold_) {
                small_candidates.push_back(candidate);
            } else {
                normal_candidates.push_back(candidate);
            }
        }
    } else {
        normal_candidates = valid_candidates;
    }

    const auto best = !normal_candidates.empty() ?
        choose_best_scored_candidate(normal_candidates) :
        choose_best_scored_candidate(small_candidates);
    if (!best.has_value()) {
        return std::nullopt;
    }

    mark_cluster_succeeded(best->candidate.cluster_centroid);
    return best->candidate.goal;
}

std::optional<ScoredFrontierCandidate> FrontierSelector::choose_best_scored_candidate(
    const std::vector<FrontierCandidate> & candidates) const
{
    last_scored_candidates_ = scorer_.score_candidates(candidates, state_.last_goal_grid);
    if (last_scored_candidates_.empty()) {
        return std::nullopt;
    }

    const auto best_it = std::max_element(
        last_scored_candidates_.begin(),
        last_scored_candidates_.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            if (lhs.total_score == rhs.total_score) {
                return lhs.candidate.distance_m > rhs.candidate.distance_m;
            }
            return lhs.total_score < rhs.total_score;
        });

    if (best_it == last_scored_candidates_.end()) {
        return std::nullopt;
    }

    log_scored_candidates(last_scored_candidates_, *best_it);
    return *best_it;
}

void FrontierSelector::set_last_goal(const GridCell & goal)
{
    state_.last_goal_grid = goal;
}

void FrontierSelector::mark_goal_failed(const GridCell & goal)
{
    auto & count = state_.failed_goal_counts[goal];
    ++count;

    if (count >= max_retry_count_) {
        state_.blacklist.insert(goal);
        RCLCPP_WARN(
            logger_,
            "Goal blacklisted: row=%d, col=%d, failures=%d, threshold=%d",
            goal.row,
            goal.col,
            count,
            max_retry_count_);
    } else {
        RCLCPP_WARN(
            logger_,
            "Goal failed: row=%d, col=%d, failures=%d/%d",
            goal.row,
            goal.col,
            count,
            max_retry_count_);
    }
}

void FrontierSelector::mark_goal_succeeded(const GridCell & goal)
{
    state_.failed_goal_counts.erase(goal);
    state_.blacklist.erase(goal);
}

const std::vector<ScoredFrontierCandidate> & FrontierSelector::last_scored_candidates() const
{
    return last_scored_candidates_;
}

std::vector<GridCell> FrontierSelector::blacklisted_goals() const
{
    std::vector<GridCell> goals;
    goals.reserve(state_.blacklist.size());
    for (const auto & goal : state_.blacklist) {
        goals.push_back(goal);
    }
    return goals;
}

void FrontierSelector::log_scored_candidates(
    const std::vector<ScoredFrontierCandidate> & scored_candidates,
    const ScoredFrontierCandidate & selected) const
{
    std::vector<ScoredFrontierCandidate> sorted = scored_candidates;
    std::sort(
        sorted.begin(),
        sorted.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            return lhs.total_score > rhs.total_score;
        });

    std::ostringstream stream;
    stream.precision(3);
    stream << std::fixed
           << "Selected frontier reason: goal=("
           << selected.candidate.goal.row << ", " << selected.candidate.goal.col
           << "), total=" << selected.total_score
           << ", distance_score=" << selected.distance_score
           << ", cluster_score=" << selected.cluster_size_score
           << ", clearance_score=" << selected.clearance_score
           << ", retry_penalty=" << selected.retry_penalty
           << ", unknown_risk_penalty=" << selected.unknown_risk_penalty
           << ", information_gain=" << selected.information_gain_score
           << ", distance_m=" << selected.candidate.distance_m
           << ", cluster_size=" << selected.candidate.cluster_size
           << ", clearance_m=" << selected.candidate.clearance_m
           << ", unknown_ratio=" << selected.candidate.unknown_ratio
           << ", retry_count=" << selected.candidate.retry_count
           << ", used_fallback=" << (selected.candidate.used_fallback ? "true" : "false");

    const std::size_t top_count = std::min<std::size_t>(3U, sorted.size());
    for (std::size_t i = 0; i < top_count; ++i) {
        const auto & scored = sorted[i];
        stream << " | top" << (i + 1)
               << " goal=(" << scored.candidate.goal.row << ", " << scored.candidate.goal.col
               << ") total=" << scored.total_score
               << " d=" << scored.distance_score
               << " c=" << scored.cluster_size_score
               << " clear=" << scored.clearance_score
               << " risk=-" << scored.unknown_risk_penalty
               << " retry=-" << scored.retry_penalty;
    }

    RCLCPP_INFO(logger_, "%s", stream.str().c_str());
}

void FrontierSelector::mark_cluster_failed(const GridCell & cluster_id)
{
    auto & count = state_.failed_cluster_counts[cluster_id];
    ++count;
    if (count >= max_cluster_retry_count_) {
        state_.cluster_blacklist.insert(cluster_id);
    }
}

void FrontierSelector::mark_cluster_succeeded(const GridCell & cluster_id)
{
    state_.failed_cluster_counts.erase(cluster_id);
    state_.cluster_blacklist.erase(cluster_id);
}

}  // namespace frontier_explorer
