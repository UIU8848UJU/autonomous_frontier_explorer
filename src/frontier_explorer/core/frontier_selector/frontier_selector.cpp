#include "frontier_selector.hpp"

#include <algorithm>

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
    std::size_t small_cluster_size_threshold)
: min_goal_distance_m_(min_goal_distance_m),
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
      candidate_unknown_margin_cells),
  scorer_(scoring_weights, max_retry_count_)
{
}

std::optional<GridCell> FrontierSelector::choose_best_frontier(
    const std::vector<FrontierCluster> & clusters,
    const GridCell & robot_grid,
    double resolution,
    const nav_msgs::msg::OccupancyGrid & map)
{
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
        &map,
        context,
        &fallback_failed_clusters);

    for (const auto & cluster_id : fallback_failed_clusters) {
        mark_cluster_failed(cluster_id);
    }

    if (valid_candidates.empty()) {
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
    const auto scored_candidates = scorer_.score_candidates(candidates, state_.last_goal_grid);
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
    }
}

void FrontierSelector::mark_goal_succeeded(const GridCell & goal)
{
    state_.failed_goal_counts.erase(goal);
    state_.blacklist.erase(goal);
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
