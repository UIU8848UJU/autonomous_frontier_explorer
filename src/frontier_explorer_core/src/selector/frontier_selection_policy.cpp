#include "frontier_explorer_core/selector/frontier_selection_policy.hpp"

#include <utility>

#include "frontier_explorer_core/scoring/frontier_candidate_ranker.hpp"

namespace frontier_explorer
{

FrontierSelectionPolicy::FrontierSelectionPolicy(
    FrontierSelectionCoreConfig config,
    FrontierScoringWeights scoring_weights)
: config_(std::move(config)),
  scorer_(std::move(scoring_weights), config_.max_retry_count)
{
}

FrontierPruningContext FrontierSelectionPolicy::pruning_context() const
{
    return FrontierPruningContext{
        state_.last_goal_grid,
        &state_.failed_goal_counts,
        &state_.blacklist,
        &state_.failed_cluster_counts,
        &state_.cluster_blacklist};
}

void FrontierSelectionPolicy::record_failed_clusters(
    const std::vector<GridCell> & cluster_ids)
{
    for (const auto & cluster_id : cluster_ids) {
        mark_cluster_failed(cluster_id);
    }
}

FrontierSelectionPolicy::CandidatePools FrontierSelectionPolicy::partition_candidates(
    const std::vector<FrontierCandidate> & candidates) const
{
    CandidatePools pools;
    pools.normal.reserve(candidates.size());
    pools.small.reserve(candidates.size());
    if (!config_.defer_small_clusters) {
        pools.normal = candidates;
        return pools;
    }

    for (const auto & candidate : candidates) {
        if (candidate.cluster_size < config_.small_cluster_size_threshold) {
            pools.small.push_back(candidate);
        } else {
            pools.normal.push_back(candidate);
        }
    }
    return pools;
}

std::vector<ScoredFrontierCandidate> FrontierSelectionPolicy::score_and_rank_candidates(
    const std::vector<FrontierCandidate> & candidates,
    const ReachabilityCheck & reachability_check) const
{
    return ::frontier_explorer::rank_frontier_candidates(
        scorer_,
        candidates,
        state_.last_goal_grid,
        reachability_check);
}

std::optional<ScoredFrontierCandidate>
FrontierSelectionPolicy::choose_best_scored_candidate(
    const std::vector<FrontierCandidate> & candidates,
    const ReachabilityCheck & reachability_check)
{
    last_scored_candidates_ = score_and_rank_candidates(candidates, reachability_check);
    for (const auto & scored : last_scored_candidates_) {
        if (config_.require_reachable_goal &&
            scored.candidate.reachability_checked &&
            !scored.candidate.reachable)
        {
            continue;
        }
        return scored;
    }
    return std::nullopt;
}

std::optional<ScoredFrontierCandidate> FrontierSelectionPolicy::choose_best_candidate(
    const std::vector<FrontierCandidate> & candidates,
    const ReachabilityCheck & reachability_check)
{
    last_scored_candidates_.clear();
    const auto pools = partition_candidates(candidates);
    const auto selected = !pools.normal.empty() ?
        choose_best_scored_candidate(pools.normal, reachability_check) :
        choose_best_scored_candidate(pools.small, reachability_check);
    if (selected.has_value()) {
        mark_cluster_succeeded(selected->candidate.cluster_centroid);
    }
    return selected;
}

std::vector<ScoredFrontierCandidate> FrontierSelectionPolicy::rank_candidates(
    const std::vector<FrontierCandidate> & candidates,
    const ReachabilityCheck & reachability_check)
{
    last_scored_candidates_.clear();
    const auto pools = partition_candidates(candidates);
    if (config_.defer_small_clusters && !pools.normal.empty() && !pools.small.empty()) {
        last_scored_candidates_ =
            score_and_rank_candidates(pools.normal, reachability_check);
        auto scored_small = score_and_rank_candidates(pools.small, reachability_check);
        last_scored_candidates_.insert(
            last_scored_candidates_.end(),
            scored_small.begin(),
            scored_small.end());
    } else {
        last_scored_candidates_ = !pools.normal.empty() ?
            score_and_rank_candidates(pools.normal, reachability_check) :
            score_and_rank_candidates(pools.small, reachability_check);
    }

    if (!last_scored_candidates_.empty()) {
        mark_cluster_succeeded(last_scored_candidates_.front().candidate.cluster_centroid);
    }
    return last_scored_candidates_;
}

void FrontierSelectionPolicy::set_last_goal(const GridCell & goal)
{
    state_.last_goal_grid = goal;
}

void FrontierSelectionPolicy::mark_goal_failed(const GridCell & goal)
{
    auto & count = state_.failed_goal_counts[goal];
    ++count;
    if (count >= config_.max_retry_count) {
        state_.blacklist.insert(goal);
    }
}

void FrontierSelectionPolicy::mark_goal_succeeded(const GridCell & goal)
{
    state_.failed_goal_counts.erase(goal);
    state_.blacklist.erase(goal);
}

const std::vector<ScoredFrontierCandidate> &
FrontierSelectionPolicy::last_scored_candidates() const
{
    return last_scored_candidates_;
}

std::vector<GridCell> FrontierSelectionPolicy::blacklisted_goals() const
{
    std::vector<GridCell> goals;
    goals.reserve(state_.blacklist.size());
    for (const auto & goal : state_.blacklist) {
        goals.push_back(goal);
    }
    return goals;
}

int FrontierSelectionPolicy::retry_count_for_goal(const GridCell & goal) const
{
    const auto it = state_.failed_goal_counts.find(goal);
    return it == state_.failed_goal_counts.end() ? 0 : it->second;
}

bool FrontierSelectionPolicy::is_goal_blacklisted(const GridCell & goal) const
{
    return state_.blacklist.find(goal) != state_.blacklist.end();
}

std::size_t FrontierSelectionPolicy::clear_blacklist()
{
    const auto count = state_.blacklist.size() + state_.cluster_blacklist.size();
    state_.blacklist.clear();
    state_.cluster_blacklist.clear();
    state_.failed_goal_counts.clear();
    state_.failed_cluster_counts.clear();
    return count;
}

void FrontierSelectionPolicy::mark_cluster_failed(const GridCell & cluster_id)
{
    auto & count = state_.failed_cluster_counts[cluster_id];
    ++count;
    if (count >= config_.max_cluster_retry_count) {
        state_.cluster_blacklist.insert(cluster_id);
    }
}

void FrontierSelectionPolicy::mark_cluster_succeeded(const GridCell & cluster_id)
{
    state_.failed_cluster_counts.erase(cluster_id);
    state_.cluster_blacklist.erase(cluster_id);
}

}  // namespace frontier_explorer
