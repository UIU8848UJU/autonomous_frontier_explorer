#include "frontier_scorer.hpp"

#include <algorithm>

namespace frontier_explorer
{

FrontierScorer::FrontierScorer(
    FrontierScoringWeights weights,
    int max_retry_count)
: weights_(weights),
  retry_penalty_score_(max_retry_count),
  unknown_risk_penalty_score_(weights.unknown_risk_threshold)
{
}

const FrontierScoringWeights & FrontierScorer::weights() const
{
    return weights_;
}

double FrontierScorer::compute_total_score(const ScoredFrontierCandidate & scored) const
{
    double total = 0.0;
    total += weights_.weight_distance * scored.distance_score;
    total += weights_.weight_cluster_size * scored.cluster_size_score;

    if (weights_.enable_clearance_score) {
        total += weights_.weight_clearance * scored.clearance_score;
    }

    if (weights_.enable_revisit_penalty) {
        total -= weights_.weight_revisit_penalty * scored.revisit_penalty;
    }

    if (weights_.enable_unknown_risk_penalty) {
        total -= weights_.weight_unknown_risk_penalty * scored.unknown_risk_penalty;
    }

    if (weights_.enable_information_gain_score) {
        total += weights_.weight_information_gain * scored.information_gain_score;
    }

    total -= weights_.weight_retry_penalty * scored.retry_penalty;
    return total;
}

std::vector<ScoredFrontierCandidate> FrontierScorer::score_candidates(
    const std::vector<FrontierCandidate> & candidates,
    const std::optional<GridCell> & last_goal) const
{
    std::vector<ScoredFrontierCandidate> scored_candidates;
    if (candidates.empty()) {
        return scored_candidates;
    }

    scored_candidates.reserve(candidates.size());

    auto distance_range = std::minmax_element(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & lhs, const FrontierCandidate & rhs) {
            return lhs.distance_m < rhs.distance_m;
        });

    auto cluster_size_range = std::minmax_element(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & lhs, const FrontierCandidate & rhs) {
            return lhs.cluster_size < rhs.cluster_size;
        });

    auto clearance_range = std::minmax_element(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & lhs, const FrontierCandidate & rhs) {
            return lhs.clearance_m < rhs.clearance_m;
        });

    const double min_distance = distance_range.first->distance_m;
    const double max_distance = distance_range.second->distance_m;

    const std::size_t min_cluster_size = cluster_size_range.first->cluster_size;
    const std::size_t max_cluster_size = cluster_size_range.second->cluster_size;
    const double max_clearance = clearance_range.second->clearance_m;

    for (const auto & candidate : candidates) {
        ScoredFrontierCandidate scored;
        scored.candidate = candidate;
        scored.distance_score = distance_score_.score(candidate, min_distance, max_distance);
        scored.cluster_size_score =
            cluster_size_score_.score(candidate, min_cluster_size, max_cluster_size);
        scored.clearance_score = clearance_score_.score(candidate, max_clearance);
        scored.revisit_penalty =
            (last_goal.has_value() && candidate.goal == last_goal.value()) ? 1.0 : 0.0;
        scored.retry_penalty = retry_penalty_score_.score(candidate);
        scored.unknown_risk_penalty = unknown_risk_penalty_score_.score(candidate);
        scored.information_gain_score = information_gain_score_.score(candidate);

        scored.total_score = compute_total_score(scored);
        scored_candidates.push_back(scored);
    }

    return scored_candidates;
}

}  // namespace frontier_explorer
