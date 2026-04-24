#include "frontier_scorer.hpp"

#include <algorithm>

namespace frontier_explorer
{

FrontierScorer::FrontierScorer(
    FrontierScoringWeights weights,
    int max_retry_count)
: weights_(weights),
  max_retry_count_(std::max(1, max_retry_count))
{
}

const FrontierScoringWeights & FrontierScorer::weights() const
{
    return weights_;
}

double FrontierScorer::normalized_distance_score(
    double distance,
    double min_distance,
    double max_distance) const
{
    if (max_distance <= min_distance) {
        return 1.0;
    }

    const double normalized = (distance - min_distance) / (max_distance - min_distance);
    return 1.0 - std::clamp(normalized, 0.0, 1.0);
}

double FrontierScorer::normalized_cluster_size_score(
    std::size_t cluster_size,
    std::size_t min_cluster_size,
    std::size_t max_cluster_size) const
{
    if (max_cluster_size <= min_cluster_size) {
        return 1.0;
    }

    const double normalized = static_cast<double>(cluster_size - min_cluster_size) /
        static_cast<double>(max_cluster_size - min_cluster_size);

    return std::clamp(normalized, 0.0, 1.0);
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

    total -= weights_.weight_retry_penalty * scored.retry_penalty;
    return total;
}

std::vector<ScoredFrontierCandidate> FrontierScorer::score_candidates(
    const std::vector<FrontierSelectionCandidate> & candidates,
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
        [](const FrontierSelectionCandidate & lhs, const FrontierSelectionCandidate & rhs) {
            return lhs.distance_m < rhs.distance_m;
        });

    auto cluster_size_range = std::minmax_element(
        candidates.begin(),
        candidates.end(),
        [](const FrontierSelectionCandidate & lhs, const FrontierSelectionCandidate & rhs) {
            return lhs.cluster_size < rhs.cluster_size;
        });

    const double min_distance = distance_range.first->distance_m;
    const double max_distance = distance_range.second->distance_m;

    const std::size_t min_cluster_size = cluster_size_range.first->cluster_size;
    const std::size_t max_cluster_size = cluster_size_range.second->cluster_size;

    for (const auto & candidate : candidates) {
        ScoredFrontierCandidate scored;
        scored.candidate = candidate;
        scored.distance_score =
            normalized_distance_score(candidate.distance_m, min_distance, max_distance);
        scored.cluster_size_score =
            normalized_cluster_size_score(candidate.cluster_size, min_cluster_size, max_cluster_size);

        scored.clearance_score = 0.0;
        scored.revisit_penalty =
            (last_goal.has_value() && candidate.goal == last_goal.value()) ? 1.0 : 0.0;

        const double retry_norm =
            static_cast<double>(std::max(0, candidate.retry_count)) /
            static_cast<double>(max_retry_count_);
        scored.retry_penalty = std::clamp(retry_norm, 0.0, 1.0);

        scored.total_score = compute_total_score(scored);
        scored_candidates.push_back(scored);
    }

    return scored_candidates;
}

}  // namespace frontier_explorer
