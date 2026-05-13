#include "frontier_explorer_core/scoring/frontier_scorer.hpp"

#include <algorithm>

namespace frontier_explorer
{

FrontierScorer::FrontierScorer(
    FrontierScoringWeights weights,
    int max_retry_count,
    const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("scorer")),
  weights_(weights),
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

ScoredFrontierCandidate FrontierScorer::score_candidate(
    const ApproachGoalCandidate & candidate,
    const FrontierCluster & cluster,
    const RobotContext & context) const
{
    FrontierCandidate normalized_candidate = candidate.to_frontier_candidate();
    normalized_candidate.cluster_centroid = cluster.centroid;
    if (!cluster.cells.empty()) {
        normalized_candidate.cluster_size = cluster.cells.size();
    }

    ScoredFrontierCandidate scored;
    scored.candidate = normalized_candidate;
    scored.distance_score = distance_score_.score(
        normalized_candidate,
        context.min_candidate_distance_m,
        context.max_candidate_distance_m);
    scored.cluster_size_score = cluster_size_score_.score(
        normalized_candidate,
        context.min_candidate_cluster_size,
        context.max_candidate_cluster_size);
    scored.clearance_score = clearance_score_.score(
        normalized_candidate,
        context.max_candidate_clearance_m);
    scored.revisit_penalty =
        (context.last_goal.has_value() && normalized_candidate.goal == context.last_goal.value()) ?
        1.0 :
        0.0;
    scored.retry_penalty = retry_penalty_score_.score(normalized_candidate);
    scored.unknown_risk_penalty = unknown_risk_penalty_score_.score(normalized_candidate);
    scored.information_gain_score = information_gain_score_.score(normalized_candidate);

    scored.total_score = compute_total_score(scored);
    return scored;
}

double FrontierScorer::score(
    const ApproachGoalCandidate & candidate,
    const FrontierCluster & cluster,
    const RobotContext & context) const
{
    return score_candidate(candidate, cluster, context).total_score;
}

std::vector<ScoredFrontierCandidate> FrontierScorer::score_candidates(
    const std::vector<FrontierCandidate> & candidates,
    const std::optional<GridCell> & last_goal) const
{
    std::vector<ScoredFrontierCandidate> scored_candidates;
    if (candidates.empty()) {
        RCLCPP_DEBUG(logger_, "No candidates to score.");
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
    const RobotContext context{
        last_goal,
        min_distance,
        max_distance,
        min_cluster_size,
        max_cluster_size,
        max_clearance};

    for (const auto & candidate : candidates) {
        FrontierCluster cluster;
        cluster.centroid = candidate.cluster_centroid;
        scored_candidates.push_back(
            score_candidate(ApproachGoalCandidate(candidate), cluster, context));
    }

    return scored_candidates;
}

}  // namespace frontier_explorer
