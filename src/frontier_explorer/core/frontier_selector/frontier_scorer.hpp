#pragma once

#include <optional>
#include <vector>

#include "frontier_decision_types.hpp"
#include "types.h"

namespace frontier_explorer
{

class FrontierScorer
{
public:
    explicit FrontierScorer(
        FrontierScoringWeights weights = FrontierScoringWeights{},
        int max_retry_count = 2);

    std::vector<ScoredFrontierCandidate> score_candidates(
        const std::vector<FrontierSelectionCandidate> & candidates,
        const std::optional<GridCell> & last_goal) const;

    const FrontierScoringWeights & weights() const;

private:
    double normalized_distance_score(
        double distance,
        double min_distance,
        double max_distance) const;

    double normalized_cluster_size_score(
        std::size_t cluster_size,
        std::size_t min_cluster_size,
        std::size_t max_cluster_size) const;

    double compute_total_score(const ScoredFrontierCandidate & scored) const;

private:
    FrontierScoringWeights weights_{};
    int max_retry_count_{2};
};

}  // namespace frontier_explorer
