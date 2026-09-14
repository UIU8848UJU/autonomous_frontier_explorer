#include "frontier_strategy_core/scoring/frontier_ranker.hpp"

#include <algorithm>
#include <utility>

namespace frontier_strategy
{

RuleBasedFrontierRanker::RuleBasedFrontierRanker(
    FrontierScoringWeights weights,
    int max_retry_count)
: scorer_(std::move(weights), max_retry_count)
{
}

std::vector<ScoredFrontierCandidate> RuleBasedFrontierRanker::rank(
    const std::vector<FrontierCandidate> & candidates,
    const std::optional<GridCell> & last_goal) const
{
    auto scored_candidates = scorer_.score_candidates(candidates, last_goal);
    std::sort(
        scored_candidates.begin(),
        scored_candidates.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            if (lhs.total_score == rhs.total_score) {
                return lhs.candidate.distance_m > rhs.candidate.distance_m;
            }
            return lhs.total_score > rhs.total_score;
        });

    return scored_candidates;
}

}  // 命名空间 frontier_strategy
