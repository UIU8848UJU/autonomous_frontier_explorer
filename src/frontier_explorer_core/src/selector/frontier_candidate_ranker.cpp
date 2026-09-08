#include "frontier_explorer_core/scoring/frontier_candidate_ranker.hpp"

#include <algorithm>

namespace frontier_explorer
{

std::vector<ScoredFrontierCandidate> rank_frontier_candidates(
    const FrontierScorer & scorer,
    const std::vector<FrontierCandidate> & candidates,
    const std::optional<GridCell> & last_goal,
    const std::function<FrontierReachabilityResult(FrontierCandidate &)> &
        reachability_check)
{
    auto scored_candidates = scorer.score_candidates(candidates, last_goal);
    if (scored_candidates.empty()) {
        return {};
    }

    std::sort(
        scored_candidates.begin(),
        scored_candidates.end(),
        [](const ScoredFrontierCandidate & lhs, const ScoredFrontierCandidate & rhs) {
            if (lhs.total_score == rhs.total_score) {
                return lhs.candidate.distance_m > rhs.candidate.distance_m;
            }
            return lhs.total_score > rhs.total_score;
        });

    for (auto & scored : scored_candidates) {
        if (!reachability_check) {
            continue;
        }

        const auto reachability = reachability_check(scored.candidate);
        scored.candidate.reachability_reason = reachability.reason;
        if (reachability.checked) {
            scored.candidate.reachability_checked = true;
            scored.candidate.reachable = reachability.reachable;
            scored.candidate.path_length_m = reachability.path_length_m;
        }
    }

    return scored_candidates;
}

}  // namespace frontier_explorer
