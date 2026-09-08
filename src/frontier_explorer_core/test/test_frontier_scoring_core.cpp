#include <optional>
#include <vector>

#include "frontier_explorer_core/scoring/frontier_candidate_ranker.hpp"
#include "frontier_explorer_core/scoring/frontier_scorer.hpp"
#include "gtest/gtest.h"

namespace frontier_explorer
{
namespace
{

FrontierCandidate make_candidate(GridCell goal, double distance, std::size_t cluster_size)
{
    FrontierCandidate candidate;
    candidate.goal = goal;
    candidate.cluster_centroid = goal;
    candidate.distance_m = distance;
    candidate.cluster_size = cluster_size;
    candidate.clearance_m = 1.0;
    return candidate;
}

}  // namespace

TEST(FrontierScoringCore, ScoresCandidatesWithoutROS)
{
    FrontierScoringWeights weights;
    weights.weight_distance = 2.0;
    weights.weight_cluster_size = 0.5;
    weights.enable_clearance_score = false;
    weights.enable_unknown_risk_penalty = false;
    weights.enable_information_gain_score = false;

    const FrontierScorer scorer(weights, 2);
    const auto scored = scorer.score_candidates(
        {
            make_candidate(GridCell{1, 1}, 1.0, 1U),
            make_candidate(GridCell{2, 2}, 3.0, 3U)
        },
        std::nullopt);

    ASSERT_EQ(scored.size(), 2U);
    EXPECT_GT(scored[0].total_score, scored[1].total_score);
    EXPECT_EQ(scored[0].candidate.goal, (GridCell{1, 1}));
}

TEST(FrontierScoringCore, ReturnsEmptyForEmptyCandidateSet)
{
    const FrontierScorer scorer;
    EXPECT_TRUE(scorer.score_candidates({}, std::nullopt).empty());
}

TEST(FrontierScoringCore, RanksAndAnnotatesReachabilityWithoutROS)
{
    FrontierScoringWeights weights;
    weights.weight_distance = 1.0;
    weights.weight_cluster_size = 0.0;
    weights.enable_unknown_risk_penalty = false;
    weights.enable_information_gain_score = false;

    const FrontierScorer scorer(weights, 2);
    const auto ranked = rank_frontier_candidates(
        scorer,
        {
            make_candidate(GridCell{1, 1}, 3.0, 1U),
            make_candidate(GridCell{2, 2}, 1.0, 1U)
        },
        std::nullopt,
        [](FrontierCandidate & candidate) {
            FrontierReachabilityResult result;
            result.checked = true;
            result.reachable = candidate.goal == (GridCell{2, 2});
            result.path_length_m = candidate.distance_m + 0.5;
            return result;
        });

    ASSERT_EQ(ranked.size(), 2U);
    EXPECT_EQ(ranked.front().candidate.goal, (GridCell{2, 2}));
    EXPECT_TRUE(ranked.front().candidate.reachability_checked);
    EXPECT_TRUE(ranked.front().candidate.reachable);
    EXPECT_FALSE(ranked.back().candidate.reachable);
    EXPECT_DOUBLE_EQ(ranked.front().candidate.path_length_m, 1.5);
}

}  // namespace frontier_explorer
