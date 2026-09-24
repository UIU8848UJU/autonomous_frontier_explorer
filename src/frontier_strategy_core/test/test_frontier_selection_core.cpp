#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "frontier_strategy_core/information_gain/information_gain_estimator.hpp"
#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "frontier_strategy_core/selector/frontier_selection_policy.hpp"
#include "gtest/gtest.h"

namespace frontier_strategy
{
namespace
{

grid_map_core::GridMap make_grid_map(unsigned int width, unsigned int height, std::int8_t value = 0)
{
    grid_map_core::GridMap map;
    map.width = width;
    map.height = height;
    map.resolution = 1.0;
    map.origin_x = -2.0;
    map.origin_y = 3.0;
    map.data.assign(static_cast<std::size_t>(width) * height, value);
    return map;
}

FrontierCandidate make_candidate(
    const GridCell & goal,
    const GridCell & cluster,
    std::size_t cluster_size,
    double distance)
{
    FrontierCandidate candidate;
    candidate.goal = goal;
    candidate.cluster_centroid = cluster;
    candidate.cluster_size = cluster_size;
    candidate.distance_m = distance;
    return candidate;
}

class RecordingRanker final : public IFrontierRanker
{
public:
    std::vector<ScoredFrontierCandidate> rank(
        const std::vector<FrontierCandidate> & candidates,
        const std::optional<GridCell> &) const override
    {
        ++call_count;
        std::vector<ScoredFrontierCandidate> ranked;
        ranked.reserve(candidates.size());
        for (const auto & candidate : candidates) {
            ScoredFrontierCandidate scored;
            scored.candidate = candidate;
            scored.total_score = static_cast<double>(ranked.size());
            ranked.push_back(scored);
        }
        return ranked;
    }

    mutable int call_count{0};
};

TEST(GridMapSelectionCoreTest, ConvertsCoordinatesAndComputesClearance)
{
    auto map = make_grid_map(6U, 6U);
    map.data[2U * map.width + 4U] = 100;

    double world_x = 0.0;
    double world_y = 0.0;
    ASSERT_TRUE(map.mapToWorld(GridCell{2, 2}, world_x, world_y));
    EXPECT_DOUBLE_EQ(world_x, 0.5);
    EXPECT_DOUBLE_EQ(world_y, 5.5);

    GridCell round_trip;
    ASSERT_TRUE(map.worldToMap(world_x, world_y, round_trip));
    EXPECT_EQ(round_trip, (GridCell{2, 2}));

    const auto clearance = map.distanceToNearestObstacle(GridCell{2, 2}, 3);
    ASSERT_TRUE(clearance.has_value());
    EXPECT_DOUBLE_EQ(clearance.value(), 2.0);
}

TEST(GridMapSelectionCoreTest, EstimatesVisibleUnknownCellsAndStopsAtObstacles)
{
    auto map = make_grid_map(7U, 7U);
    map.data[3U * map.width + 4U] = -1;
    map.data[3U * map.width + 5U] = -1;

    const InformationGainEstimator estimator(4.0);
    const auto visible = estimator.estimate(map, GridCell{3, 3});
    ASSERT_TRUE(visible.valid);
    EXPECT_EQ(visible.visible_unknown_cells, 2U);
    EXPECT_DOUBLE_EQ(visible.visible_unknown_area_m2, 2.0);

    map.data[3U * map.width + 4U] = 100;
    const auto occluded = estimator.estimate(map, GridCell{3, 3});
    ASSERT_TRUE(occluded.valid);
    EXPECT_EQ(occluded.visible_unknown_cells, 0U);
    EXPECT_DOUBLE_EQ(occluded.visible_unknown_area_m2, 0.0);
}

TEST(InformationGainEstimatorTest, ExcludesUnknownCellsOutsideSensorRange)
{
    auto map = make_grid_map(7U, 7U);
    map.data[3U * map.width + 5U] = -1;

    const auto estimate = InformationGainEstimator(1.5).estimate(map, GridCell{3, 3});

    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.visible_unknown_cells, 0U);
}

TEST(InformationGainEstimatorTest, DoesNotSeeThroughObstacleCorners)
{
    auto map = make_grid_map(5U, 5U);
    map.data[2U * map.width + 2U] = -1;
    map.data[1U * map.width + 2U] = 100;

    const auto estimate = InformationGainEstimator(3.0).estimate(map, GridCell{1, 1});

    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.visible_unknown_cells, 0U);
}

TEST(InformationGainEstimatorTest, DoesNotMistakeOrdinaryDiagonalStepsForCornerCrossings)
{
    const GridCell viewpoint{4, 4};
    const std::vector<std::pair<GridCell, GridCell>> cases{
        {{6, 7}, {5, 4}},
        {{2, 7}, {3, 4}},
        {{6, 1}, {5, 4}},
        {{2, 1}, {3, 4}},
        {{7, 6}, {4, 5}},
        {{1, 6}, {4, 5}},
        {{7, 2}, {4, 3}},
        {{1, 2}, {4, 3}},
    };

    for (const auto & [target, side_obstacle] : cases) {
        auto map = make_grid_map(9U, 9U);
        map.data[static_cast<std::size_t>(target.row) * map.width + target.col] = -1;
        map.data[static_cast<std::size_t>(side_obstacle.row) * map.width +
            side_obstacle.col] = 100;

        const auto estimate = InformationGainEstimator(5.0).estimate(map, viewpoint);

        ASSERT_TRUE(estimate.valid);
        EXPECT_EQ(estimate.visible_unknown_cells, 1U)
            << "target=(" << target.row << "," << target.col << ")";
    }
}

TEST(InformationGainEstimatorTest, SafelySaturatesVeryLargeFiniteRangeToMapBounds)
{
    auto map = make_grid_map(5U, 5U);
    map.data[4U * map.width + 4U] = -1;

    const auto estimate = InformationGainEstimator(
        std::numeric_limits<double>::max()).estimate(map, GridCell{0, 0});

    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.visible_unknown_cells, 1U);
}

TEST(InformationGainEstimatorTest, ExaminesEachCellInTheRangeBoundingBoxOnlyOnce)
{
    auto map = make_grid_map(121U, 121U, -1);
    map.resolution = 0.05;
    const GridCell viewpoint{60, 60};
    map.data[static_cast<std::size_t>(viewpoint.row) * map.width + viewpoint.col] = 0;

    std::size_t expected_unknown_cells = 0U;
    for (int row = 0; row < static_cast<int>(map.height); ++row) {
        for (int col = 0; col < static_cast<int>(map.width); ++col) {
            if (row == viewpoint.row && col == viewpoint.col) {
                continue;
            }
            if (std::hypot(
                    static_cast<double>(row - viewpoint.row),
                    static_cast<double>(col - viewpoint.col)) * map.resolution <= 3.0)
            {
                ++expected_unknown_cells;
            }
        }
    }

    const auto estimate = InformationGainEstimator(3.0).estimate(map, viewpoint);

    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.visible_unknown_cells, expected_unknown_cells);
    EXPECT_EQ(estimate.examined_cells, map.data.size());
}

TEST(InformationGainEstimatorTest, UsesPhysicalAreaAcrossMapResolutions)
{
    auto coarse = make_grid_map(9U, 9U);
    coarse.origin_x = 0.0;
    coarse.origin_y = 0.0;
    coarse.data[4U * coarse.width + 5U] = -1;

    auto fine = make_grid_map(18U, 18U);
    fine.resolution = 0.5;
    fine.origin_x = 0.0;
    fine.origin_y = 0.0;
    fine.data[8U * fine.width + 10U] = -1;
    fine.data[8U * fine.width + 11U] = -1;
    fine.data[9U * fine.width + 10U] = -1;
    fine.data[9U * fine.width + 11U] = -1;

    const InformationGainEstimator estimator(3.0);
    const auto coarse_estimate = estimator.estimate(coarse, GridCell{4, 4});
    const auto fine_estimate = estimator.estimate(fine, GridCell{8, 8});

    ASSERT_TRUE(coarse_estimate.valid);
    ASSERT_TRUE(fine_estimate.valid);
    EXPECT_DOUBLE_EQ(coarse_estimate.visible_unknown_area_m2, 1.0);
    EXPECT_DOUBLE_EQ(fine_estimate.visible_unknown_area_m2, 1.0);
}

TEST(InformationGainEstimatorTest, IsInvalidWhenExplicitlyDisabled)
{
    const auto estimate = InformationGainEstimator(0.0).estimate(
        make_grid_map(5U, 5U), GridCell{2, 2});

    EXPECT_FALSE(estimate.valid);
    EXPECT_DOUBLE_EQ(estimate.visible_unknown_area_m2, 0.0);
}

TEST(FrontierPrunerSelectionCoreTest, RejectsCandidateWhenRequiredGainCannotBeEstimated)
{
    FrontierPruner pruner(
        0.0,
        2,
        3,
        1U,
        0,
        0,
        1.0,
        {},
        {},
        30.0,
        3.0,
        0.1);
    FrontierPruningEnvironment environment;
    const std::vector<FrontierCluster> clusters{
        FrontierCluster{{GridCell{3, 3}}, GridCell{3, 3}}};

    const auto candidates = pruner.prune_clusters(
        clusters,
        GridCell{0, 0},
        1.0,
        environment,
        FrontierPruningContext{});

    EXPECT_TRUE(candidates.empty());
}

TEST(FrontierPrunerSelectionCoreTest, UsesPureMapAndInjectedPlatformChecks)
{
    auto map = make_grid_map(10U, 10U);
    FrontierPruner pruner(1.0, 2, 3, 1U, 0, 1, 1.0);

    std::vector<GridCell> safety_checks;
    FrontierPruningEnvironment environment;
    environment.frontier_map = &map;
    environment.safety_check = [&safety_checks](const GridCell & cell) {
            safety_checks.push_back(cell);
            return !(cell == GridCell{1, 1});
        };
    environment.clearance_query = [](const GridCell &) {
            return std::optional<double>{2.5};
        };

    const std::vector<FrontierCluster> clusters{
        FrontierCluster{{GridCell{5, 7}, GridCell{5, 8}}, GridCell{5, 7}},
        FrontierCluster{{GridCell{1, 1}}, GridCell{1, 1}}};

    const auto candidates = pruner.prune_clusters(
        clusters,
        GridCell{5, 4},
        1.0,
        environment,
        FrontierPruningContext{});

    ASSERT_FALSE(candidates.empty());
    const auto inset = std::find_if(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & candidate) {
            return candidate.goal == GridCell{5, 6};
        });
    ASSERT_NE(inset, candidates.end());
    EXPECT_TRUE(inset->goal_inset_applied);
    EXPECT_DOUBLE_EQ(inset->clearance_m, 2.5);
    EXPECT_TRUE(std::find(
        safety_checks.begin(),
        safety_checks.end(),
        GridCell{1, 1}) != safety_checks.end());
    EXPECT_TRUE(std::none_of(
        candidates.begin(),
        candidates.end(),
        [](const FrontierCandidate & candidate) {
            return candidate.goal == GridCell{1, 1};
        }));
}

TEST(FrontierPrunerSelectionCoreTest, ReportsCandidateRejectionReasons)
{
    auto map = make_grid_map(5U, 5U);
    map.data[2U * map.width + 2U] = -1;
    // 第二个 cluster 的邻域含未知格；将阈值设为零，确保该门禁稳定触发。
    FrontierPruner pruner(0.5, 2, 3, 2U, 2, 1, 0.0);
    FrontierDecisionDiagnostics diagnostics;

    const std::vector<FrontierCluster> clusters{
        FrontierCluster{{GridCell{1, 1}}, GridCell{1, 1}},
        FrontierCluster{{GridCell{2, 1}, GridCell{2, 2}}, GridCell{2, 1}}};

    FrontierPruningEnvironment environment;
    environment.frontier_map = &map;
    environment.safety_check = [](const GridCell &) { return true; };

    const auto candidates = pruner.prune_clusters(
        clusters,
        GridCell{4, 4},
        1.0,
        environment,
        FrontierPruningContext{},
        nullptr,
        &diagnostics);

    EXPECT_TRUE(candidates.empty());
    EXPECT_EQ(
        diagnostics.rejection_count(FrontierRejectionReason::CLUSTER_TOO_SMALL),
        1U);
    EXPECT_GT(
        diagnostics.rejection_count(FrontierRejectionReason::UNKNOWN_RATIO_TOO_HIGH),
        0U);
}

TEST(FrontierSelectionPolicyTest, DefersSmallPoolAndKeepsReachabilityReason)
{
    FrontierSelectionCoreConfig config;
    config.defer_small_clusters = true;
    config.small_cluster_size_threshold = 3U;
    config.require_reachable_goal = true;
    FrontierSelectionPolicy policy(config, FrontierScoringWeights{});

    const std::vector<FrontierCandidate> candidates{
        make_candidate(GridCell{4, 10}, GridCell{4, 10}, 3U, 10.0),
        make_candidate(GridCell{4, 2}, GridCell{4, 2}, 2U, 2.0)};
    const auto reachability = [](FrontierCandidate & candidate) {
            FrontierReachabilityResult result;
            result.checked = true;
            result.reachable = candidate.goal.col < 10;
            result.reason = result.reachable ? "reachable" : "blocked";
            return result;
        };

    const auto selected = policy.choose_best_candidate(candidates, reachability);
    EXPECT_FALSE(selected.has_value());
    ASSERT_EQ(policy.last_scored_candidates().size(), 1U);
    EXPECT_EQ(
        policy.last_scored_candidates().front().candidate.reachability_reason,
        "blocked");

    const auto ranked = policy.rank_candidates(candidates, reachability);
    ASSERT_EQ(ranked.size(), 2U);
    EXPECT_EQ(ranked.front().candidate.cluster_size, 3U);
    EXPECT_EQ(ranked.back().candidate.cluster_size, 2U);
}

TEST(FrontierSelectionPolicyTest, SelectsCandidateWithHigherMeasuredInformationGain)
{
    FrontierSelectionCoreConfig config;
    config.defer_small_clusters = false;
    FrontierScoringWeights weights;
    weights.weight_distance = 0.0;
    weights.weight_cluster_size = 0.0;
    weights.weight_retry_penalty = 0.0;
    weights.weight_information_gain = 1.0;
    weights.enable_unknown_risk_penalty = false;

    auto lower_gain = make_candidate(GridCell{2, 2}, GridCell{2, 2}, 5U, 1.0);
    lower_gain.information_gain = 0.25;
    lower_gain.information_gain_valid = true;
    auto higher_gain = make_candidate(GridCell{3, 3}, GridCell{3, 3}, 5U, 1.0);
    higher_gain.information_gain = 1.0;
    higher_gain.information_gain_valid = true;

    FrontierSelectionPolicy policy(config, weights);
    const auto selected = policy.choose_best_candidate({lower_gain, higher_gain});

    ASSERT_TRUE(selected.has_value());
    EXPECT_EQ(selected->candidate.goal, higher_gain.goal);
    EXPECT_DOUBLE_EQ(selected->candidate.information_gain, 1.0);
    EXPECT_TRUE(selected->candidate.information_gain_valid);
}

TEST(FrontierSelectionPolicyTest, AppliesReachabilityAfterInjectedRanker)
{
    FrontierSelectionCoreConfig config;
    config.defer_small_clusters = false;
    config.require_reachable_goal = true;
    auto ranker = std::make_shared<RecordingRanker>();
    FrontierSelectionPolicy policy(config, FrontierScoringWeights{}, ranker);

    const std::vector<FrontierCandidate> candidates{
        make_candidate(GridCell{4, 10}, GridCell{4, 10}, 3U, 10.0),
        make_candidate(GridCell{4, 2}, GridCell{4, 2}, 2U, 2.0)};
    int reachability_call_count = 0;
    const auto selected = policy.choose_best_candidate(
        candidates,
        [&reachability_call_count](FrontierCandidate & candidate) {
            ++reachability_call_count;
            FrontierReachabilityResult result;
            result.checked = true;
            result.reachable = candidate.goal.col < 10;
            result.reason = result.reachable ? "reachable" : "blocked";
            return result;
        });

    ASSERT_TRUE(selected.has_value());
    EXPECT_EQ(selected->candidate.goal, (GridCell{4, 2}));
    EXPECT_EQ(ranker->call_count, 1);
    EXPECT_EQ(reachability_call_count, 2);
    ASSERT_EQ(policy.last_scored_candidates().size(), 2U);
    EXPECT_EQ(
        policy.last_scored_candidates().front().candidate.reachability_reason,
        "blocked");
    EXPECT_EQ(
        policy.last_scored_candidates().back().candidate.reachability_reason,
        "reachable");
}

TEST(FrontierSelectionPolicyTest, ClearsFailuresWithoutClearingLastGoal)
{
    FrontierSelectionCoreConfig config;
    config.max_retry_count = 2;
    FrontierSelectionPolicy policy(config, FrontierScoringWeights{});
    const GridCell goal{3, 4};

    policy.set_last_goal(goal);
    policy.mark_goal_failed(goal);
    policy.mark_goal_failed(goal);
    EXPECT_TRUE(policy.is_goal_blacklisted(goal));
    EXPECT_EQ(policy.clear_blacklist(), 1U);
    EXPECT_FALSE(policy.is_goal_blacklisted(goal));
    EXPECT_EQ(policy.retry_count_for_goal(goal), 0);
    EXPECT_EQ(policy.pruning_context().last_goal, goal);
}

}  // 命名空间
}  // 命名空间 frontier_strategy
