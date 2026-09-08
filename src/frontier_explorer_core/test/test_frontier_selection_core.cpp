#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

#include "frontier_explorer_core/selector/filters/frontier_pruner.hpp"
#include "frontier_explorer_core/selector/frontier_selection_policy.hpp"
#include "gtest/gtest.h"

namespace frontier_explorer
{
namespace
{

GridMap make_grid_map(unsigned int width, unsigned int height, std::int8_t value = 0)
{
    GridMap map;
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

}  // namespace
}  // namespace frontier_explorer
