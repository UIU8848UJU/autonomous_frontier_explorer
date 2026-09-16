#include <cstdint>

#include <gtest/gtest.h>

#include "frontier_strategy_core/policy/frontier_strategy_policy.hpp"

TEST(FrontierStrategyPolicyTest, WaitsWhenMapHasNoFrontier)
{
    frontier_strategy::FrontierStrategyPolicy policy;

    grid_map_core::GridMap map;
    map.width = 3U;
    map.height = 3U;
    map.resolution = 0.1;
    map.data.assign(map.width * map.height, static_cast<std::int8_t>(0));

    frontier_strategy::FrontierPruningEnvironment environment;
    environment.frontier_map = &map;

    const auto evaluation = policy.evaluate(
        map,
        grid_map_core::GridCell{1, 1},
        environment);
    EXPECT_EQ(
        evaluation.decision.type,
        exploration_core::ExplorationDecisionType::WAIT);
    EXPECT_EQ(evaluation.decision.detail, "no_frontier_found");
}

TEST(FrontierStrategyPolicyTest, UsesCleanupForSingleCellFrontier)
{
    frontier_strategy::FrontierStrategyPolicyConfig config;
    config.obstacle_search_radius_cells = 0;
    config.min_goal_distance_m = 0.0;
    config.min_cluster_size = 2U;
    config.cleanup_min_cluster_size = 1U;
    config.unknown_margin_cells = 1;
    config.goal_inset_cells = 0;
    config.max_unknown_ratio = 0.0;
    config.cleanup_max_unknown_ratio = 0.4;

    frontier_strategy::FrontierStrategyPolicy policy(config);
    grid_map_core::GridMap map;
    map.width = 5U;
    map.height = 5U;
    map.resolution = 1.0;
    map.data.assign(map.width * map.height, static_cast<std::int8_t>(100));
    map.data[2U * map.width + 2U] = static_cast<std::int8_t>(-1);
    map.data[2U * map.width + 1U] = static_cast<std::int8_t>(0);

    frontier_strategy::FrontierPruningEnvironment environment;
    environment.frontier_map = &map;

    const auto evaluation = policy.evaluate(
        map,
        grid_map_core::GridCell{4, 4},
        environment);
    EXPECT_EQ(
        evaluation.decision.type,
        exploration_core::ExplorationDecisionType::NAVIGATE);
    EXPECT_TRUE(evaluation.cleanup_mode);
    ASSERT_TRUE(evaluation.decision.goal.has_value());
    EXPECT_EQ(evaluation.decision.goal.value(), (grid_map_core::GridCell{2, 1}));
}

TEST(FrontierStrategyPolicyTest, CleanupAcceptsNearbyResidualFrontier)
{
    frontier_strategy::FrontierStrategyPolicyConfig config;
    config.obstacle_search_radius_cells = 0;
    config.min_goal_distance_m = 0.45;
    config.cleanup_min_goal_distance_m = 0.0;
    config.min_cluster_size = 2U;
    config.cleanup_min_cluster_size = 1U;
    config.unknown_margin_cells = 0;
    config.goal_inset_cells = 2;
    config.cleanup_goal_inset_cells = 0;
    config.max_unknown_ratio = 0.0;
    config.cleanup_max_unknown_ratio = 0.4;
    config.cleanup_trigger_no_candidate_cycles = 1;

    frontier_strategy::FrontierStrategyPolicy policy(config);
    grid_map_core::GridMap map;
    map.width = 5U;
    map.height = 5U;
    map.resolution = 0.1;
    map.data.assign(map.width * map.height, static_cast<std::int8_t>(100));
    map.data[2U * map.width + 1U] = static_cast<std::int8_t>(0);
    map.data[2U * map.width + 2U] = static_cast<std::int8_t>(0);
    map.data[2U * map.width + 3U] = static_cast<std::int8_t>(-1);

    frontier_strategy::FrontierPruningEnvironment environment;
    environment.frontier_map = &map;

    const auto evaluation = policy.evaluate(
        map,
        grid_map_core::GridCell{2, 1},
        environment);
    EXPECT_EQ(
        evaluation.decision.type,
        exploration_core::ExplorationDecisionType::NAVIGATE);
    EXPECT_TRUE(evaluation.cleanup_mode);
    ASSERT_TRUE(evaluation.decision.goal.has_value());
    EXPECT_EQ(evaluation.decision.goal.value(), (grid_map_core::GridCell{2, 2}));
}


