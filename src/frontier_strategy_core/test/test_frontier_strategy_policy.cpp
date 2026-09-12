#include <cstdint>

#include <gtest/gtest.h>

#include "frontier_strategy_core/policy/frontier_strategy_policy.hpp"

TEST(FrontierStrategyPolicyTest, WaitsUntilMappingHasUsableInputs)
{
    frontier_strategy::FrontierStrategyPolicy policy;
    exploration_core::ExplorationObservation observation;

    auto decision = policy.decide(observation);
    EXPECT_EQ(decision.type, exploration_core::ExplorationDecisionType::WAIT);
    EXPECT_EQ(decision.detail, "mapping_not_active");

    observation.mapping_active = true;
    decision = policy.decide(observation);
    EXPECT_EQ(decision.type, exploration_core::ExplorationDecisionType::WAIT);
    EXPECT_EQ(decision.detail, "map_not_ready");
}

TEST(FrontierStrategyPolicyTest, ReportsCompletionWhenMapHasNoFrontier)
{
    frontier_strategy::FrontierStrategyPolicy policy;
    exploration_core::ExplorationObservation observation;
    observation.mapping_active = true;
    observation.robot_cell = grid_map_core::GridCell{1, 1};

    grid_map_core::GridMap map;
    map.width = 3U;
    map.height = 3U;
    map.resolution = 0.1;
    map.data.assign(map.width * map.height, static_cast<std::int8_t>(0));
    observation.map = map;

    const auto decision = policy.decide(observation);
    EXPECT_EQ(decision.type, exploration_core::ExplorationDecisionType::COMPLETED);
    EXPECT_EQ(decision.detail, "no_frontier_found");
}


