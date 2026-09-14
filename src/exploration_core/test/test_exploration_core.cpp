#include <gtest/gtest.h>

#include "exploration_core/types/exploration_decision.hpp"
#include "exploration_core/types/exploration_outcome.hpp"

TEST(ExplorationCoreTest, DecisionAndOutcomeRemainRosFreeDataTypes)
{
    exploration_core::ExplorationDecision decision;
    decision.type = exploration_core::ExplorationDecisionType::NAVIGATE;
    decision.goal = grid_map_core::GridCell{3, 4};
    decision.detail = "frontier_goal_selected";

    exploration_core::ExplorationOutcome outcome;
    outcome.type = exploration_core::ExplorationOutcomeType::NAVIGATION_SUCCEEDED;

    ASSERT_TRUE(decision.goal.has_value());
    EXPECT_EQ(decision.goal.value(), (grid_map_core::GridCell{3, 4}));
    EXPECT_EQ(outcome.type, exploration_core::ExplorationOutcomeType::NAVIGATION_SUCCEEDED);
}
