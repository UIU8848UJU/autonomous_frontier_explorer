#include <gtest/gtest.h>

#include "exploration_bt/bt/feasibility_cache_key.hpp"

TEST(FeasibilityCacheKeyTest, SeparatesMapCostmapStartGoalAndPlanner)
{
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 1.24;
    goal.pose.position.y = -0.76;
    const auto base = exploration::make_feasibility_cache_key(
        3U, 4U, 5U, goal, "GridBased", 0.5);

    EXPECT_NE(base, exploration::make_feasibility_cache_key(
        4U, 4U, 5U, goal, "GridBased", 0.5));
    EXPECT_NE(base, exploration::make_feasibility_cache_key(
        3U, 5U, 5U, goal, "GridBased", 0.5));
    EXPECT_NE(base, exploration::make_feasibility_cache_key(
        3U, 4U, 6U, goal, "GridBased", 0.5));
    EXPECT_NE(base, exploration::make_feasibility_cache_key(
        3U, 4U, 5U, goal, "Smac", 0.5));

    goal.pose.position.x += 0.5;
    EXPECT_NE(base, exploration::make_feasibility_cache_key(
        3U, 4U, 5U, goal, "GridBased", 0.5));
}
