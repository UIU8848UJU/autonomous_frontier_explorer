#include <cstddef>

#include "exploration_bt/bt/feasibility_batch.hpp"
#include "gtest/gtest.h"

namespace exploration
{

TEST(FeasibilityBatchTest, StartsWithConfiguredTopK)
{
    EXPECT_EQ(initial_feasibility_batch_end(16U, 3U), 3U);
    EXPECT_EQ(initial_feasibility_batch_end(2U, 3U), 2U);
    EXPECT_EQ(initial_feasibility_batch_end(16U, 0U), 1U);
}

TEST(FeasibilityBatchTest, ExpandsOnlyWhenCurrentBatchHasNoFeasibleGoal)
{
    EXPECT_EQ(expand_feasibility_batch_end(3U, 16U, 3U), 6U);
    EXPECT_EQ(expand_feasibility_batch_end(15U, 16U, 3U), 16U);
}

}  // 命名空间 exploration
