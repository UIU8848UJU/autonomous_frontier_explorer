#include <gtest/gtest.h>

#include "exploration_nodes/adapters/navigation_converters.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

TEST(NavigationConverters, KeepsNav2ThresholdInOccupancyGridScale)
{
  EXPECT_EQ(
    exploration::adapters::occupancyThresholdFromNav2Cost(
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
    99);
  EXPECT_EQ(
    exploration::adapters::occupancyThresholdFromNav2Cost(
      nav2_costmap_2d::LETHAL_OBSTACLE),
    100);
}

TEST(NavigationConverters, MapsIntermediateNav2CostToOccupancyGridScale)
{
  const auto threshold =
    exploration::adapters::occupancyThresholdFromNav2Cost(128);
  EXPECT_GE(threshold, 50);
  EXPECT_LE(threshold, 51);
}
