#include <cstdint>

#include <gtest/gtest.h>

#include "grid_map_ros/costmap_adapter.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(CostmapAdapter, PreservesOccupancyGridCostScale)
{
  nav_msgs::msg::OccupancyGrid message;
  message.info.width = 5U;
  message.info.height = 1U;
  message.info.resolution = 0.05F;
  message.data = {0, -1, 60, 99, 100};

  grid_map_ros::CostmapAdapter adapter(rclcpp::get_logger("test_costmap_adapter"));
  ASSERT_TRUE(adapter.updateFromOccupancyGrid(message));

  EXPECT_EQ(adapter.getCost(0U, 0U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(adapter.getCost(1U, 0U), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(adapter.getCost(2U, 0U), 153U);
  EXPECT_EQ(adapter.getCost(3U, 0U), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(adapter.getCost(4U, 0U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(adapter.isObstacle(3U, 0U));
  EXPECT_TRUE(adapter.isObstacle(4U, 0U));
}
