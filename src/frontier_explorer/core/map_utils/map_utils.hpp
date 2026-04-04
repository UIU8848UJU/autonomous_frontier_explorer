#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"

namespace frontier_explorer
{

bool in_bounds( const nav_msgs::msg::OccupancyGrid & map, int row, int col);

bool is_cell_free(const nav_msgs::msg::OccupancyGrid & map, int row, int col);

bool is_cell_unknown(const nav_msgs::msg::OccupancyGrid & map, int row, int col);

bool is_cell_obstacle(const nav_msgs::msg::OccupancyGrid & map, int row, int col);

geometry_msgs::msg::PoseStamped grid_to_goal_pose(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridCell & cell,
    const rclcpp::Time & stamp);

}  // namespace frontier_explorer