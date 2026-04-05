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

/// @brief 地图坐标转换为格子
std::optional<GridCell> world_to_grid(
    const geometry_msgs::msg::Point & pos,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map_msg);

std::optional<geometry_msgs::msg::Point> grid_to_world(
    const GridCell & cell,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map_msg);

double distance_in_meters(const geometry_msgs::msg::Point & p1,
                          const geometry_msgs::msg::Point & p2);


}  // namespace frontier_explorer