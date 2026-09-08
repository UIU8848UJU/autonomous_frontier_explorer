#pragma once

#include "map_manager_core/map_manager_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"

namespace map_manager::adapters
{

MapSnapshot to_core_map(const nav_msgs::msg::OccupancyGrid & message);
ExplorationState to_core_exploration_state(
  const robot_interfaces::msg::ExplorationState & message);

}  // namespace map_manager::adapters
