#pragma once

#include <cstdint>
#include <optional>

#include "map_lifecycle_core/map_lifecycle_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"
#include "robot_interfaces/msg/map_lifecycle_state.hpp"

namespace map_lifecycle::adapters
{

grid_map_core::GridMap to_core_map(const nav_msgs::msg::OccupancyGrid & message);

std::uint8_t to_ros_map_lifecycle_state(MapLifecycleState state);

std::optional<ExplorationCompletedEvent> to_core_exploration_completed_event(
  const robot_interfaces::msg::ExplorationState & message);

}  // 命名空间 map_lifecycle::adapters
