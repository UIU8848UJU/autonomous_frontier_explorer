#pragma once

#include "robot_interfaces/msg/exploration_state.hpp"

#include "task_types/task_types.hpp"

namespace task_manager::adapters
{

ExplorationEvent to_core_exploration_event(
  const robot_interfaces::msg::ExplorationState & message);

}  // namespace task_manager::adapters
