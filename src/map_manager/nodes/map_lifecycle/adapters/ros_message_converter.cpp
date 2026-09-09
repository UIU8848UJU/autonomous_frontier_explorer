#include "ros_message_converter.hpp"

namespace map_manager::adapters
{

MapSnapshot to_core_map(const nav_msgs::msg::OccupancyGrid & message)
{
  MapSnapshot map;
  map.width = message.info.width;
  map.height = message.info.height;
  map.resolution = message.info.resolution;
  map.data = message.data;
  return map;
}

ExplorationState to_core_exploration_state(
  const robot_interfaces::msg::ExplorationState & message)
{
  ExplorationState state;
  using RosExplorationState = robot_interfaces::msg::ExplorationState;
  switch (message.state) {
    case RosExplorationState::IDLE:
      state.phase = ExplorationPhase::IDLE;
      break;
    case RosExplorationState::RUNNING:
      state.phase = ExplorationPhase::RUNNING;
      break;
    case RosExplorationState::STOPPED:
      state.phase = ExplorationPhase::STOPPED;
      break;
    case RosExplorationState::COMPLETED:
      state.phase = ExplorationPhase::COMPLETED;
      break;
    case RosExplorationState::STUCK:
      state.phase = ExplorationPhase::STUCK;
      break;
    default:
      state.phase = ExplorationPhase::UNKNOWN;
      break;
  }
  state.detail = message.detail;
  return state;
}

}  // namespace map_manager::adapters
