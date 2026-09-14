#include "ros_message_converter.hpp"

namespace map_lifecycle::adapters
{

grid_map_core::GridMap to_core_map(const nav_msgs::msg::OccupancyGrid & message)
{
  grid_map_core::GridMap map;
  map.width = message.info.width;
  map.height = message.info.height;
  map.resolution = message.info.resolution;
  map.origin_x = message.info.origin.position.x;
  map.origin_y = message.info.origin.position.y;
  map.data = message.data;
  return map;
}

std::uint8_t to_ros_map_lifecycle_state(MapLifecycleState state)
{
  using RosMapLifecycleState = robot_interfaces::msg::MapLifecycleState;
  switch (state) {
    case MapLifecycleState::EMPTY:
      return RosMapLifecycleState::EMPTY;
    case MapLifecycleState::ACTIVE:
      return RosMapLifecycleState::ACTIVE;
    case MapLifecycleState::READY:
      return RosMapLifecycleState::READY;
    case MapLifecycleState::SAVING:
      return RosMapLifecycleState::SAVING;
    case MapLifecycleState::SAVED:
      return RosMapLifecycleState::SAVED;
    case MapLifecycleState::FAILED:
      return RosMapLifecycleState::FAILED;
    default:
      return 255U;
  }
}

std::optional<ExplorationCompletedEvent> to_core_exploration_completed_event(
  const robot_interfaces::msg::ExplorationState & message)
{
  using RosExplorationState = robot_interfaces::msg::ExplorationState;
  if (message.state != RosExplorationState::COMPLETED) {
    return std::nullopt;
  }

  ExplorationCompletedEvent event;
  event.detail = message.detail;
  return event;
}

}  // 命名空间 map_lifecycle::adapters
