#include "ros_message_converter.hpp"

namespace task_manager::adapters
{

ExplorationEvent to_core_exploration_event(
  const robot_interfaces::msg::ExplorationState & message)
{
  ExplorationEvent event;
  using RosExplorationState = robot_interfaces::msg::ExplorationState;
  switch (message.state) {
    case RosExplorationState::IDLE:
      event.state = ExplorationState::IDLE;
      break;
    case RosExplorationState::RUNNING:
      event.state = ExplorationState::RUNNING;
      break;
    case RosExplorationState::STOPPED:
      event.state = ExplorationState::STOPPED;
      break;
    case RosExplorationState::COMPLETED:
      event.state = ExplorationState::COMPLETED;
      break;
    case RosExplorationState::STUCK:
      event.state = ExplorationState::STUCK;
      break;
    default:
      event.state = ExplorationState::UNKNOWN;
      break;
  }
  event.detail = message.detail;
  return event;
}

}  // namespace task_manager::adapters
