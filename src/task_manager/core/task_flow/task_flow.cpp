#include "task_flow/task_flow.hpp"

#include <chrono>
#include <string>

namespace task_manager
{

TaskFlow::TaskFlow() = default;

void TaskFlow::touch_state_time() noexcept
{
  context_.last_state_update = std::chrono::steady_clock::now();
}

void TaskFlow::set_state(TaskManagerState new_state)
{
  if (context_.state == new_state) {
    return;
  }

  context_.state = new_state;
  touch_state_time();
}

bool TaskFlow::start_mapping_flow()
{
  if (context_.state == TaskManagerState::STARTING_EXPLORATION ||
    context_.state == TaskManagerState::EXPLORING)
  {
    return false;
  }

  context_.exploration_running = true;
  context_.map_ready = false;
  context_.last_error.clear();

  set_state(TaskManagerState::STARTING_EXPLORATION);
  set_state(TaskManagerState::EXPLORING);
  return true;
}

bool TaskFlow::mark_map_saved()
{
  context_.exploration_running = false;
  context_.map_ready = true;
  context_.last_error.clear();
  set_state(TaskManagerState::MAPPING_DONE);
  return true;
}

bool TaskFlow::stop_all()
{
  const bool was_active =
    context_.exploration_running ||
    context_.state == TaskManagerState::NAVIGATING ||
    context_.state == TaskManagerState::EXPLORING;

  context_.exploration_running = false;
  set_state(TaskManagerState::IDLE);
  return was_active;
}

namespace
{
std::string exploration_state_to_string(ExplorationState state)
{
  switch (state) {
    case ExplorationState::IDLE: return "IDLE";
    case ExplorationState::RUNNING: return "RUNNING";
    case ExplorationState::STOPPED: return "STOPPED";
    case ExplorationState::COMPLETED: return "COMPLETED";
    case ExplorationState::STUCK: return "STUCK";
    default: return "UNKNOWN";
  }
}
}  // namespace

void TaskFlow::update_exploration_state(const ExplorationEvent & event)
{
  context_.last_exploration_state = exploration_state_to_string(event.state);
  if (!event.detail.empty()) {
    context_.last_exploration_state += " - " + event.detail;
  }

  switch (event.state) {
    case ExplorationState::RUNNING:
      context_.exploration_running = true;
      context_.last_error.clear();
      set_state(TaskManagerState::EXPLORING);
      break;
    case ExplorationState::COMPLETED:
      context_.exploration_running = false;
      context_.map_ready = true;
      set_state(TaskManagerState::MAPPING_DONE);
      break;
    case ExplorationState::STOPPED:
    case ExplorationState::IDLE:
      context_.exploration_running = false;
      context_.last_error.clear();
      set_state(TaskManagerState::IDLE);
      break;
    case ExplorationState::STUCK:
      context_.exploration_running = false;
      context_.last_error = event.detail.empty() ?
        "Exploration reported STUCK state." : event.detail;
      set_state(TaskManagerState::FAILED);
      break;
    default:
      break;
  }

  touch_state_time();
}

void TaskFlow::set_error(const std::string & error_text)
{
  context_.last_error = error_text;
  touch_state_time();
}

}  // namespace task_manager
