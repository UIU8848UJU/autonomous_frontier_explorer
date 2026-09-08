#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "task_state/task_state.hpp"

namespace task_manager
{

enum class ExplorationState : std::uint8_t
{
  IDLE = 0U,
  RUNNING = 1U,
  STOPPED = 2U,
  COMPLETED = 3U,
  STUCK = 4U,
  UNKNOWN = 255U
};

struct ExplorationEvent
{
  ExplorationState state{ExplorationState::IDLE};
  std::string detail;
};

struct TaskContext
{
  TaskManagerState state{TaskManagerState::IDLE};
  bool map_ready{false};
  bool exploration_running{false};
  std::chrono::steady_clock::time_point last_state_update;
  std::string last_exploration_state;
  std::string last_error;

  TaskContext()
  : last_state_update(std::chrono::steady_clock::now())
  {
  }
};

}  // namespace task_manager
