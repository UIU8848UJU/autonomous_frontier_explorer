#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "task_state/task_state.hpp"

namespace task_manager
{

struct TaskContext
{
    TaskManagerState state{TaskManagerState::IDLE};
    bool map_ready{false};
    bool exploration_running{false};
    bool navigation_running{false};
    std::chrono::steady_clock::time_point last_state_update;
    std::string last_exploration_state;
    std::string last_error;

    TaskContext()
    : last_state_update(std::chrono::steady_clock::now())
    {
    }
};

}  // namespace task_manager
