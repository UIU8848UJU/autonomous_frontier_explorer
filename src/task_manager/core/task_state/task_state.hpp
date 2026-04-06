#pragma once

#include <string>

namespace task_manager
{

enum class TaskManagerState
{
    IDLE = 0,
    STARTING_EXPLORATION,
    EXPLORING,
    MAPPING_DONE,
    STARTING_NAVIGATION,
    NAVIGATING,
    PAUSED,
    FAILED
};

inline std::string to_string(TaskManagerState state)
{
    switch (state) {
        case TaskManagerState::IDLE: return "IDLE";
        case TaskManagerState::STARTING_EXPLORATION: return "STARTING_EXPLORATION";
        case TaskManagerState::EXPLORING: return "EXPLORING";
        case TaskManagerState::MAPPING_DONE: return "MAPPING_DONE";
        case TaskManagerState::STARTING_NAVIGATION: return "STARTING_NAVIGATION";
        case TaskManagerState::NAVIGATING: return "NAVIGATING";
        case TaskManagerState::PAUSED: return "PAUSED";
        case TaskManagerState::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

}  // namespace task_manager

