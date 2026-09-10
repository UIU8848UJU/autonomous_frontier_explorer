#pragma once

#include <cstdint>
#include <string>

namespace task_manager
{

enum class TaskManagerState : std::uint8_t
{
  IDLE = 0,
  STARTING_MAPPING,
  MAPPING,
  WAITING_MAP_SAVE,
  MAPPING_DONE,
  FAILED
};

inline std::string to_string(TaskManagerState state)
{
  switch (state) {
    case TaskManagerState::IDLE: return "IDLE";
    case TaskManagerState::STARTING_MAPPING: return "STARTING_MAPPING";
    case TaskManagerState::MAPPING: return "MAPPING";
    case TaskManagerState::WAITING_MAP_SAVE: return "WAITING_MAP_SAVE";
    case TaskManagerState::MAPPING_DONE: return "MAPPING_DONE";
    case TaskManagerState::FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

}  // 命名空间 task_manager
