#pragma once

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace task_manager
{

enum class TaskState
{
  IDLE,
  NAVIGATING,
  PAUSED,
  SUCCEEDED,
  CANCELED,
  FAILED
};

inline std::string to_string(TaskState state)
{
  switch (state) {
    case TaskState::IDLE: return "IDLE";
    case TaskState::NAVIGATING: return "NAVIGATING";
    case TaskState::PAUSED: return "PAUSED";
    case TaskState::SUCCEEDED: return "SUCCEEDED";
    case TaskState::CANCELED: return "CANCELED";
    case TaskState::FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

struct TaskPoint
{
  std::string name;
  geometry_msgs::msg::PoseStamped pose;
};

}  // namespace task_manager