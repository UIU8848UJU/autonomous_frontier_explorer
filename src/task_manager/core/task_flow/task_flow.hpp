#pragma once

#include <string>

#include "task_state/task_state.hpp"
#include "task_types/task_types.hpp"

namespace task_manager
{

class TaskFlow
{
public:
  TaskFlow();

  const TaskContext & context() const noexcept {return context_;}
  TaskManagerState state() const noexcept {return context_.state;}

  void set_state(TaskManagerState new_state);

  bool start_mapping_flow();
  bool mark_map_saved();
  bool stop_all();

  void update_exploration_state(const ExplorationEvent & event);
  void set_error(const std::string & error_text);

private:
  TaskContext context_;

  void touch_state_time() noexcept;
};

}  // namespace task_manager
