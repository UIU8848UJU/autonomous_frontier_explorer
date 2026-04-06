#pragma once

#include <string>

#include "robot_interfaces/msg/exploration_state.hpp"

#include "task_state/task_state.hpp"
#include "task_types/task_types.hpp"

namespace task_manager
{

class TaskFlow
{
public:
    TaskFlow();

    const TaskContext & context() const noexcept { return context_; }
    TaskManagerState state() const noexcept { return context_.state; }

    void set_state(TaskManagerState new_state);

    bool start_mapping_flow();
    bool mark_mapping_done();
    bool start_navigation_flow();
    bool stop_all();

    void update_exploration_state(const robot_interfaces::msg::ExplorationState & state_msg);
    void set_error(const std::string & error_text);

private:
    TaskContext context_;

    void touch_state_time() noexcept;
};

}  // namespace task_manager
