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
    context_.navigation_running = false;
    context_.map_ready = false;
    context_.last_error.clear();

    set_state(TaskManagerState::STARTING_EXPLORATION);
    set_state(TaskManagerState::EXPLORING);
    return true;
}

bool TaskFlow::mark_mapping_done()
{
    if (!context_.exploration_running &&
        context_.state != TaskManagerState::EXPLORING)
    {
        return false;
    }

    context_.exploration_running = false;
    context_.map_ready = true;
    set_state(TaskManagerState::MAPPING_DONE);
    return true;
}

bool TaskFlow::start_navigation_flow()
{
    if (!context_.map_ready) {
        return false;
    }

    if (context_.state == TaskManagerState::NAVIGATING ||
        context_.state == TaskManagerState::STARTING_NAVIGATION)
    {
        return false;
    }

    context_.navigation_running = true;
    context_.exploration_running = false;
    context_.last_error.clear();

    set_state(TaskManagerState::STARTING_NAVIGATION);
    set_state(TaskManagerState::NAVIGATING);
    return true;
}

bool TaskFlow::stop_all()
{
    const bool was_active =
        context_.exploration_running || context_.navigation_running ||
        context_.state == TaskManagerState::NAVIGATING ||
        context_.state == TaskManagerState::EXPLORING;

    context_.exploration_running = false;
    context_.navigation_running = false;
    set_state(TaskManagerState::IDLE);
    return was_active;
}

namespace
{
std::string exploration_state_to_string(uint8_t state)
{
    using robot_interfaces::msg::ExplorationState;
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

void TaskFlow::update_exploration_state(const robot_interfaces::msg::ExplorationState & state_msg)
{
    context_.last_exploration_state = exploration_state_to_string(state_msg.state);
    if (!state_msg.detail.empty()) {
        context_.last_exploration_state += " - " + state_msg.detail;
    }

    switch (state_msg.state) {
        case robot_interfaces::msg::ExplorationState::RUNNING:
            context_.exploration_running = true;
            context_.navigation_running = false;
            context_.last_error.clear();
            set_state(TaskManagerState::EXPLORING);
            break;
        case robot_interfaces::msg::ExplorationState::COMPLETED:
            context_.exploration_running = false;
            context_.map_ready = true;
            set_state(TaskManagerState::MAPPING_DONE);
            break;
        case robot_interfaces::msg::ExplorationState::STOPPED:
        case robot_interfaces::msg::ExplorationState::IDLE:
            context_.exploration_running = false;
            context_.last_error.clear();
            set_state(TaskManagerState::IDLE);
            break;
        case robot_interfaces::msg::ExplorationState::STUCK:
            context_.exploration_running = false;
            context_.last_error = state_msg.detail.empty() ?
                "Exploration reported STUCK state." : state_msg.detail;
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
