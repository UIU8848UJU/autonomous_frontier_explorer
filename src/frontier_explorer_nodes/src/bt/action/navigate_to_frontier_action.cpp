#include "frontier_explorer_nodes/nodes/bt/action/navigate_to_frontier_action.hpp"

#include <chrono>
#include <mutex>

namespace frontier_explorer
{

NavigateToFrontierAction::NavigateToFrontierAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
: NavigateToFrontierAction(name, config, get_exploration_bt_context(config))
{
}

NavigateToFrontierAction::NavigateToFrontierAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::NodeStatus NavigateToFrontierAction::onStart()
{
    accepted_ = false;
    rejected_ = false;
    result_ready_ = false;
    result_success_ = false;

    std::optional<geometry_msgs::msg::PoseStamped> goal_pose;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        goal_pose = context_->current_goal;
        context_->last_detail = "NAVIGATING";
    }

    if (!goal_pose.has_value()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_failed = true;
        context_->last_detail = "NO_FRONTIER_GOAL";
        return BT::NodeStatus::FAILURE;
    }

    if (!context_->nav_client->wait_for_action_server(std::chrono::milliseconds(100))) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_failed = true;
        context_->last_detail = "NAVIGATION_SERVER_NOT_READY";
        return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal;
    goal.pose = goal_pose.value();

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
        [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
            if (!goal_handle) {
                rejected_ = true;
                return;
            }
            goal_handle_ = goal_handle;
            accepted_ = true;
        };
    options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult & result) {
            result_success_ =
                result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                result.result &&
                result.result->success;
            result_ready_ = true;
        };

    context_->nav_client->async_send_goal(goal, options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToFrontierAction::onRunning()
{
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (context_->stop_requested) {
            return BT::NodeStatus::FAILURE;
        }
    }

    if (rejected_) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_failed = true;
        context_->last_detail = "NAVIGATION_GOAL_REJECTED";
        return BT::NodeStatus::FAILURE;
    }

    if (!accepted_ || !result_ready_) {
        return BT::NodeStatus::RUNNING;
    }

    std::lock_guard<std::mutex> lock(context_->mutex);
    if (result_success_) {
        context_->current_goal.reset();
        context_->navigation_failed = false;
        context_->last_detail = "NAVIGATION_SUCCEEDED";
        return BT::NodeStatus::SUCCESS;
    }

    context_->navigation_failed = true;
    context_->last_detail = "NAVIGATION_FAILED";
    return BT::NodeStatus::FAILURE;
}

void NavigateToFrontierAction::onHalted()
{
    if (goal_handle_) {
        context_->nav_client->async_cancel_goal(goal_handle_);
    }
}

}  // namespace frontier_explorer
