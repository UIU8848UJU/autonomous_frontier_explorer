#include "nodes/bt/action/compute_next_frontier_goal_action.hpp"

#include <mutex>

namespace frontier_explorer
{

ComputeNextFrontierGoalAction::ComputeNextFrontierGoalAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::NodeStatus ComputeNextFrontierGoalAction::onStart()
{
    response_ready_ = false;
    request_sent_ = false;
    next_request_time_ = context_->now();
    return onRunning();
}

BT::NodeStatus ComputeNextFrontierGoalAction::onRunning()
{
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (context_->stop_requested) {
            return BT::NodeStatus::FAILURE;
        }
    }

    if (response_ready_) {
        response_ready_ = false;
        request_sent_ = false;

        if (last_response_->success) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->current_goal = last_response_->goal;
            context_->exploration_complete = false;
            context_->navigation_failed = false;
            context_->last_detail = "FRONTIER_SELECTED";
            return BT::NodeStatus::SUCCESS;
        }

        if (last_response_->exploration_complete) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->exploration_complete = true;
            context_->last_detail = last_response_->reason_text.empty() ?
                "EXPLORATION_COMPLETE" : last_response_->reason_text;
            return BT::NodeStatus::FAILURE;
        }

        if (last_response_->recoverable) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->last_detail = last_response_->reason_text.empty() ?
                "FRONTIER_RECOVERABLE_WAIT" : last_response_->reason_text;
            next_request_time_ =
                context_->now() +
                rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
            return BT::NodeStatus::RUNNING;
        }

        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = last_response_->reason_text.empty() ?
            "FRONTIER_REQUEST_FAILED" : last_response_->reason_text;
        return BT::NodeStatus::FAILURE;
    }

    if (request_sent_) {
        return BT::NodeStatus::RUNNING;
    }

    if (context_->now() < next_request_time_) {
        return BT::NodeStatus::RUNNING;
    }

    if (!context_->get_next_client->service_is_ready()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "WAITING_FRONTIER_SERVICE";
        next_request_time_ =
            context_->now() +
            rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
        return BT::NodeStatus::RUNNING;
    }

    auto request = std::make_shared<robot_interfaces::srv::GetNextFrontierGoal::Request>();
    request_sent_ = true;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "REQUESTING_FRONTIER";
    }
    context_->get_next_client->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces::srv::GetNextFrontierGoal>::SharedFuture future) {
            last_response_ = future.get();
            response_ready_ = true;
        });

    return BT::NodeStatus::RUNNING;
}

void ComputeNextFrontierGoalAction::onHalted()
{
    request_sent_ = false;
}

}  // namespace frontier_explorer
