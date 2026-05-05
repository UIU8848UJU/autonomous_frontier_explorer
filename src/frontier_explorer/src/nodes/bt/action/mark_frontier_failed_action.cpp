#include "nodes/bt/action/mark_frontier_failed_action.hpp"

#include <mutex>
#include <optional>

namespace frontier_explorer
{
namespace
{
constexpr uint16_t kNavigationFailedReason = 100U;
}

MarkFrontierFailedAction::MarkFrontierFailedAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
: MarkFrontierFailedAction(name, config, get_exploration_bt_context(config))
{
}

MarkFrontierFailedAction::MarkFrontierFailedAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::NodeStatus MarkFrontierFailedAction::onStart()
{
    response_ready_ = false;
    request_sent_ = false;
    return onRunning();
}

BT::NodeStatus MarkFrontierFailedAction::onRunning()
{
    if (response_ready_) {
        response_ready_ = false;
        request_sent_ = false;
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (!last_response_->success) {
            context_->last_detail = "MARK_FRONTIER_FAILED_REJECTED";
            return BT::NodeStatus::FAILURE;
        }
        context_->current_goal.reset();
        context_->navigation_failed = false;
        context_->last_detail = last_response_->blacklisted ?
            "FRONTIER_BLACKLISTED" : "FRONTIER_FAILURE_RECORDED";
        return BT::NodeStatus::SUCCESS;
    }

    if (request_sent_) {
        return BT::NodeStatus::RUNNING;
    }

    std::optional<geometry_msgs::msg::PoseStamped> failed_goal;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        failed_goal = context_->current_goal;
        context_->last_detail = "MARKING_FRONTIER_FAILED";
    }

    if (!failed_goal.has_value()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "NO_FAILED_FRONTIER_TO_MARK";
        return BT::NodeStatus::FAILURE;
    }

    if (!context_->mark_failed_client->service_is_ready()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "WAITING_MARK_FAILED_SERVICE";
        return BT::NodeStatus::RUNNING;
    }

    auto request = std::make_shared<robot_interfaces::srv::MarkFrontierFailed::Request>();
    request->failed_goal = failed_goal->pose.position;
    request->failure_reason = kNavigationFailedReason;
    request->failure_text = "NavigateToPose failed in Exploration BT";

    request_sent_ = true;
    context_->mark_failed_client->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces::srv::MarkFrontierFailed>::SharedFuture future) {
            last_response_ = future.get();
            response_ready_ = true;
        });
    return BT::NodeStatus::RUNNING;
}

void MarkFrontierFailedAction::onHalted()
{
    request_sent_ = false;
}

}  // namespace frontier_explorer
