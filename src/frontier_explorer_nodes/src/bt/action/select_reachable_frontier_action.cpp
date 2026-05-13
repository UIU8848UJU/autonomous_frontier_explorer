#include "frontier_explorer_nodes/nodes/bt/action/select_reachable_frontier_action.hpp"

#include <mutex>

namespace frontier_explorer
{

SelectReachableFrontierAction::SelectReachableFrontierAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
: SelectReachableFrontierAction(name, config, get_exploration_bt_context(config))
{
}

SelectReachableFrontierAction::SelectReachableFrontierAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::NodeStatus SelectReachableFrontierAction::onStart()
{
    request_sent_ = false;
    response_ready_ = false;
    current_index_ = 0U;
    next_request_time_ = context_->now();
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        candidates_ = context_->frontier_candidates;
        context_->current_goal.reset();
        context_->last_detail = "SELECTING_REACHABLE_FRONTIER";
    }
    return onRunning();
}

BT::NodeStatus SelectReachableFrontierAction::onRunning()
{
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (context_->stop_requested) {
            return BT::NodeStatus::FAILURE;
        }
    }

    if (candidates_.empty()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "NO_FRONTIER_CANDIDATES";
        return BT::NodeStatus::FAILURE;
    }

    if (response_ready_) {
        response_ready_ = false;
        request_sent_ = false;

        auto & candidate = candidates_[current_index_];
        candidate.reachability_checked = true;
        candidate.reachable = last_response_->success && last_response_->reachable;
        candidate.path_length_m = last_response_->path_length_m;

        if (candidate.reachable) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->current_goal = candidate.goal;
            context_->navigation_failed = false;
            context_->frontier_candidates = candidates_;
            context_->last_detail = "REACHABLE_FRONTIER_SELECTED";
            return BT::NodeStatus::SUCCESS;
        }

        if (!last_response_->success && last_response_->recoverable) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->last_detail = "REACHABILITY_RECOVERABLE_WAIT";
            next_request_time_ =
                context_->now() +
                rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
            return BT::NodeStatus::RUNNING;
        }

        ++current_index_;
    }

    if (current_index_ >= candidates_.size()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->current_goal = candidates_.front().goal;
        context_->frontier_candidates = candidates_;
        context_->navigation_failed = true;
        context_->last_detail = "NO_REACHABLE_FRONTIER";
        return BT::NodeStatus::FAILURE;
    }

    if (request_sent_) {
        return BT::NodeStatus::RUNNING;
    }
    if (context_->now() < next_request_time_) {
        return BT::NodeStatus::RUNNING;
    }

    return send_current_request() ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

void SelectReachableFrontierAction::onHalted()
{
    request_sent_ = false;
}

bool SelectReachableFrontierAction::send_current_request()
{
    if (!context_->reachability_client->service_is_ready()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "WAITING_REACHABILITY_SERVICE";
        next_request_time_ =
            context_->now() +
            rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
        return true;
    }

    const auto & candidate = candidates_[current_index_];
    auto request = std::make_shared<robot_interfaces::srv::CheckPoseReachability::Request>();
    request->goal = candidate.goal;
    request->use_start = false;
    request_sent_ = true;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "CHECKING_FRONTIER_REACHABILITY";
    }
    context_->reachability_client->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces::srv::CheckPoseReachability>::SharedFuture future) {
            last_response_ = future.get();
            response_ready_ = true;
        });
    return true;
}

}  // namespace frontier_explorer
