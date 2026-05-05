#include "nodes/bt/action/compute_frontier_candidates_action.hpp"

#include <algorithm>
#include <mutex>

namespace frontier_explorer
{

ComputeFrontierCandidatesAction::ComputeFrontierCandidatesAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
: ComputeFrontierCandidatesAction(name, config, get_exploration_bt_context(config))
{
}

ComputeFrontierCandidatesAction::ComputeFrontierCandidatesAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::NodeStatus ComputeFrontierCandidatesAction::onStart()
{
    response_ready_ = false;
    request_sent_ = false;
    next_request_time_ = context_->now();
    return onRunning();
}

BT::NodeStatus ComputeFrontierCandidatesAction::onRunning()
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
            std::vector<ExplorationBtContext::FrontierCandidate> candidates;
            const auto count = std::min({
                last_response_->goals.size(),
                last_response_->scores.size(),
                last_response_->distance_m.size(),
                last_response_->clearance_m.size(),
                last_response_->unknown_ratio.size(),
                last_response_->cluster_sizes.size(),
                last_response_->retry_counts.size()});
            candidates.reserve(count);
            for (std::size_t index = 0; index < count; ++index) {
                ExplorationBtContext::FrontierCandidate candidate;
                candidate.goal = last_response_->goals[index];
                candidate.score = last_response_->scores[index];
                candidate.distance_m = last_response_->distance_m[index];
                candidate.clearance_m = last_response_->clearance_m[index];
                candidate.unknown_ratio = last_response_->unknown_ratio[index];
                candidate.cluster_size = last_response_->cluster_sizes[index];
                candidate.retry_count = last_response_->retry_counts[index];
                if (index < last_response_->reachability_checked.size()) {
                    candidate.reachability_checked = last_response_->reachability_checked[index];
                }
                if (index < last_response_->reachable.size()) {
                    candidate.reachable = last_response_->reachable[index];
                }
                if (index < last_response_->path_length_m.size()) {
                    candidate.path_length_m = last_response_->path_length_m[index];
                }
                candidates.push_back(candidate);
            }

            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->frontier_candidates = candidates;
            context_->current_goal.reset();
            context_->exploration_complete = false;
            context_->navigation_failed = false;
            context_->last_detail = "FRONTIER_CANDIDATES_READY";
            return candidates.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }

        if (last_response_->exploration_complete) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->frontier_candidates.clear();
            context_->current_goal.reset();
            context_->exploration_complete = true;
            context_->last_detail = last_response_->reason_text.empty() ?
                "EXPLORATION_COMPLETE" : last_response_->reason_text;
            return BT::NodeStatus::FAILURE;
        }

        if (last_response_->recoverable) {
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->last_detail = last_response_->reason_text.empty() ?
                "FRONTIER_CANDIDATES_RECOVERABLE_WAIT" : last_response_->reason_text;
            next_request_time_ =
                context_->now() +
                rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
            return BT::NodeStatus::RUNNING;
        }

        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = last_response_->reason_text.empty() ?
            "FRONTIER_CANDIDATES_FAILED" : last_response_->reason_text;
        return BT::NodeStatus::FAILURE;
    }

    if (request_sent_) {
        return BT::NodeStatus::RUNNING;
    }
    if (context_->now() < next_request_time_) {
        return BT::NodeStatus::RUNNING;
    }
    if (!context_->get_candidates_client->service_is_ready()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "WAITING_FRONTIER_CANDIDATES_SERVICE";
        next_request_time_ =
            context_->now() +
            rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
        return BT::NodeStatus::RUNNING;
    }

    auto request = std::make_shared<robot_interfaces::srv::GetFrontierCandidates::Request>();
    request->max_candidates = context_->max_frontier_candidates;
    request_sent_ = true;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "REQUESTING_FRONTIER_CANDIDATES";
    }
    context_->get_candidates_client->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces::srv::GetFrontierCandidates>::SharedFuture future) {
            last_response_ = future.get();
            response_ready_ = true;
        });
    return BT::NodeStatus::RUNNING;
}

void ComputeFrontierCandidatesAction::onHalted()
{
    request_sent_ = false;
}

}  // namespace frontier_explorer
