#include "nodes/bt/action/select_feasible_frontier_action.hpp"

#include <cmath>
#include <mutex>

namespace frontier_explorer
{

SelectFeasibleFrontierAction::SelectFeasibleFrontierAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
: SelectFeasibleFrontierAction(name, config, get_exploration_bt_context(config))
{
}

SelectFeasibleFrontierAction::SelectFeasibleFrontierAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::NodeStatus SelectFeasibleFrontierAction::onStart()
{
    request_sent_ = false;
    response_ready_ = false;
    current_index_ = 0U;
    best_feasible_index_.reset();
    saw_recoverable_failure_ = false;
    next_request_time_ = context_->now();
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        candidates_ = context_->frontier_candidates;
        recoverable_retry_counts_.assign(candidates_.size(), 0U);
        context_->current_goal.reset();
        context_->last_detail = "SELECTING_FEASIBLE_FRONTIER";
    }
    return onRunning();
}

BT::NodeStatus SelectFeasibleFrontierAction::onRunning()
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
        candidate.reachable = last_response_->reachable;
        candidate.feasible = last_response_->success && last_response_->feasible;
        candidate.footprint_valid = last_response_->footprint_valid;
        candidate.path_length_m = last_response_->path_length_m;
        candidate.footprint_cost = last_response_->footprint_cost;
        candidate.feasibility_detail = last_response_->message;

        if (candidate.feasible) {
            if (!best_feasible_index_.has_value() ||
                is_better_feasible_candidate(candidate, candidates_[best_feasible_index_.value()]))
            {
                best_feasible_index_ = current_index_;
            }
            ++current_index_;
            return onRunning();
        }

        if (!last_response_->success && last_response_->recoverable) {
            saw_recoverable_failure_ = true;
            const auto retry_count = ++recoverable_retry_counts_[current_index_];
            if (retry_count > context_->max_feasibility_recoverable_retries) {
                RCLCPP_WARN(
                    context_->logger,
                    "Skipping frontier candidate after recoverable feasibility retries: index=%zu retries=%u reason=%s",
                    current_index_,
                    retry_count,
                    candidate.feasibility_detail.c_str());
                ++current_index_;
                return onRunning();
            }

            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->last_detail = "FEASIBILITY_RECOVERABLE_WAIT";
            next_request_time_ =
                context_->now() +
                rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
            return BT::NodeStatus::RUNNING;
        }

        ++current_index_;
    }

    if (current_index_ >= candidates_.size()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (best_feasible_index_.has_value()) {
            const auto & selected = candidates_[best_feasible_index_.value()];
            context_->current_goal = selected.goal;
            context_->navigation_failed = false;
            context_->frontier_candidates = candidates_;
            context_->last_detail = "FEASIBLE_FRONTIER_SELECTED";
            RCLCPP_INFO(
                context_->logger,
                "Selected feasible frontier: index=%zu score=%.3f distance=%.3f path=%.3f detail=%s",
                best_feasible_index_.value(),
                selected.score,
                selected.distance_m,
                selected.path_length_m,
                selected.feasibility_detail.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        if (saw_recoverable_failure_) {
            current_index_ = 0U;
            saw_recoverable_failure_ = false;
            recoverable_retry_counts_.assign(candidates_.size(), 0U);
            context_->last_detail = "NO_FEASIBLE_FRONTIER_RECOVERABLE_WAIT";
            next_request_time_ =
                context_->now() +
                rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
            return BT::NodeStatus::RUNNING;
        }
        context_->current_goal = candidates_.front().goal;
        context_->frontier_candidates = candidates_;
        context_->navigation_failed = true;
        context_->last_detail = "NO_FEASIBLE_FRONTIER";
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

bool SelectFeasibleFrontierAction::is_better_feasible_candidate(
    const ExplorationBtContext::FrontierCandidate & candidate,
    const ExplorationBtContext::FrontierCandidate & best_candidate) const
{
    const auto candidate_path_length =
        candidate.path_length_m > 0.0F ? candidate.path_length_m : candidate.distance_m;
    const auto best_path_length =
        best_candidate.path_length_m > 0.0F ?
        best_candidate.path_length_m : best_candidate.distance_m;
    const auto candidate_utility =
        static_cast<double>(candidate.score) -
        context_->feasible_path_length_weight * static_cast<double>(candidate_path_length);
    const auto best_utility =
        static_cast<double>(best_candidate.score) -
        context_->feasible_path_length_weight * static_cast<double>(best_path_length);

    if (std::fabs(candidate_utility - best_utility) > 1e-6) {
        return candidate_utility > best_utility;
    }
    if (std::fabs(candidate_path_length - best_path_length) > 1e-6F) {
        return candidate_path_length < best_path_length;
    }
    return candidate.score > best_candidate.score;
}

void SelectFeasibleFrontierAction::onHalted()
{
    request_sent_ = false;
}

bool SelectFeasibleFrontierAction::send_current_request()
{
    if (!context_->feasibility_client->service_is_ready()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "WAITING_FEASIBILITY_SERVICE";
        next_request_time_ =
            context_->now() +
            rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
        return true;
    }

    const auto & candidate = candidates_[current_index_];
    auto request = std::make_shared<robot_interfaces::srv::CheckGoalFeasibility::Request>();
    request->goal = candidate.goal;
    request->use_start = false;
    request_sent_ = true;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "CHECKING_FRONTIER_FEASIBILITY";
    }
    context_->feasibility_client->async_send_request(
        request,
        [this](rclcpp::Client<robot_interfaces::srv::CheckGoalFeasibility>::SharedFuture future) {
            last_response_ = future.get();
            response_ready_ = true;
        });
    return true;
}

}  // namespace frontier_explorer
