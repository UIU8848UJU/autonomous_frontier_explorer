#include "exploration_bt/bt/action/compute_frontier_candidates_action.hpp"

#include <mutex>

namespace exploration
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

    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (context_->prefetch_ready) {
            const auto age_sec =
                (context_->now() - context_->prefetched_generated_at).seconds();
            const bool fresh_enough = age_sec >= 0.0 &&
                age_sec <= context_->prefetch_max_age_sec &&
                context_->prefetched_map_revision >= context_->latest_map_revision;
            if (fresh_enough) {
                if (context_->prefetched_map_revision != context_->latest_map_revision) {
                    // 地图版本变化后，旧版本的 planner 结论不能复用。
                    context_->feasibility_cache.clear();
                }
                context_->frontier_candidates = std::move(context_->prefetched_candidates);
                context_->latest_map_revision = context_->prefetched_map_revision;
                context_->exploration_complete = context_->prefetched_exploration_complete;
                context_->navigation_failed = false;
                context_->last_detail = context_->exploration_complete ?
                    "PREFETCHED_EXPLORATION_COMPLETE" : "PREFETCHED_FRONTIER_CANDIDATES_READY";
                context_->prefetch_ready = false;
                return context_->exploration_complete || context_->frontier_candidates.empty() ?
                    BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
            }
            // 旧预取不能直接执行；清掉后回退到普通同步请求。
            context_->prefetched_candidates.clear();
            context_->prefetch_ready = false;
            context_->prefetched_exploration_complete = false;
        }
    }
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
            const auto candidates = last_response_->candidates;

            std::lock_guard<std::mutex> lock(context_->mutex);
            if (last_response_->map_revision != context_->latest_map_revision) {
                context_->feasibility_cache.clear();
            }
            context_->frontier_candidates = candidates;
            context_->latest_map_revision = last_response_->map_revision;
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

}  // 命名空间 exploration
