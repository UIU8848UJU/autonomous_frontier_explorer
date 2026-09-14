#include "exploration_bt/bt/action/prefetch_frontier_candidates_action.hpp"

#include <algorithm>
#include <utility>
#include <cmath>

#include "exploration_bt/bt/frontier_candidate_conversion.hpp"

namespace exploration
{

PrefetchFrontierCandidatesAction::PrefetchFrontierCandidatesAction(
    const std::string & name,
    const BT::NodeConfiguration & config)
: PrefetchFrontierCandidatesAction(name, config, get_exploration_bt_context(config))
{
}

PrefetchFrontierCandidatesAction::PrefetchFrontierCandidatesAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::StatefulActionNode(name, config),
  context_(context),
  request_state_(std::make_shared<RequestState>())
{
}

BT::NodeStatus PrefetchFrontierCandidatesAction::onStart()
{
    response_consumed_ = false;
    prefetch_finished_ = false;
    prefetch_success_ = false;
    request_state_ = std::make_shared<RequestState>();

    if (!context_->enable_candidate_prefetch) {
        // 禁用预取时仍保持 RUNNING，直到同一 Parallel 中的导航结束。
        return BT::NodeStatus::RUNNING;
    }

    if (!context_->get_candidates_client->service_is_ready()) {
        return BT::NodeStatus::RUNNING;
    }

    auto request = std::make_shared<robot_interfaces::srv::GetFrontierCandidates::Request>();
    request->max_candidates = context_->max_frontier_candidates;
    {
        std::lock_guard<std::mutex> lock(request_state_->mutex);
        request_state_->sent = true;
    }
    const auto state = request_state_;
    context_->get_candidates_client->async_send_request(
        request,
        [state](rclcpp::Client<robot_interfaces::srv::GetFrontierCandidates>::SharedFuture future) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->response = future.get();
            state->ready = true;
        });
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "PREFETCHING_FRONTIER_CANDIDATES";
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PrefetchFrontierCandidatesAction::onRunning()
{
    if (!response_consumed_) {
        robot_interfaces::srv::GetFrontierCandidates::Response::SharedPtr response;
        {
            std::lock_guard<std::mutex> lock(request_state_->mutex);
            if (request_state_->ready) {
                response = request_state_->response;
                request_state_->ready = false;
            }
        }

        if (response) {
            response_consumed_ = true;
            prefetch_finished_ = true;
            prefetch_success_ = response->success || response->exploration_complete;
            if (prefetch_success_) {
                std::lock_guard<std::mutex> lock(context_->mutex);
                if (response->map_revision >= context_->latest_map_revision) {
                    auto candidates = convert_frontier_candidates(*response);
                    if (context_->enable_active_goal_replacement &&
                        context_->navigation_active &&
                        context_->goal_switch_count <
                        context_->max_goal_switches_per_navigation &&
                        !context_->current_goal.has_value()) {
                        // 当前目标在极短竞态窗口内刚刚结束时不再安排切换。
                        candidates.clear();
                    }
                    context_->prefetched_candidates = candidates;
                    context_->prefetched_map_revision = response->map_revision;
                    context_->prefetched_generated_at = context_->now();
                    context_->prefetched_exploration_complete = response->exploration_complete;
                    context_->prefetch_ready = true;

                    if (context_->enable_active_goal_replacement &&
                        context_->navigation_active &&
                        context_->current_goal.has_value() &&
                        context_->goal_switch_count <
                        context_->max_goal_switches_per_navigation &&
                        context_->now() - context_->navigation_started_at >=
                        rclcpp::Duration::from_seconds(context_->goal_min_hold_duration_sec) &&
                        std::isfinite(context_->navigation_distance_remaining) &&
                        context_->navigation_distance_remaining >
                        context_->active_goal_reached_tolerance_m)
                    {
                        const auto current_goal = context_->current_goal->pose.position;
                        const auto current_it = std::find_if(
                            context_->frontier_candidates.begin(),
                            context_->frontier_candidates.end(),
                            [&current_goal](const ExplorationBtContext::FrontierCandidate & candidate) {
                                return std::hypot(
                                    candidate.goal.pose.position.x - current_goal.x,
                                    candidate.goal.pose.position.y - current_goal.y) < 1e-3;
                            });
                        if (current_it != context_->frontier_candidates.end()) {
                            const auto current_path = current_it->path_length_m > 0.0F ?
                                current_it->path_length_m : current_it->distance_m;
                            const auto current_utility = static_cast<double>(current_it->score) -
                                context_->feasible_path_length_weight * current_path;
                            const auto replacement_it = std::find_if(
                                candidates.begin(),
                                candidates.end(),
                                [&current_goal, context = context_, current_utility](
                                    const ExplorationBtContext::FrontierCandidate & candidate) {
                                    if (!candidate.reachability_checked || !candidate.reachable ||
                                        std::hypot(
                                            candidate.goal.pose.position.x - current_goal.x,
                                            candidate.goal.pose.position.y - current_goal.y) < 1e-3)
                                    {
                                        return false;
                                    }
                                    const auto path = candidate.path_length_m > 0.0F ?
                                        candidate.path_length_m : candidate.distance_m;
                                    const auto utility = static_cast<double>(candidate.score) -
                                        context->feasible_path_length_weight * path;
                                    return utility - current_utility >=
                                        context->goal_switch_min_utility_gain;
                                });
                            if (replacement_it != candidates.end()) {
                                context_->replacement_candidate = *replacement_it;
                                context_->last_detail = "ACTIVE_GOAL_REPLACEMENT_READY";
                            }
                        }
                    }
                }
            }
            RCLCPP_DEBUG(
                context_->logger,
                "Frontier prefetch finished: success=%s candidates=%zu map_revision=%llu",
                prefetch_success_ ? "true" : "false",
                response->goals.size(),
                static_cast<unsigned long long>(response->map_revision));
        }
    }

    std::lock_guard<std::mutex> lock(context_->mutex);
    if (!context_->navigation_finished) {
        return BT::NodeStatus::RUNNING;
    }
    // 预取失败只回退到下一轮同步计算，不能把当前导航结果判成失败。
    return BT::NodeStatus::SUCCESS;
}

void PrefetchFrontierCandidatesAction::onHalted()
{
    // 回调只捕获 RequestState，不捕获 this，树销毁后不会访问悬空对象。
    if (request_state_) {
        std::lock_guard<std::mutex> lock(request_state_->mutex);
        request_state_->sent = false;
    }
}

}  // 命名空间 exploration
