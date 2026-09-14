#include "exploration_bt/bt/action/select_feasible_frontier_action.hpp"

#include <cmath>
#include <mutex>
#include <sstream>

#include "exploration_bt/bt/feasibility_batch.hpp"
#include "exploration_bt/bt/feasibility_cache_key.hpp"

namespace exploration
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
        batch_end_ = initial_feasibility_batch_end(
            candidates_.size(),
            context_->feasibility_top_k);
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

        {
            std::lock_guard<std::mutex> lock(context_->mutex);
            if (last_response_->costmap_revision != context_->latest_costmap_revision) {
                context_->feasibility_cache.clear();
                context_->latest_costmap_revision = last_response_->costmap_revision;
            }
            if (context_->feasibility_cache_enabled &&
                request_cache_key_.size() > 0U &&
                request_map_revision_ == context_->latest_map_revision &&
                request_start_region_revision_ == context_->feasibility_start_region_revision)
            {
                ExplorationBtContext::CachedFeasibilityResult cached;
                cached.success = last_response_->success;
                cached.feasible = last_response_->feasible;
                cached.reachable = last_response_->reachable;
                cached.footprint_valid = last_response_->footprint_valid;
                cached.recoverable = last_response_->recoverable;
                cached.result_code = last_response_->result_code;
                cached.message = last_response_->message;
                cached.path_length_m = last_response_->path_length_m;
                cached.footprint_cost = last_response_->footprint_cost;
                cached.created_at = context_->now();
                context_->feasibility_cache[make_feasibility_cache_key(
                    context_->latest_map_revision,
                    context_->latest_costmap_revision,
                    context_->feasibility_start_region_revision,
                    candidate.goal,
                    context_->feasibility_planner_id,
                    context_->feasibility_cache_region_size_m)] = std::move(cached);
            }
        }

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

    if (current_index_ >= batch_end_) {
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
        if (batch_end_ < candidates_.size()) {
            // 当前批次没有可行点时才扩大检查范围，避免每轮固定规划全部候选。
            batch_end_ = expand_feasibility_batch_end(
                batch_end_,
                candidates_.size(),
                context_->feasibility_top_k);
            saw_recoverable_failure_ = false;
            context_->last_detail = "CHECKING_NEXT_FEASIBILITY_BATCH";
            return BT::NodeStatus::RUNNING;
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
    const auto & candidate = candidates_[current_index_];
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        request_cache_key_ = make_feasibility_cache_key(
            context_->latest_map_revision,
            context_->latest_costmap_revision,
            context_->feasibility_start_region_revision,
            candidate.goal,
            context_->feasibility_planner_id,
            context_->feasibility_cache_region_size_m);
        request_map_revision_ = context_->latest_map_revision;
        request_start_region_revision_ = context_->feasibility_start_region_revision;
        if (context_->feasibility_cache_enabled) {
            const auto cache_it = context_->feasibility_cache.find(request_cache_key_);
            if (cache_it != context_->feasibility_cache.end()) {
                const auto age_sec = (context_->now() - cache_it->second.created_at).seconds();
                if (age_sec >= 0.0 && age_sec <= context_->feasibility_cache_ttl_sec) {
                    last_response_ = std::make_shared<robot_interfaces::srv::CheckGoalFeasibility::Response>();
                    last_response_->success = cache_it->second.success;
                    last_response_->feasible = cache_it->second.feasible;
                    last_response_->reachable = cache_it->second.reachable;
                    last_response_->footprint_valid = cache_it->second.footprint_valid;
                    last_response_->recoverable = cache_it->second.recoverable;
                    last_response_->result_code = cache_it->second.result_code;
                    last_response_->message = cache_it->second.message;
                    last_response_->path_length_m = cache_it->second.path_length_m;
                    last_response_->footprint_cost = cache_it->second.footprint_cost;
                    last_response_->costmap_revision = context_->latest_costmap_revision;
                    response_ready_ = true;
                    context_->last_detail = "FEASIBILITY_CACHE_HIT";
                    return true;
                }
                context_->feasibility_cache.erase(cache_it);
            }
        }
    }

    if (!context_->feasibility_client->service_is_ready()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->last_detail = "WAITING_FEASIBILITY_SERVICE";
        next_request_time_ =
            context_->now() +
            rclcpp::Duration::from_seconds(context_->service_retry_delay_sec);
        return true;
    }

    auto request = std::make_shared<robot_interfaces::srv::CheckGoalFeasibility::Request>();
    request->goal = candidate.goal;
    request->use_start = false;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        request->planner_id = context_->feasibility_planner_id;
    }
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

}  // 命名空间 exploration
