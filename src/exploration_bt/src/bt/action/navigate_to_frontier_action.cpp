#include "exploration_bt/bt/action/navigate_to_frontier_action.hpp"

#include <chrono>
#include <limits>
#include <mutex>

namespace exploration
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
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        accepted_ = false;
        rejected_ = false;
        result_ready_ = false;
        result_success_ = false;
        goal_handle_.reset();
    }

    std::optional<geometry_msgs::msg::PoseStamped> goal_pose;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        ++context_->feasibility_start_region_revision;
        context_->feasibility_cache.clear();
        goal_pose = context_->current_goal;
        context_->navigation_active = true;
        context_->navigation_finished = false;
        context_->navigation_result_success = false;
        context_->navigation_distance_remaining =
            std::numeric_limits<double>::infinity();
        context_->last_detail = "NAVIGATING";
    }

    if (!goal_pose.has_value()) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_active = false;
        context_->navigation_finished = true;
        context_->navigation_failed = true;
        context_->last_detail = "NO_FRONTIER_GOAL";
        return BT::NodeStatus::FAILURE;
    }

    if (!context_->nav_client->wait_for_action_server(std::chrono::milliseconds(100))) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_active = false;
        context_->navigation_finished = true;
        context_->navigation_failed = true;
        context_->last_detail = "NAVIGATION_SERVER_NOT_READY";
        return BT::NodeStatus::FAILURE;
    }

    send_goal(goal_pose.value());
    return BT::NodeStatus::RUNNING;
}

bool NavigateToFrontierAction::send_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        accepted_ = false;
        rejected_ = false;
        result_ready_ = false;
        result_success_ = false;
        goal_handle_.reset();
    }

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
        [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (!goal_handle) {
                rejected_ = true;
                return;
            }
            goal_handle_ = goal_handle;
            accepted_ = true;
        };
    options.feedback_callback =
        [this](
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            if (!feedback) {
                return;
            }
            std::lock_guard<std::mutex> lock(context_->mutex);
            context_->navigation_distance_remaining = feedback->distance_remaining;
        };
    options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult & result) {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            result_success_ =
                result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                result.result &&
                result.result->success;
            result_ready_ = true;
        };
    context_->nav_client->async_send_goal(goal, options);
    return true;
}

BT::NodeStatus NavigateToFrontierAction::onRunning()
{
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (context_->stop_requested) {
            return BT::NodeStatus::FAILURE;
        }
    }

    bool request_replacement = false;
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        if (context_->enable_active_goal_replacement &&
            context_->replacement_candidate.has_value() &&
            context_->goal_switch_count < context_->max_goal_switches_per_navigation &&
            context_->now() - context_->navigation_started_at >=
            rclcpp::Duration::from_seconds(context_->goal_min_hold_duration_sec))
        {
            context_->current_goal = context_->replacement_candidate->goal;
            context_->replacement_candidate.reset();
            ++context_->goal_switch_count;
            context_->last_detail = "ACTIVE_GOAL_REPLACEMENT_CANCELING";
            request_replacement = true;
        }
    }
    if (request_replacement) {
        replacement_cancel_requested_ = true;
        GoalHandleNavigateToPose::SharedPtr goal_handle;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            goal_handle = goal_handle_;
        }
        if (goal_handle) {
            context_->nav_client->async_cancel_goal(goal_handle);
        }
        return BT::NodeStatus::RUNNING;
    }

    if (replacement_cancel_requested_) {
        bool result_ready = false;
        bool rejected = false;
        bool result_success = false;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            result_ready = result_ready_;
            rejected = rejected_;
            result_success = result_success_;
        }
        if (!result_ready && !rejected) {
            return BT::NodeStatus::RUNNING;
        }
        replacement_cancel_requested_ = false;
        if (!result_success) {
            std::optional<geometry_msgs::msg::PoseStamped> replacement_goal;
            {
                std::lock_guard<std::mutex> lock(context_->mutex);
                replacement_goal = context_->current_goal;
                context_->navigation_active = true;
                context_->navigation_finished = false;
                context_->last_detail = "NAVIGATION_GOAL_REPLACED";
            }
            if (replacement_goal.has_value()) {
                send_goal(replacement_goal.value());
                return BT::NodeStatus::RUNNING;
            }
        }
        // 如果旧目标在取消前已成功到达，继续按普通成功路径收尾。
    }

    bool rejected = false;
    bool accepted = false;
    bool result_ready = false;
    bool result_success = false;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        rejected = rejected_;
        accepted = accepted_;
        result_ready = result_ready_;
        result_success = result_success_;
    }

    if (rejected) {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_active = false;
        context_->navigation_finished = true;
        context_->navigation_result_success = false;
        context_->navigation_distance_remaining =
            std::numeric_limits<double>::infinity();
        context_->navigation_failed = true;
        context_->last_detail = "NAVIGATION_GOAL_REJECTED";
        return BT::NodeStatus::FAILURE;
    }

    if (!accepted || !result_ready) {
        return BT::NodeStatus::RUNNING;
    }

    std::lock_guard<std::mutex> lock(context_->mutex);
    context_->navigation_active = false;
    context_->navigation_finished = true;
    // 导航已经改变机器人所在区域，上一轮起点生成的规划结果必须失效。
    context_->feasibility_cache.clear();
    context_->navigation_result_success = result_success;
    if (result_success) {
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
    {
        std::lock_guard<std::mutex> lock(context_->mutex);
        context_->navigation_active = false;
        context_->navigation_finished = true;
        context_->navigation_result_success = false;
        context_->replacement_candidate.reset();
    }
    replacement_cancel_requested_ = false;
    GoalHandleNavigateToPose::SharedPtr goal_handle;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        goal_handle = goal_handle_;
    }
    if (goal_handle) {
        context_->nav_client->async_cancel_goal(goal_handle);
    }
}

}  // 命名空间 exploration
