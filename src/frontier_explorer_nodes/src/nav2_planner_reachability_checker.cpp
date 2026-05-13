#include "frontier_explorer_nodes/nodes/nav2_planner_reachability_checker.hpp"

#include <chrono>
#include <cmath>
#include <future>

#include "nav_msgs/msg/path.hpp"

namespace frontier_explorer
{
namespace
{
double pathLength(const nav_msgs::msg::Path & path)
{
    if (path.poses.size() < 2U) {
        return 0.0;
    }

    double length = 0.0;
    for (std::size_t index = 1U; index < path.poses.size(); ++index) {
        const auto & previous = path.poses[index - 1U].pose.position;
        const auto & current = path.poses[index].pose.position;
        length += std::hypot(current.x - previous.x, current.y - previous.y);
    }
    return length;
}
}  // namespace

Nav2PlannerReachabilityChecker::Nav2PlannerReachabilityChecker(
    rclcpp::Node * node,
    const FrontierExplorerParams & params)
: node_(node),
  logger_(node->get_logger().get_child("reachability")),
  params_(params),
  client_(rclcpp_action::create_client<ComputePathToPose>(
      node,
      params.runtime.compute_path_to_pose_action))
{
}

FrontierReachabilityResult Nav2PlannerReachabilityChecker::check(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    FrontierReachabilityResult result;
    result.checked = true;
    result.reachable = false;

    if (!client_->wait_for_action_server(params_.runtime.reachability_server_timeout)) {
        result.checked = false;
        result.reachable = true;
        result.reason = "planner_action_unavailable";
        RCLCPP_WARN_THROTTLE(
            logger_,
            *node_->get_clock(),
            3000,
            "ComputePathToPose action unavailable: %s",
            params_.runtime.compute_path_to_pose_action.c_str());
        return result;
    }

    ComputePathToPose::Goal planner_goal;
    planner_goal.start = start;
    planner_goal.goal = goal;
    planner_goal.use_start = true;
    planner_goal.planner_id = params_.runtime.reachability_planner_id;

    // 这一步是查看goal是否被接受
    auto goal_future = client_->async_send_goal(planner_goal);
    if (goal_future.wait_for(params_.runtime.reachability_check_timeout) !=
        std::future_status::ready)
    {
        result.reason = "planner_goal_timeout";
        return result;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        result.reason = "planner_goal_rejected";
        return result;
    }

    auto result_future = client_->async_get_result(goal_handle);
    if (result_future.wait_for(params_.runtime.reachability_check_timeout) !=
        std::future_status::ready)
    {
        client_->async_cancel_goal(goal_handle);
        result.reason = "planner_result_timeout";
        return result;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped_result.result ||
        wrapped_result.result->path.poses.empty())
    {
        result.reason = "planner_no_path";
        return result;
    }

    result.reachable = true;
    result.path_length_m = pathLength(wrapped_result.result->path);
    result.reason = "reachable";
    return result;
}

}  // namespace frontier_explorer
