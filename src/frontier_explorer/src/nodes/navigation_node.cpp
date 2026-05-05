#include "nodes/navigation_node.hpp"

#include <cmath>
#include <chrono>
#include <algorithm>
#include <functional>
#include <future>
#include <limits>
#include <thread>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"

namespace frontier_explorer
{
namespace
{
constexpr uint16_t kResultSucceeded = 0U;
constexpr uint16_t kResultRejected = 1U;
constexpr uint16_t kResultCanceled = 2U;
constexpr uint16_t kResultAborted = 3U;
constexpr uint16_t kResultNav2Unavailable = 4U;
constexpr uint16_t kResultException = 5U;
constexpr uint16_t kResultFootprintCollision = 6U;
constexpr uint16_t kResultPathUnsafe = 7U;

double path_length_m(const nav_msgs::msg::Path & path)
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

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

bool point_in_polygon(
    double x,
    double y,
    const std::vector<geometry_msgs::msg::Point> & polygon)
{
    if (polygon.size() < 3U) {
        return false;
    }

    bool inside = false;
    for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
        const auto & pi = polygon[i];
        const auto & pj = polygon[j];
        const bool intersects =
            ((pi.y > y) != (pj.y > y)) &&
            (x < (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1e-12) + pi.x);
        if (intersects) {
            inside = !inside;
        }
    }
    return inside;
}
}

NavigationNode::NavigationNode(const rclcpp::NodeOptions & options)
: Node("navigation_node", options),
  logger_(get_logger().get_child("navigation"))
{
    declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    declare_parameter<std::string>("compute_path_to_pose_action", "compute_path_to_pose");
    declare_parameter<std::string>("navigation_action", "~/navigate_to_pose");
    declare_parameter<std::string>("check_pose_reachability_service", "~/check_pose_reachability");
    declare_parameter<std::string>("check_goal_feasibility_service", "~/check_goal_feasibility");
    declare_parameter<std::string>("reachability_planner_id", "");
    declare_parameter<std::string>("footprint_costmap_topic", "/global_costmap/costmap");
    declare_parameter<bool>("enable_footprint_collision_check", true);
    declare_parameter<bool>("allow_unknown_footprint", false);
    declare_parameter<bool>("enable_path_safety_check", true);
    declare_parameter<bool>("allow_unknown_path", false);
    declare_parameter<double>("robot_radius", 0.1);
    declare_parameter<double>("footprint_padding", 0.0);
    declare_parameter<int>(
        "footprint_cost_threshold",
        static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
    declare_parameter<int>(
        "path_cost_threshold",
        static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
    declare_parameter<int>("nav2_server_timeout_ms", 500);
    declare_parameter<int>("reachability_timeout_ms", 500);

    nav2_action_name_ = get_parameter("navigate_to_pose_action").as_string();
    compute_path_action_name_ = get_parameter("compute_path_to_pose_action").as_string();
    default_planner_id_ = get_parameter("reachability_planner_id").as_string();
    footprint_costmap_topic_ = get_parameter("footprint_costmap_topic").as_string();
    enable_footprint_collision_check_ =
        get_parameter("enable_footprint_collision_check").as_bool();
    allow_unknown_footprint_ = get_parameter("allow_unknown_footprint").as_bool();
    enable_path_safety_check_ =
        get_parameter("enable_path_safety_check").as_bool();
    allow_unknown_path_ = get_parameter("allow_unknown_path").as_bool();
    robot_radius_ = std::max(0.01, get_parameter("robot_radius").as_double());
    footprint_padding_ = std::max(0.0, get_parameter("footprint_padding").as_double());
    footprint_cost_threshold_ = static_cast<unsigned char>(
        std::clamp(
            static_cast<int>(get_parameter("footprint_cost_threshold").as_int()),
            1,
            255));
    path_cost_threshold_ = static_cast<unsigned char>(
        std::clamp(
            static_cast<int>(get_parameter("path_cost_threshold").as_int()),
            1,
            255));
    nav2_server_timeout_ = std::chrono::milliseconds(
        std::max(10, static_cast<int>(get_parameter("nav2_server_timeout_ms").as_int())));
    reachability_timeout_ = std::chrono::milliseconds(
        std::max(50, static_cast<int>(get_parameter("reachability_timeout_ms").as_int())));

    navigation_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    nav2_client_ = rclcpp_action::create_client<Nav2NavigateToPose>(
        this,
        nav2_action_name_,
        navigation_callback_group_);
    compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
        this,
        compute_path_action_name_,
        navigation_callback_group_);
    robot_footprint_ = nav2_costmap_2d::makeFootprintFromRadius(robot_radius_);
    if (footprint_padding_ > 0.0) {
        nav2_costmap_2d::padFootprint(robot_footprint_, footprint_padding_);
    }

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        get_parameter("navigation_action").as_string(),
        std::bind(
            &NavigationNode::handle_goal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &NavigationNode::handle_cancel,
            this,
            std::placeholders::_1),
        std::bind(
            &NavigationNode::handle_accepted,
            this,
            std::placeholders::_1),
        rcl_action_server_get_default_options(),
        navigation_callback_group_);
    reachability_srv_ = create_service<robot_interfaces::srv::CheckPoseReachability>(
        get_parameter("check_pose_reachability_service").as_string(),
        std::bind(
            &NavigationNode::handle_check_pose_reachability,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        navigation_callback_group_);
    feasibility_srv_ = create_service<robot_interfaces::srv::CheckGoalFeasibility>(
        get_parameter("check_goal_feasibility_service").as_string(),
        std::bind(
            &NavigationNode::handle_check_goal_feasibility,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default,
        navigation_callback_group_);
    if ((enable_footprint_collision_check_ || enable_path_safety_check_) &&
        !footprint_costmap_topic_.empty())
    {
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = navigation_callback_group_;
        footprint_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            footprint_costmap_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(
                &NavigationNode::footprint_costmap_callback,
                this,
                std::placeholders::_1),
            subscription_options);
    }

    RCLCPP_INFO(
        logger_,
        "NavigationNode started: action=%s nav2_action=%s reachability_service=%s feasibility_service=%s planner_action=%s footprint_check=%s path_check=%s",
        get_parameter("navigation_action").as_string().c_str(),
        nav2_action_name_.c_str(),
        get_parameter("check_pose_reachability_service").as_string().c_str(),
        get_parameter("check_goal_feasibility_service").as_string().c_str(),
        compute_path_action_name_.c_str(),
        enable_footprint_collision_check_ ? "true" : "false",
        enable_path_safety_check_ ? "true" : "false");
}

rclcpp_action::GoalResponse NavigationNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    if (!goal) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->pose.header.frame_id.empty()) {
        RCLCPP_WARN(logger_, "Rejecting navigation goal with empty frame_id.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    {
        std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
        if (navigation_goal_active_) {
            RCLCPP_WARN(
                logger_,
                "Rejecting navigation goal while another exploration goal is active.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        navigation_goal_active_ = true;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationNode::handle_cancel(
    const std::shared_ptr<NavigateGoalHandle>)
{
    std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
    if (active_nav2_goal_) {
        nav2_client_->async_cancel_goal(active_nav2_goal_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationNode::handle_accepted(
    const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
    std::thread{
        std::bind(&NavigationNode::execute_navigation, this, goal_handle)
    }.detach();
}

void NavigationNode::execute_navigation(
    const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
    auto result = std::make_shared<NavigateToPose::Result>();

    try {
        if (!nav2_client_->wait_for_action_server(nav2_server_timeout_)) {
            result->success = false;
            result->result_code = kResultNav2Unavailable;
            result->message = "Nav2 NavigateToPose action server unavailable";
            goal_handle->abort(result);
            std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
            navigation_goal_active_ = false;
            return;
        }

        Nav2NavigateToPose::Goal nav2_goal;
        nav2_goal.pose = goal_handle->get_goal()->pose;
        nav2_goal.behavior_tree = goal_handle->get_goal()->behavior_tree;

        auto send_options = rclcpp_action::Client<Nav2NavigateToPose>::SendGoalOptions();
        send_options.feedback_callback =
            [goal_handle](
                Nav2GoalHandle::SharedPtr,
                const std::shared_ptr<const Nav2NavigateToPose::Feedback> feedback) {
                if (!feedback) {
                    return;
                }
                auto out_feedback = std::make_shared<NavigateToPose::Feedback>();
                out_feedback->current_pose = feedback->current_pose;
                out_feedback->navigation_time = feedback->navigation_time;
                out_feedback->estimated_time_remaining = feedback->estimated_time_remaining;
                out_feedback->number_of_recoveries = feedback->number_of_recoveries;
                out_feedback->distance_remaining = feedback->distance_remaining;
                out_feedback->phase = "NAVIGATING";
                goal_handle->publish_feedback(out_feedback);
            };

        auto nav2_goal_future = nav2_client_->async_send_goal(nav2_goal, send_options);
        if (nav2_goal_future.wait_for(nav2_server_timeout_) != std::future_status::ready) {
            result->success = false;
            result->result_code = kResultNav2Unavailable;
            result->message = "Nav2 NavigateToPose goal response timeout";
            goal_handle->abort(result);
            std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
            navigation_goal_active_ = false;
            return;
        }

        auto nav2_goal_handle = nav2_goal_future.get();
        if (!nav2_goal_handle) {
            result->success = false;
            result->result_code = kResultRejected;
            result->message = "Nav2 NavigateToPose goal rejected";
            goal_handle->abort(result);
            std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
            navigation_goal_active_ = false;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
            active_nav2_goal_ = nav2_goal_handle;
        }

        auto nav2_result_future = nav2_client_->async_get_result(nav2_goal_handle);
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                nav2_client_->async_cancel_goal(nav2_goal_handle);
                result->success = false;
                result->result_code = kResultCanceled;
                result->message = "Navigation canceled";
                goal_handle->canceled(result);
                std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
                active_nav2_goal_.reset();
                navigation_goal_active_ = false;
                return;
            }
            if (nav2_result_future.wait_for(std::chrono::milliseconds(100)) ==
                std::future_status::ready)
            {
                break;
            }
        }

        const auto wrapped_result = nav2_result_future.get();
        {
            std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
            active_nav2_goal_.reset();
            navigation_goal_active_ = false;
        }

        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            result->success = true;
            result->result_code = kResultSucceeded;
            result->message = "Navigation succeeded";
            goal_handle->succeed(result);
            return;
        }
        if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
            result->success = false;
            result->result_code = kResultCanceled;
            result->message = "Navigation canceled by Nav2";
            goal_handle->canceled(result);
            return;
        }

        result->success = false;
        result->result_code = kResultAborted;
        result->message = "Navigation aborted by Nav2";
        goal_handle->abort(result);
    } catch (const std::exception & ex) {
        result->success = false;
        result->result_code = kResultException;
        result->message = ex.what();
        if (goal_handle->is_active()) {
            goal_handle->abort(result);
        }
        std::lock_guard<std::mutex> lock(nav2_goal_mutex_);
        active_nav2_goal_.reset();
        navigation_goal_active_ = false;
    }
}

void NavigationNode::handle_check_pose_reachability(
    const std::shared_ptr<robot_interfaces::srv::CheckPoseReachability::Request> request,
    std::shared_ptr<robot_interfaces::srv::CheckPoseReachability::Response> response)
{
    if (!request) {
        response->success = false;
        response->reachable = false;
        response->recoverable = false;
        response->result_code = kResultException;
        response->message = "empty request";
        return;
    }

    if (request->goal.header.frame_id.empty()) {
        response->success = false;
        response->reachable = false;
        response->recoverable = false;
        response->result_code = kResultRejected;
        response->message = "goal frame_id is empty";
        return;
    }

    compute_path_reachability(*request, *response, nullptr);
}

void NavigationNode::compute_path_reachability(
    const robot_interfaces::srv::CheckPoseReachability::Request & request,
    robot_interfaces::srv::CheckPoseReachability::Response & response,
    nav_msgs::msg::Path * path)
{
    if (!compute_path_client_->wait_for_action_server(nav2_server_timeout_)) {
        response.success = false;
        response.reachable = false;
        response.recoverable = true;
        response.result_code = kResultNav2Unavailable;
        response.message = "ComputePathToPose action server unavailable";
        return;
    }

    ComputePathToPose::Goal goal;
    goal.goal = request.goal;
    goal.start = request.start;
    goal.use_start = request.use_start;
    goal.planner_id = request.planner_id.empty() ? default_planner_id_ : request.planner_id;

    auto goal_future = compute_path_client_->async_send_goal(goal);
    if (goal_future.wait_for(reachability_timeout_) != std::future_status::ready) {
        response.success = false;
        response.reachable = false;
        response.recoverable = true;
        response.result_code = kResultNav2Unavailable;
        response.message = "ComputePathToPose goal response timeout";
        return;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        response.success = true;
        response.reachable = false;
        response.recoverable = false;
        response.result_code = kResultRejected;
        response.message = "ComputePathToPose goal rejected";
        return;
    }

    auto result_future = compute_path_client_->async_get_result(goal_handle);
    if (result_future.wait_for(reachability_timeout_) != std::future_status::ready) {
        compute_path_client_->async_cancel_goal(goal_handle);
        response.success = false;
        response.reachable = false;
        response.recoverable = true;
        response.result_code = kResultNav2Unavailable;
        response.message = "ComputePathToPose result timeout";
        return;
    }

    const auto wrapped_result = result_future.get();
    response.success = true;
    response.reachable =
        wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result &&
        !wrapped_result.result->path.poses.empty();
    response.recoverable = false;
    response.result_code = response.reachable ? kResultSucceeded : kResultAborted;
    response.message = response.reachable ? "reachable" : "no path";
    if (wrapped_result.result) {
        response.pose_count = static_cast<uint32_t>(wrapped_result.result->path.poses.size());
        response.path_length_m = static_cast<float>(path_length_m(wrapped_result.result->path));
        if (path != nullptr) {
            *path = wrapped_result.result->path;
        }
    }
}

void NavigationNode::handle_check_goal_feasibility(
    const std::shared_ptr<robot_interfaces::srv::CheckGoalFeasibility::Request> request,
    std::shared_ptr<robot_interfaces::srv::CheckGoalFeasibility::Response> response)
{
    if (!request) {
        response->success = false;
        response->feasible = false;
        response->reachable = false;
        response->footprint_valid = false;
        response->recoverable = false;
        response->result_code = kResultException;
        response->message = "empty request";
        return;
    }

    std::string footprint_reason;
    double footprint_cost = 0.0;
    const bool footprint_valid =
        is_goal_footprint_valid(request->goal, footprint_reason, footprint_cost);
    response->footprint_valid = footprint_valid;
    response->footprint_reason = footprint_reason;
    response->footprint_cost = static_cast<float>(footprint_cost);
    if (!footprint_valid) {
        response->success = footprint_reason != "footprint_costmap_unavailable";
        response->feasible = false;
        response->reachable = false;
        response->recoverable = footprint_reason == "footprint_costmap_unavailable";
        response->result_code = response->recoverable ?
            kResultNav2Unavailable : kResultFootprintCollision;
        response->message = footprint_reason;
        return;
    }

    auto reachability_request =
        std::make_shared<robot_interfaces::srv::CheckPoseReachability::Request>();
    reachability_request->start = request->start;
    reachability_request->goal = request->goal;
    reachability_request->use_start = request->use_start;
    reachability_request->planner_id = request->planner_id;
    auto reachability_response =
        std::make_shared<robot_interfaces::srv::CheckPoseReachability::Response>();
    nav_msgs::msg::Path planned_path;
    compute_path_reachability(*reachability_request, *reachability_response, &planned_path);

    response->success = reachability_response->success;
    response->reachable = reachability_response->reachable;
    response->recoverable = reachability_response->recoverable;
    response->result_code = reachability_response->result_code;
    response->message = reachability_response->message;
    response->path_length_m = reachability_response->path_length_m;
    response->pose_count = reachability_response->pose_count;
    response->feasible =
        response->success &&
        response->reachable &&
        response->footprint_valid;
    if (response->feasible) {
        std::string path_reason;
        double max_path_cost = 0.0;
        if (!is_path_costmap_safe(planned_path, path_reason, max_path_cost)) {
            response->success = path_reason != "path_costmap_unavailable";
            response->feasible = false;
            response->recoverable = path_reason == "path_costmap_unavailable";
            response->result_code = response->recoverable ?
                kResultNav2Unavailable : kResultPathUnsafe;
            response->message = path_reason;
            RCLCPP_WARN(
                logger_,
                "Rejected feasible goal because planned path is unsafe: reason=%s max_cost=%.1f poses=%u",
                path_reason.c_str(),
                max_path_cost,
                response->pose_count);
            return;
        }
    }
    if (response->feasible) {
        response->message = "feasible";
    }
}

void NavigationNode::footprint_costmap_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg || msg->info.width == 0U || msg->info.height == 0U || msg->info.resolution <= 0.0F) {
        RCLCPP_WARN(logger_, "Received invalid footprint costmap.");
        return;
    }

    const auto expected_size =
        static_cast<std::size_t>(msg->info.width) * static_cast<std::size_t>(msg->info.height);
    if (msg->data.size() != expected_size) {
        RCLCPP_WARN(
            logger_,
            "Footprint costmap size mismatch: expected=%zu actual=%zu",
            expected_size,
            msg->data.size());
        return;
    }

    auto costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
        msg->info.width,
        msg->info.height,
        static_cast<double>(msg->info.resolution),
        msg->info.origin.position.x,
        msg->info.origin.position.y,
        nav2_costmap_2d::NO_INFORMATION);

    for (unsigned int y = 0U; y < msg->info.height; ++y) {
        for (unsigned int x = 0U; x < msg->info.width; ++x) {
            const auto index = static_cast<std::size_t>(y) * msg->info.width + x;
            costmap->setCost(x, y, interpret_occupancy_value(msg->data[index]));
        }
    }

    std::lock_guard<std::mutex> lock(footprint_costmap_mutex_);
    footprint_costmap_ = std::move(costmap);
}

bool NavigationNode::is_goal_footprint_valid(
    const geometry_msgs::msg::PoseStamped & goal,
    std::string & reason,
    double & footprint_cost) const
{
    footprint_cost = 0.0;
    if (!enable_footprint_collision_check_) {
        reason = "footprint_check_disabled";
        return true;
    }

    std::lock_guard<std::mutex> lock(footprint_costmap_mutex_);
    if (!footprint_costmap_) {
        reason = "footprint_costmap_unavailable";
        return false;
    }

    const double theta = yaw_from_quaternion(goal.pose.orientation);
    const double cos_th = std::cos(theta);
    const double sin_th = std::sin(theta);
    std::vector<geometry_msgs::msg::Point> oriented;
    oriented.reserve(robot_footprint_.size());
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto & point : robot_footprint_) {
        geometry_msgs::msg::Point transformed;
        transformed.x = goal.pose.position.x + point.x * cos_th - point.y * sin_th;
        transformed.y = goal.pose.position.y + point.x * sin_th + point.y * cos_th;
        oriented.push_back(transformed);
        min_x = std::min(min_x, transformed.x);
        min_y = std::min(min_y, transformed.y);
        max_x = std::max(max_x, transformed.x);
        max_y = std::max(max_y, transformed.y);
    }

    unsigned int min_mx = 0U;
    unsigned int min_my = 0U;
    unsigned int max_mx = 0U;
    unsigned int max_my = 0U;
    if (!footprint_costmap_->worldToMap(min_x, min_y, min_mx, min_my) ||
        !footprint_costmap_->worldToMap(max_x, max_y, max_mx, max_my))
    {
        reason = "footprint_out_of_costmap";
        return false;
    }

    const auto start_x = std::min(min_mx, max_mx);
    const auto end_x = std::max(min_mx, max_mx);
    const auto start_y = std::min(min_my, max_my);
    const auto end_y = std::max(min_my, max_my);
    bool sampled = false;
    for (unsigned int my = start_y; my <= end_y; ++my) {
        for (unsigned int mx = start_x; mx <= end_x; ++mx) {
            double wx = 0.0;
            double wy = 0.0;
            footprint_costmap_->mapToWorld(mx, my, wx, wy);
            if (!point_in_polygon(wx, wy, oriented)) {
                continue;
            }
            sampled = true;
            const auto cost = footprint_costmap_->getCost(mx, my);
            footprint_cost = std::max(footprint_cost, static_cast<double>(cost));
            if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_footprint_) {
                reason = "footprint_over_unknown";
                return false;
            }
            if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= footprint_cost_threshold_) {
                reason = "footprint_collision";
                return false;
            }
        }
    }

    if (!sampled) {
        reason = "footprint_not_sampled";
        return false;
    }

    reason = "footprint_valid";
    return true;
}

bool NavigationNode::is_path_costmap_safe(
    const nav_msgs::msg::Path & path,
    std::string & reason,
    double & max_path_cost) const
{
    max_path_cost = 0.0;
    if (!enable_path_safety_check_) {
        reason = "path_check_disabled";
        return true;
    }
    if (path.poses.empty()) {
        reason = "path_empty";
        return false;
    }

    std::lock_guard<std::mutex> lock(footprint_costmap_mutex_);
    if (!footprint_costmap_) {
        reason = "path_costmap_unavailable";
        return false;
    }

    for (const auto & pose : path.poses) {
        unsigned int mx = 0U;
        unsigned int my = 0U;
        if (!footprint_costmap_->worldToMap(
                pose.pose.position.x,
                pose.pose.position.y,
                mx,
                my))
        {
            reason = "path_out_of_costmap";
            return false;
        }

        const auto cost = footprint_costmap_->getCost(mx, my);
        max_path_cost = std::max(max_path_cost, static_cast<double>(cost));
        if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_path_) {
            reason = "path_crosses_unknown";
            return false;
        }
        if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= path_cost_threshold_) {
            reason = "path_crosses_high_cost";
            return false;
        }
    }

    reason = "path_safe";
    return true;
}

unsigned char NavigationNode::interpret_occupancy_value(int8_t occupancy) const
{
    if (occupancy < 0) {
        return nav2_costmap_2d::NO_INFORMATION;
    }
    if (occupancy == 0) {
        return nav2_costmap_2d::FREE_SPACE;
    }
    if (occupancy > 50) {
        return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    return static_cast<unsigned char>(
        std::clamp(
            static_cast<int>(
                std::round(
                    static_cast<double>(occupancy) / 50.0 *
                    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1))),
            1,
            static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));
}

}  // namespace frontier_explorer
