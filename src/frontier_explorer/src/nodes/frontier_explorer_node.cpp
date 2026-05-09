#include "nodes/frontier_explorer_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "navigation/nav2_planner_reachability_checker.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace frontier_explorer
{
namespace
{
constexpr size_t kStatePublisherDepth = 10;
}  // namespace

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: Node("frontier_explorer_node", options),
  goal_provider_(this->get_logger())
{
    declare_params();
    load_params();
    apply_params();
    if (params_.runtime.enable_reachability_filter) {
        goal_provider_.set_reachability_checker(
            std::make_shared<Nav2PlannerReachabilityChecker>(this, params_));
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    create_interfaces();
    marker_publisher_ = std::make_unique<FrontierMarkerPublisher>(this, this->get_logger(), "map");

    state_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.runtime.explore_period_sec),
        std::bind(&FrontierExplorerNode::state_timer_callback, this));

    set_state(ExplorationState::IDLE);
    publish_state();

    RCLCPP_INFO(this->get_logger(), "FrontierExplorerNode capability wrapper started.");
}

void FrontierExplorerNode::declare_params()
{
    this->declare_parameter<double>("explore_period_sec", 3.0);
    this->declare_parameter<int>("obstacle_search_radius_cells", 1);
    this->declare_parameter<int>("min_frontier_cluster_size", 1);
    this->declare_parameter<double>("min_goal_distance_m", 0.45);
    this->declare_parameter<int>("max_retry_count", 2);
    this->declare_parameter<int>(
        "frontier_decision.max_cluster_retry_count",
        params_.selection.max_cluster_retry_count);
    this->declare_parameter<bool>(
        "frontier_decision.defer_small_clusters",
        params_.selection.defer_small_clusters);
    this->declare_parameter<int>(
        "frontier_decision.small_cluster_size_threshold",
        static_cast<int>(params_.selection.small_cluster_size_threshold));
    this->declare_parameter<int>(
        "frontier_decision.candidate_unknown_margin_cells",
        params_.pruner.candidate_unknown_margin_cells);
    this->declare_parameter<int>(
        "frontier_decision.candidate_goal_inset_cells",
        params_.pruner.candidate_goal_inset_cells);
    this->declare_parameter<double>(
        "frontier_decision.candidate_max_unknown_ratio",
        params_.pruner.candidate_max_unknown_ratio);
    this->declare_parameter<int>(
        "map_stale_timeout_ms", static_cast<int>(params_.runtime.map_stale_timeout.count()));
    this->declare_parameter<int>(
        "max_frontier_failures", params_.runtime.max_frontier_failures);
    this->declare_parameter<double>("edge_tolerance_m", params_.runtime.edge_tolerance_m);
    this->declare_parameter<double>(
        "goal_reached_tolerance_m",
        params_.runtime.goal_reached_tolerance_m);
    this->declare_parameter<std::string>("map_topic", params_.runtime.map_topic);
    this->declare_parameter<std::string>(
        "global_costmap_topic",
        params_.runtime.global_costmap_topic);
    this->declare_parameter<bool>(
        "use_global_costmap_for_safety",
        params_.runtime.use_global_costmap_for_safety);
    this->declare_parameter<std::string>("global_frame", params_.runtime.global_frame);
    this->declare_parameter<std::string>("robot_base_frame", params_.runtime.robot_base_frame);
    this->declare_parameter<int>(
        "robot_pose_timeout_ms",
        static_cast<int>(params_.runtime.robot_pose_timeout.count()));
    this->declare_parameter<bool>(
        "show_all_candidate_markers",
        params_.runtime.show_all_candidate_markers);
    this->declare_parameter<bool>(
        "frontier_decision.enable_reachability_filter",
        params_.runtime.enable_reachability_filter);
    this->declare_parameter<bool>(
        "frontier_decision.require_reachable_goal",
        params_.runtime.require_reachable_goal);
    this->declare_parameter<int>(
        "frontier_decision.max_reachability_checks",
        params_.runtime.max_reachability_checks);
    this->declare_parameter<std::string>(
        "frontier_decision.compute_path_to_pose_action",
        params_.runtime.compute_path_to_pose_action);
    this->declare_parameter<int>(
        "frontier_decision.reachability_server_timeout_ms",
        static_cast<int>(params_.runtime.reachability_server_timeout.count()));
    this->declare_parameter<int>(
        "frontier_decision.reachability_check_timeout_ms",
        static_cast<int>(params_.runtime.reachability_check_timeout.count()));
    this->declare_parameter<std::string>(
        "frontier_decision.reachability_planner_id",
        params_.runtime.reachability_planner_id);
    this->declare_parameter<bool>("enable_internal_navigation_loop", false);
}

void FrontierExplorerNode::load_params()
{
    params_.runtime.explore_period_sec =
        this->get_parameter("explore_period_sec").as_double();
    params_.runtime.obstacle_search_radius_cells =
        this->get_parameter("obstacle_search_radius_cells").as_int();
    params_.runtime.min_frontier_cluster_size =
        this->get_parameter("min_frontier_cluster_size").as_int();
    params_.pruner.min_goal_distance_m =
        this->get_parameter("min_goal_distance_m").as_double();
    params_.selection.max_retry_count =
        this->get_parameter("max_retry_count").as_int();
    params_.selection.max_cluster_retry_count =
        this->get_parameter("frontier_decision.max_cluster_retry_count").as_int();
    params_.selection.defer_small_clusters =
        this->get_parameter("frontier_decision.defer_small_clusters").as_bool();
    params_.selection.small_cluster_size_threshold = static_cast<std::size_t>(
        this->get_parameter("frontier_decision.small_cluster_size_threshold").as_int());
    params_.pruner.candidate_unknown_margin_cells =
        this->get_parameter("frontier_decision.candidate_unknown_margin_cells").as_int();
    params_.pruner.candidate_goal_inset_cells =
        this->get_parameter("frontier_decision.candidate_goal_inset_cells").as_int();
    params_.pruner.candidate_max_unknown_ratio =
        this->get_parameter("frontier_decision.candidate_max_unknown_ratio").as_double();
    params_.runtime.map_stale_timeout =
        std::chrono::milliseconds(this->get_parameter("map_stale_timeout_ms").as_int());
    params_.runtime.max_frontier_failures =
        this->get_parameter("max_frontier_failures").as_int();
    params_.runtime.edge_tolerance_m =
        this->get_parameter("edge_tolerance_m").as_double();
    params_.runtime.goal_reached_tolerance_m =
        this->get_parameter("goal_reached_tolerance_m").as_double();
    params_.runtime.map_topic =
        this->get_parameter("map_topic").as_string();
    params_.runtime.global_costmap_topic =
        this->get_parameter("global_costmap_topic").as_string();
    params_.runtime.use_global_costmap_for_safety =
        this->get_parameter("use_global_costmap_for_safety").as_bool();
    params_.runtime.global_frame =
        this->get_parameter("global_frame").as_string();
    params_.runtime.robot_base_frame =
        this->get_parameter("robot_base_frame").as_string();
    params_.runtime.robot_pose_timeout =
        std::chrono::milliseconds(this->get_parameter("robot_pose_timeout_ms").as_int());
    params_.runtime.show_all_candidate_markers =
        this->get_parameter("show_all_candidate_markers").as_bool();
    params_.runtime.enable_reachability_filter =
        this->get_parameter("frontier_decision.enable_reachability_filter").as_bool();
    params_.runtime.require_reachable_goal =
        this->get_parameter("frontier_decision.require_reachable_goal").as_bool();
    params_.runtime.max_reachability_checks =
        static_cast<int>(this->get_parameter(
            "frontier_decision.max_reachability_checks").as_int());
    params_.runtime.compute_path_to_pose_action =
        this->get_parameter("frontier_decision.compute_path_to_pose_action").as_string();
    params_.runtime.reachability_server_timeout =
        std::chrono::milliseconds(this->get_parameter(
            "frontier_decision.reachability_server_timeout_ms").as_int());
    params_.runtime.reachability_check_timeout =
        std::chrono::milliseconds(this->get_parameter(
            "frontier_decision.reachability_check_timeout_ms").as_int());
    params_.runtime.reachability_planner_id =
        this->get_parameter("frontier_decision.reachability_planner_id").as_string();

    if (this->get_parameter("enable_internal_navigation_loop").as_bool()) {
        RCLCPP_WARN(
            this->get_logger(),
            "enable_internal_navigation_loop is deprecated and ignored. Use exploration BT orchestrator.");
    }
}

void FrontierExplorerNode::apply_params()
{
    params_.runtime.explore_period_sec =
        std::max(0.1, params_.runtime.explore_period_sec);
    params_.runtime.obstacle_search_radius_cells =
        std::max(0, params_.runtime.obstacle_search_radius_cells);
    params_.runtime.min_frontier_cluster_size =
        std::max(1, params_.runtime.min_frontier_cluster_size);
    params_.runtime.map_stale_timeout =
        std::chrono::milliseconds(std::max<int64_t>(1000, params_.runtime.map_stale_timeout.count()));
    params_.runtime.max_frontier_failures =
        std::max(1, params_.runtime.max_frontier_failures);
    params_.runtime.edge_tolerance_m =
        std::max(0.05, params_.runtime.edge_tolerance_m);
    params_.runtime.goal_reached_tolerance_m =
        std::max(0.01, params_.runtime.goal_reached_tolerance_m);
    params_.runtime.robot_pose_timeout =
        std::chrono::milliseconds(std::max<int64_t>(10, params_.runtime.robot_pose_timeout.count()));
    if (params_.runtime.global_frame.empty()) {
        params_.runtime.global_frame = "map";
    }
    if (params_.runtime.robot_base_frame.empty()) {
        params_.runtime.robot_base_frame = "base_link";
    }
    params_.runtime.max_reachability_checks =
        std::max(1, params_.runtime.max_reachability_checks);
    params_.pruner.candidate_goal_inset_cells =
        std::max(0, params_.pruner.candidate_goal_inset_cells);
    params_.runtime.reachability_server_timeout =
        std::chrono::milliseconds(std::max<int64_t>(
            10,
            params_.runtime.reachability_server_timeout.count()));
    params_.runtime.reachability_check_timeout =
        std::chrono::milliseconds(std::max<int64_t>(
            50,
            params_.runtime.reachability_check_timeout.count()));
    if (params_.runtime.compute_path_to_pose_action.empty()) {
        params_.runtime.compute_path_to_pose_action = "compute_path_to_pose";
    }

    params_.selection.max_retry_count =
        std::max(1, params_.selection.max_retry_count);
    params_.selection.max_cluster_retry_count =
        std::max(1, params_.selection.max_cluster_retry_count);
    params_.pruner.candidate_max_unknown_ratio =
        std::clamp(params_.pruner.candidate_max_unknown_ratio, 0.0, 1.0);

    params_.pruner.min_cluster_size =
        static_cast<std::size_t>(params_.runtime.min_frontier_cluster_size);
    params_.selection.small_cluster_size_threshold = std::max<std::size_t>(
        params_.pruner.min_cluster_size + 1U,
        params_.selection.small_cluster_size_threshold);

    goal_provider_.configure(params_);
}

void FrontierExplorerNode::create_interfaces()
{
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(kStatePublisherDepth)).reliable();
    state_pub_ = this->create_publisher<robot_interfaces::msg::ExplorationState>(
        "/frontier_explorer/state", state_qos);
    legacy_state_pub_ = this->create_publisher<robot_interfaces::msg::ExplorationState>(
        "/exploration_state", state_qos);

    start_srv_ = this->create_service<std_srvs::srv::Trigger>("/start_exploration",
        std::bind(&FrontierExplorerNode::handle_start, this,
            std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = this->create_service<std_srvs::srv::Trigger>("/stop_exploration",
        std::bind(&FrontierExplorerNode::handle_stop, this,
            std::placeholders::_1, std::placeholders::_2));

    get_next_frontier_goal_srv_ =
        this->create_service<robot_interfaces::srv::GetNextFrontierGoal>(
            "~/get_next_frontier_goal",
            std::bind(
                &FrontierExplorerNode::handle_get_next_frontier_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    get_frontier_candidates_srv_ =
        this->create_service<robot_interfaces::srv::GetFrontierCandidates>(
            "~/get_frontier_candidates",
            std::bind(
                &FrontierExplorerNode::handle_get_frontier_candidates,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    mark_frontier_failed_srv_ =
        this->create_service<robot_interfaces::srv::MarkFrontierFailed>(
            "~/mark_frontier_failed",
            std::bind(
                &FrontierExplorerNode::handle_mark_frontier_failed,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    clear_frontier_blacklist_srv_ =
        this->create_service<robot_interfaces::srv::ClearFrontierBlacklist>(
            "~/clear_frontier_blacklist",
            std::bind(
                &FrontierExplorerNode::handle_clear_frontier_blacklist,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    get_exploration_state_srv_ =
        this->create_service<robot_interfaces::srv::GetExplorationState>(
            "~/get_exploration_state",
            std::bind(
                &FrontierExplorerNode::handle_get_exploration_state,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        params_.runtime.map_topic, map_qos,
        std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));

    if (params_.runtime.use_global_costmap_for_safety &&
        !params_.runtime.global_costmap_topic.empty())
    {
        global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            params_.runtime.global_costmap_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(
                &FrontierExplorerNode::global_costmap_callback,
                this,
                std::placeholders::_1));
    }
}

void FrontierExplorerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_INFO(this->get_logger(), "Received null map message");
        return;
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Receiving map updates...new map(%u, %u)",
        msg->info.width,
        msg->info.height);

    if (!goal_provider_.update_map(msg, this->now())) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Failed to update frontier map adapter.");
    }
}

void FrontierExplorerNode::global_costmap_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null global costmap message");
        return;
    }

    if (!goal_provider_.update_global_costmap(msg)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Failed to update global costmap adapter.");
    }
}

void FrontierExplorerNode::state_timer_callback()
{
    const auto current_state = get_state();
    if (marker_publisher_ &&
        (current_state == ExplorationState::COMPLETED ||
        current_state == ExplorationState::STOPPED))
    {
        marker_publisher_->clearAll();
    }
    publish_state();
}

bool FrontierExplorerNode::update_robot_pose_from_tf()
{
    if (!tf_buffer_) {
        RCLCPP_WARN(this->get_logger(), "TF buffer is not initialized.");
        return false;
    }

    try {
        const auto transform = tf_buffer_->lookupTransform(
            params_.runtime.global_frame,
            params_.runtime.robot_base_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(
                static_cast<double>(params_.runtime.robot_pose_timeout.count()) / 1000.0));

        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header = transform.header;
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
        goal_provider_.update_robot_pose(robot_pose);
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Failed to lookup robot pose transform %s <- %s: %s",
            params_.runtime.global_frame.c_str(),
            params_.runtime.robot_base_frame.c_str(),
            ex.what());
    }
    goal_provider_.clear_robot_pose();
    return false;
}

void FrontierExplorerNode::publish_markers(const FrontierGoalVisualization & visualization)
{
    if (!marker_publisher_ || !goal_provider_.map_costmap().isReady()) {
        return;
    }

    const auto & costmap = goal_provider_.map_costmap();
    marker_publisher_->clearCandidateMarkers();
    marker_publisher_->clearRejectedMarkers();
    if (!visualization.raw_clusters.empty()) {
        marker_publisher_->publishRawFrontiers(visualization.raw_clusters, costmap);
    }
    if (!visualization.rejected_clusters.empty()) {
        marker_publisher_->publishRejectedFrontiers(visualization.rejected_clusters, costmap);
    }
    if (!visualization.candidates.empty()) {
        std::vector<FrontierCandidate> candidate_markers;
        if (params_.runtime.show_all_candidate_markers) {
            candidate_markers = visualization.candidates;
        } else if (visualization.selected_goal.has_value()) {
            for (const auto & candidate : visualization.candidates) {
                if (candidate.goal == visualization.selected_goal.value()) {
                    candidate_markers.push_back(candidate);
                    break;
                }
            }
        }
        marker_publisher_->publishCandidates(
            candidate_markers,
            costmap,
            visualization.selected_goal);
    }
    if (!visualization.scored_candidates.empty()) {
        marker_publisher_->publishScoredCandidates(
            visualization.scored_candidates,
            costmap,
            visualization.selected_goal);
    }
    if (visualization.selected_goal.has_value() && visualization.robot_grid.has_value()) {
        marker_publisher_->publishSelectedGoal(
            visualization.selected_goal.value(),
            visualization.robot_grid.value(),
            costmap);
    }
    marker_publisher_->publishBlacklist(visualization.blacklisted_goals, costmap);
}

void FrontierExplorerNode::set_state(ExplorationState new_state, const std::string & detail)
{
    state_.store(new_state);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!detail.empty()) {
        state_detail_ = detail;
    } else if (new_state == ExplorationState::RUNNING ||
        new_state == ExplorationState::IDLE ||
        new_state == ExplorationState::COMPLETED ||
        new_state == ExplorationState::STOPPED)
    {
        state_detail_.clear();
    }
}

ExplorationState FrontierExplorerNode::get_state() const
{
    return state_.load();
}

std::string FrontierExplorerNode::state_detail() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_detail_;
}

std::string FrontierExplorerNode::state_to_string() const
{
    return state_to_string(get_state());
}

std::string FrontierExplorerNode::state_to_string(ExplorationState state) const
{
    switch (state) {
        case ExplorationState::IDLE: return "IDLE";
        case ExplorationState::RUNNING: return "RUNNING";
        case ExplorationState::STOPPED: return "STOPPED";
        case ExplorationState::COMPLETED: return "COMPLETED";
        case ExplorationState::STUCK: return "STUCK";
        default: return "UNKNOWN";
    }
}

void FrontierExplorerNode::publish_state()
{
    if (!state_pub_) {
        RCLCPP_WARN(this->get_logger(), "state_pub_ is null");
        return;
    }

    robot_interfaces::msg::ExplorationState msg;
    msg.stamp = this->now();
    const auto current_state = get_state();
    switch (current_state) {
        case ExplorationState::IDLE: msg.state = msg.IDLE; break;
        case ExplorationState::RUNNING: msg.state = msg.RUNNING; break;
        case ExplorationState::STOPPED: msg.state = msg.STOPPED; break;
        case ExplorationState::COMPLETED: msg.state = msg.COMPLETED; break;
        case ExplorationState::STUCK: msg.state = msg.STUCK; break;
        default: msg.state = msg.IDLE; break;
    }
    const auto detail = state_detail();
    msg.detail = detail.empty() ? state_to_string(current_state) : detail;
    state_pub_->publish(msg);
    if (legacy_state_pub_) {
        legacy_state_pub_->publish(msg);
    }
}

void FrontierExplorerNode::handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (marker_publisher_) {
        marker_publisher_->clearAll();
    }
    set_state(ExplorationState::RUNNING);
    publish_state();

    response->success = true;
    response->message = "Frontier capability node marked running. Use exploration BT orchestrator for navigation.";
}

void FrontierExplorerNode::handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    set_state(ExplorationState::STOPPED);
    if (marker_publisher_) {
        marker_publisher_->clearAll();
    }
    publish_state();

    response->success = true;
    response->message = "Frontier capability node stopped.";
}

void FrontierExplorerNode::handle_get_next_frontier_goal(
    const std::shared_ptr<robot_interfaces::srv::GetNextFrontierGoal::Request>,
    std::shared_ptr<robot_interfaces::srv::GetNextFrontierGoal::Response> response)
{
    update_robot_pose_from_tf();
    const auto result = goal_provider_.compute_next_frontier_goal(this->now());
    response->success = result.success;
    response->goal = result.goal;
    response->reason_code = result.reason_code;
    response->reason_text = result.reason_text;
    response->score = result.score;
    response->distance_m = result.distance_m;
    response->clearance_m = result.clearance_m;
    response->raw_frontier_count = result.raw_frontier_count;
    response->candidate_count = result.candidate_count;
    response->blacklist_count = result.blacklist_count;
    response->exploration_complete = result.exploration_complete;
    response->recoverable = result.recoverable;

    if (result.exploration_complete) {
        if (marker_publisher_) {
            marker_publisher_->clearAll();
        }
    } else {
        publish_markers(result.visualization);
    }
    set_state(result.state, result.state_detail);
    publish_state();
}

void FrontierExplorerNode::handle_get_frontier_candidates(
    const std::shared_ptr<robot_interfaces::srv::GetFrontierCandidates::Request> request,
    std::shared_ptr<robot_interfaces::srv::GetFrontierCandidates::Response> response)
{
    update_robot_pose_from_tf();
    const auto max_candidates = request ?
        static_cast<std::size_t>(request->max_candidates) : 0U;
    const auto result = goal_provider_.compute_frontier_candidates(this->now(), max_candidates);

    response->success = result.success;
    response->reason_code = result.reason_code;
    response->reason_text = result.reason_text;
    response->raw_frontier_count = result.raw_frontier_count;
    response->candidate_count = result.candidate_count;
    response->blacklist_count = result.blacklist_count;
    response->exploration_complete = result.exploration_complete;
    response->recoverable = result.recoverable;

    response->goals.reserve(result.candidates.size());
    response->scores.reserve(result.candidates.size());
    response->distance_m.reserve(result.candidates.size());
    response->clearance_m.reserve(result.candidates.size());
    response->unknown_ratio.reserve(result.candidates.size());
    response->cluster_sizes.reserve(result.candidates.size());
    response->retry_counts.reserve(result.candidates.size());
    response->used_fallback.reserve(result.candidates.size());
    response->goal_inset_applied.reserve(result.candidates.size());
    response->reachability_checked.reserve(result.candidates.size());
    response->reachable.reserve(result.candidates.size());
    response->path_length_m.reserve(result.candidates.size());

    for (const auto & candidate : result.candidates) {
        response->goals.push_back(candidate.goal);
        response->scores.push_back(candidate.score);
        response->distance_m.push_back(candidate.distance_m);
        response->clearance_m.push_back(candidate.clearance_m);
        response->unknown_ratio.push_back(candidate.unknown_ratio);
        response->cluster_sizes.push_back(candidate.cluster_size);
        response->retry_counts.push_back(candidate.retry_count);
        response->used_fallback.push_back(candidate.used_fallback);
        response->goal_inset_applied.push_back(candidate.goal_inset_applied);
        response->reachability_checked.push_back(candidate.reachability_checked);
        response->reachable.push_back(candidate.reachable);
        response->path_length_m.push_back(candidate.path_length_m);
    }

    if (result.exploration_complete) {
        if (marker_publisher_) {
            marker_publisher_->clearAll();
        }
    } else {
        publish_markers(result.visualization);
    }
    set_state(result.state, result.state_detail);
    publish_state();
}

void FrontierExplorerNode::handle_mark_frontier_failed(
    const std::shared_ptr<robot_interfaces::srv::MarkFrontierFailed::Request> request,
    std::shared_ptr<robot_interfaces::srv::MarkFrontierFailed::Response> response)
{
    if (!request) {
        response->success = false;
        response->message = "empty request";
        return;
    }

    const auto result = goal_provider_.mark_frontier_failed(request->failed_goal);
    response->success = result.success;
    response->retry_count = result.retry_count;
    response->blacklisted = result.blacklisted;
    response->message = result.message;

    if (marker_publisher_ && goal_provider_.map_costmap().isReady()) {
        marker_publisher_->publishBlacklist(
            result.blacklisted_goals,
            goal_provider_.map_costmap());
    }

    RCLCPP_WARN(
        this->get_logger(),
        "Frontier failure event: reason=%u text=%s retry=%u blacklisted=%s message=%s",
        request->failure_reason,
        request->failure_text.c_str(),
        response->retry_count,
        response->blacklisted ? "true" : "false",
        response->message.c_str());

    set_state(result.state, result.state_detail);
    publish_state();
}

void FrontierExplorerNode::handle_clear_frontier_blacklist(
    const std::shared_ptr<robot_interfaces::srv::ClearFrontierBlacklist::Request>,
    std::shared_ptr<robot_interfaces::srv::ClearFrontierBlacklist::Response> response)
{
    const auto cleared_count = goal_provider_.clear_blacklist();
    response->success = true;
    response->cleared_count = static_cast<uint32_t>(cleared_count);
    response->message = "frontier blacklist cleared";

    if (marker_publisher_ && goal_provider_.map_costmap().isReady()) {
        marker_publisher_->publishBlacklist(
            goal_provider_.blacklisted_goals(),
            goal_provider_.map_costmap());
    }
    set_state(ExplorationState::RUNNING, "BLACKLIST_CLEARED");
    publish_state();
}

void FrontierExplorerNode::handle_get_exploration_state(
    const std::shared_ptr<robot_interfaces::srv::GetExplorationState::Request>,
    std::shared_ptr<robot_interfaces::srv::GetExplorationState::Response> response)
{
    response->state.stamp = this->now();
    const auto current_state = get_state();
    switch (current_state) {
        case ExplorationState::IDLE: response->state.state = response->state.IDLE; break;
        case ExplorationState::RUNNING: response->state.state = response->state.RUNNING; break;
        case ExplorationState::STOPPED: response->state.state = response->state.STOPPED; break;
        case ExplorationState::COMPLETED: response->state.state = response->state.COMPLETED; break;
        case ExplorationState::STUCK: response->state.state = response->state.STUCK; break;
        default: response->state.state = response->state.IDLE; break;
    }
    const auto detail = state_detail();
    response->state.detail = detail.empty() ? state_to_string(current_state) : detail;
}

}  // namespace frontier_explorer
