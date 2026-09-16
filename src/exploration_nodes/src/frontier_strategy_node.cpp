#include "exploration_nodes/nodes/frontier_strategy_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <sstream>

#include "exploration_nodes/nodes/nav2_planner_reachability_checker.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace exploration
{

using namespace frontier_strategy;
namespace
{
}  // 命名空间

FrontierStrategyNode::FrontierStrategyNode(const rclcpp::NodeOptions & options)
: Node("frontier_strategy_node", options),
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

    set_state(ExplorationStatus::IDLE);

    RCLCPP_INFO(this->get_logger(), "FrontierStrategyNode capability wrapper started.");
}

void FrontierStrategyNode::declare_params()
{
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
    this->declare_parameter<bool>(
        "frontier_decision.enable_footprint_filter",
        params_.pruner.enable_footprint_filter);
    this->declare_parameter<bool>(
        "frontier_decision.allow_unknown_footprint",
        params_.pruner.allow_unknown_footprint);
    this->declare_parameter<double>(
        "frontier_decision.robot_radius",
        params_.pruner.robot_radius);
    this->declare_parameter<double>(
        "frontier_decision.footprint_padding",
        params_.pruner.footprint_padding);
    this->declare_parameter<int>(
        "frontier_decision.footprint_cost_threshold",
        params_.pruner.footprint_cost_threshold);
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
    this->declare_parameter<int>(
        "stable_no_frontier_cycles", params_.runtime.stable_no_frontier_cycles);
    this->declare_parameter<bool>(
        "frontier_decision.cleanup_enabled", params_.runtime.cleanup_enabled);
    this->declare_parameter<int>(
        "frontier_decision.cleanup_trigger_no_candidate_cycles",
        params_.runtime.cleanup_trigger_no_candidate_cycles);
    this->declare_parameter<bool>(
        "frontier_decision.cleanup_trigger_only_small_clusters",
        params_.runtime.cleanup_trigger_only_small_clusters);
    this->declare_parameter<int>(
        "frontier_decision.cleanup_min_cluster_size",
        static_cast<int>(params_.pruner.cleanup_min_cluster_size));
    this->declare_parameter<double>(
        "frontier_decision.cleanup_candidate_max_unknown_ratio",
        params_.pruner.cleanup_candidate_max_unknown_ratio);
    this->declare_parameter<double>(
        "frontier_decision.sensor_range_m",
        params_.pruner.sensor_range_m);
    this->declare_parameter<std::vector<double>>(
        "frontier_decision.viewpoint_retreat_distances_m",
        params_.pruner.viewpoint_retreat_distances_m);
    this->declare_parameter<std::vector<double>>(
        "frontier_decision.viewpoint_sample_radii_m",
        params_.pruner.viewpoint_sample_radii_m);
    this->declare_parameter<double>(
        "frontier_decision.viewpoint_angle_step_deg",
        params_.pruner.viewpoint_angle_step_deg);
    this->declare_parameter<double>(
        "frontier_decision.information_gain_ray_step_cells",
        params_.pruner.information_gain_ray_step_cells);
    this->declare_parameter<int>(
        "frontier_decision.minimum_visible_unknown_cells",
        static_cast<int>(params_.pruner.minimum_visible_unknown_cells));
}

void FrontierStrategyNode::load_params()
{
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
    params_.pruner.enable_footprint_filter =
        this->get_parameter("frontier_decision.enable_footprint_filter").as_bool();
    params_.pruner.allow_unknown_footprint =
        this->get_parameter("frontier_decision.allow_unknown_footprint").as_bool();
    params_.pruner.robot_radius =
        this->get_parameter("frontier_decision.robot_radius").as_double();
    params_.pruner.footprint_padding =
        this->get_parameter("frontier_decision.footprint_padding").as_double();
    params_.pruner.footprint_cost_threshold =
        static_cast<int>(this->get_parameter(
            "frontier_decision.footprint_cost_threshold").as_int());
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
    params_.runtime.stable_no_frontier_cycles =
        this->get_parameter("stable_no_frontier_cycles").as_int();
    params_.runtime.cleanup_enabled =
        this->get_parameter("frontier_decision.cleanup_enabled").as_bool();
    params_.runtime.cleanup_trigger_no_candidate_cycles =
        this->get_parameter("frontier_decision.cleanup_trigger_no_candidate_cycles").as_int();
    params_.runtime.cleanup_trigger_only_small_clusters =
        this->get_parameter("frontier_decision.cleanup_trigger_only_small_clusters").as_bool();
    params_.pruner.cleanup_min_cluster_size = static_cast<std::size_t>(
        this->get_parameter("frontier_decision.cleanup_min_cluster_size").as_int());
    params_.pruner.cleanup_candidate_max_unknown_ratio =
        this->get_parameter("frontier_decision.cleanup_candidate_max_unknown_ratio").as_double();
    params_.pruner.sensor_range_m =
        this->get_parameter("frontier_decision.sensor_range_m").as_double();
    params_.pruner.viewpoint_retreat_distances_m =
        this->get_parameter("frontier_decision.viewpoint_retreat_distances_m").as_double_array();
    params_.pruner.viewpoint_sample_radii_m =
        this->get_parameter("frontier_decision.viewpoint_sample_radii_m").as_double_array();
    params_.pruner.viewpoint_angle_step_deg =
        this->get_parameter("frontier_decision.viewpoint_angle_step_deg").as_double();
    params_.pruner.information_gain_ray_step_cells =
        this->get_parameter("frontier_decision.information_gain_ray_step_cells").as_double();
    params_.pruner.minimum_visible_unknown_cells = static_cast<std::size_t>(
        this->get_parameter("frontier_decision.minimum_visible_unknown_cells").as_int());

}

void FrontierStrategyNode::apply_params()
{
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
    params_.runtime.stable_no_frontier_cycles =
        std::max(1, params_.runtime.stable_no_frontier_cycles);
    params_.runtime.cleanup_trigger_no_candidate_cycles =
        std::max(1, params_.runtime.cleanup_trigger_no_candidate_cycles);
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
    params_.pruner.cleanup_min_cluster_size = std::max<std::size_t>(
        1U, params_.pruner.cleanup_min_cluster_size);
    params_.pruner.cleanup_candidate_max_unknown_ratio = std::clamp(
        params_.pruner.cleanup_candidate_max_unknown_ratio, 0.0, 1.0);
    params_.pruner.robot_radius =
        std::max(0.01, params_.pruner.robot_radius);
    params_.pruner.footprint_padding =
        std::max(0.0, params_.pruner.footprint_padding);
    params_.pruner.footprint_cost_threshold =
        std::clamp(params_.pruner.footprint_cost_threshold, 1, 255);
    params_.pruner.sensor_range_m = std::max(0.0, params_.pruner.sensor_range_m);
    params_.pruner.viewpoint_angle_step_deg = std::clamp(
        params_.pruner.viewpoint_angle_step_deg, 0.1, 360.0);
    params_.pruner.information_gain_ray_step_cells = std::max(
        0.1, params_.pruner.information_gain_ray_step_cells);
    params_.pruner.viewpoint_retreat_distances_m.erase(
        std::remove_if(
            params_.pruner.viewpoint_retreat_distances_m.begin(),
            params_.pruner.viewpoint_retreat_distances_m.end(),
            [](double value) { return value <= 0.0; }),
        params_.pruner.viewpoint_retreat_distances_m.end());
    params_.pruner.viewpoint_sample_radii_m.erase(
        std::remove_if(
            params_.pruner.viewpoint_sample_radii_m.begin(),
            params_.pruner.viewpoint_sample_radii_m.end(),
            [](double value) { return value <= 0.0; }),
        params_.pruner.viewpoint_sample_radii_m.end());

    params_.pruner.min_cluster_size =
        static_cast<std::size_t>(params_.runtime.min_frontier_cluster_size);
    params_.selection.small_cluster_size_threshold = std::max<std::size_t>(
        params_.pruner.min_cluster_size + 1U,
        params_.selection.small_cluster_size_threshold);

    goal_provider_.configure(params_);
}

void FrontierStrategyNode::create_interfaces()
{
    decision_debug_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/exploration/decision_debug_json",
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    get_frontier_candidates_srv_ =
        this->create_service<robot_interfaces::srv::GetFrontierCandidates>(
            "~/get_frontier_candidates",
            std::bind(
                &FrontierStrategyNode::handle_get_frontier_candidates,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    mark_frontier_failed_srv_ =
        this->create_service<robot_interfaces::srv::MarkFrontierFailed>(
            "~/mark_frontier_failed",
            std::bind(
                &FrontierStrategyNode::handle_mark_frontier_failed,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    clear_frontier_blacklist_srv_ =
        this->create_service<robot_interfaces::srv::ClearFrontierBlacklist>(
            "~/clear_frontier_blacklist",
            std::bind(
                &FrontierStrategyNode::handle_clear_frontier_blacklist,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    get_exploration_state_srv_ =
        this->create_service<robot_interfaces::srv::GetExplorationState>(
            "~/get_exploration_state",
            std::bind(
                &FrontierStrategyNode::handle_get_exploration_state,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        params_.runtime.map_topic, map_qos,
        std::bind(&FrontierStrategyNode::map_callback, this, std::placeholders::_1));

    if (params_.runtime.use_global_costmap_for_safety &&
        !params_.runtime.global_costmap_topic.empty())
    {
        global_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            params_.runtime.global_costmap_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            std::bind(
                &FrontierStrategyNode::global_costmap_callback,
                this,
                std::placeholders::_1));
    }
}

void FrontierStrategyNode::map_callback(
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

void FrontierStrategyNode::global_costmap_callback(
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

bool FrontierStrategyNode::update_robot_pose_from_tf()
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

void FrontierStrategyNode::publish_markers(const FrontierGoalVisualization & visualization)
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

std::string FrontierStrategyNode::escape_json_string(const std::string & value) const
{
    std::ostringstream escaped;
    for (const char ch : value) {
        switch (ch) {
            case '\\': escaped << "\\\\"; break;
            case '"': escaped << "\\\""; break;
            case '\b': escaped << "\\b"; break;
            case '\f': escaped << "\\f"; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20U) {
                    escaped << "\\u"
                            << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<int>(static_cast<unsigned char>(ch))
                            << std::dec << std::setfill(' ');
                } else {
                    escaped << ch;
                }
                break;
        }
    }
    return escaped.str();
}

void FrontierStrategyNode::publish_decision_debug(const FrontierCandidatesResult & result)
{
    if (!decision_debug_pub_) {
        return;
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{"
         << "\"event\":\"frontier_candidates\","
         << "\"success\":" << (result.success ? "true" : "false") << ","
         << "\"reason_code\":" << result.reason_code << ","
         << "\"reason_text\":\"" << escape_json_string(result.reason_text) << "\","
         << "\"raw_frontier_count\":" << result.raw_frontier_count << ","
         << "\"candidate_count\":" << result.candidate_count << ","
         << "\"blacklist_count\":" << result.blacklist_count << ","
         << "\"exploration_complete\":" << (result.exploration_complete ? "true" : "false") << ","
         << "\"cleanup_mode\":" << (result.cleanup_mode ? "true" : "false") << ","
         << "\"detection_ms\":" << result.detection_ms << ","
         << "\"pruning_ms\":" << result.pruning_ms << ","
         << "\"ranking_ms\":" << result.ranking_ms << ","
         << "\"total_ms\":" << result.total_ms << ","
         << "\"map_revision\":" << result.map_revision << ","
         << "\"stable_no_frontier_cycles\":" << result.stable_no_frontier_cycles << ","
         << "\"diagnostics\":{";
    for (std::size_t reason_index = 0U;
        reason_index < static_cast<std::size_t>(FrontierRejectionReason::COUNT);
        ++reason_index)
    {
        if (reason_index > 0U) {
            json << ",";
        }
        const auto reason = static_cast<FrontierRejectionReason>(reason_index);
        json << "\"" << frontier_rejection_reason_name(reason) << "\":"
             << result.diagnostics.rejection_count(reason);
    }
    json << "},"
         << "\"raw_frontier_cells\":" << result.diagnostics.raw_frontier_cells << ","
         << "\"raw_clusters\":" << result.diagnostics.raw_clusters << ","
         << "\"generated_candidates\":" << result.diagnostics.generated_candidates
         << ",\"candidates\":[";
    for (std::size_t index = 0; index < result.candidates.size(); ++index) {
        const auto & candidate = result.candidates[index];
        if (index > 0U) {
            json << ",";
        }
        json << "{"
             << "\"candidate_id\":" << index << ","
             << "\"x\":" << candidate.goal.pose.position.x << ","
             << "\"y\":" << candidate.goal.pose.position.y << ","
             << "\"score\":" << candidate.score << ","
             << "\"distance_m\":" << candidate.distance_m << ","
             << "\"clearance_m\":" << candidate.clearance_m << ","
             << "\"unknown_ratio\":" << candidate.unknown_ratio << ","
             << "\"information_gain\":" << candidate.information_gain << ","
             << "\"cluster_size\":" << candidate.cluster_size << ","
             << "\"retry_count\":" << candidate.retry_count << ","
             << "\"reachable\":" << (candidate.reachable ? "true" : "false") << ","
             << "\"path_length_m\":" << candidate.path_length_m
             << "}";
    }
    json << "]}";

    std_msgs::msg::String msg;
    msg.data = json.str();
    decision_debug_pub_->publish(msg);
}

void FrontierStrategyNode::set_state(ExplorationStatus new_state, const std::string & detail)
{
    state_.store(new_state);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!detail.empty()) {
        state_detail_ = detail;
    } else if (new_state == ExplorationStatus::RUNNING ||
        new_state == ExplorationStatus::IDLE ||
        new_state == ExplorationStatus::COMPLETED)
    {
        state_detail_.clear();
    }
}

ExplorationStatus FrontierStrategyNode::get_state() const
{
    return state_.load();
}

std::string FrontierStrategyNode::state_detail() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_detail_;
}

std::string FrontierStrategyNode::state_to_string() const
{
    return state_to_string(get_state());
}

std::string FrontierStrategyNode::state_to_string(ExplorationStatus state) const
{
    switch (state) {
        case ExplorationStatus::IDLE: return "IDLE";
        case ExplorationStatus::RUNNING: return "RUNNING";
        case ExplorationStatus::COMPLETED: return "COMPLETED";
        case ExplorationStatus::STUCK: return "STUCK";
        case ExplorationStatus::FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

void FrontierStrategyNode::handle_get_frontier_candidates(
    const std::shared_ptr<robot_interfaces::srv::GetFrontierCandidates::Request> request,
    std::shared_ptr<robot_interfaces::srv::GetFrontierCandidates::Response> response)
{
    update_robot_pose_from_tf();
    const auto max_candidates = request ?
        static_cast<std::size_t>(request->max_candidates) : 0U;
    const auto result = goal_provider_.compute_frontier_candidates(this->now(), max_candidates);
    publish_decision_debug(result);

    response->success = result.success;
    response->reason_code = result.reason_code;
    response->reason_text = result.reason_text;
    response->raw_frontier_count = result.raw_frontier_count;
    response->candidate_count = result.candidate_count;
    response->blacklist_count = result.blacklist_count;
    response->map_revision = result.map_revision;
    response->exploration_complete = result.exploration_complete;
    response->recoverable = result.recoverable;

    response->candidates = result.candidates;

    if (result.exploration_complete) {
        if (marker_publisher_) {
            marker_publisher_->clearAll();
        }
    } else {
        publish_markers(result.visualization);
    }
    set_state(result.state, result.state_detail);
}

void FrontierStrategyNode::handle_mark_frontier_failed(
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
}

void FrontierStrategyNode::handle_clear_frontier_blacklist(
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
    set_state(ExplorationStatus::RUNNING, "BLACKLIST_CLEARED");
}

void FrontierStrategyNode::handle_get_exploration_state(
    const std::shared_ptr<robot_interfaces::srv::GetExplorationState::Request>,
    std::shared_ptr<robot_interfaces::srv::GetExplorationState::Response> response)
{
    response->state.stamp = this->now();
    const auto current_state = get_state();
    switch (current_state) {
        case ExplorationStatus::IDLE: response->state.state = response->state.IDLE; break;
        case ExplorationStatus::RUNNING: response->state.state = response->state.RUNNING; break;
        case ExplorationStatus::COMPLETED: response->state.state = response->state.COMPLETED; break;
        case ExplorationStatus::STUCK: response->state.state = response->state.STUCK; break;
        case ExplorationStatus::FAILED: response->state.state = response->state.FAILED; break;
        default: response->state.state = response->state.IDLE; break;
    }
    const auto detail = state_detail();
    response->state.detail = detail.empty() ? state_to_string(current_state) : detail;
}

}  // 命名空间 exploration
