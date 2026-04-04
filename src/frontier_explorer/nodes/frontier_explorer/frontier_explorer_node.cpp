#include "frontier_explorer_node.hpp"
#include "map_utils.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_set>

namespace frontier_explorer
{

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: Node("frontier_explorer_node", options),
  detector_(obstacle_search_radius_cells_, min_frontier_cluster_size_),
  selector_(min_goal_distance_m_, max_retry_count_)
{
    declare_params();
    detector_ = FrontierDetector(
        obstacle_search_radius_cells_,
        min_frontier_cluster_size_);

    selector_ = FrontierSelector(
        min_goal_distance_m_,
        max_retry_count_);

    create_interfaces();

    explore_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(explore_period_sec_),
        std::bind(&FrontierExplorerNode::explore_timer_callback, this));

    state_ = ExplorationState::IDLE;
    publish_state();

    RCLCPP_INFO(this->get_logger(), "FrontierExplorerNode started.");
}

void FrontierExplorerNode::declare_params()
{
    this->declare_parameter<double>("explore_period_sec", 3.0);
    this->declare_parameter<int>("obstacle_search_radius_cells", 2);
    this->declare_parameter<int>("min_frontier_cluster_size", 5);
    this->declare_parameter<double>("min_goal_distance_m", 0.5);
    this->declare_parameter<int>("max_retry_count", 2);

    explore_period_sec_ = 
        this->get_parameter("explore_period_sec").as_double();
    obstacle_search_radius_cells_ = 
        this->get_parameter("obstacle_search_radius_cells").as_int();
    min_frontier_cluster_size_ = 
        this->get_parameter("min_frontier_cluster_size").as_int();
    min_goal_distance_m_ = 
        this->get_parameter("min_goal_distance_m").as_double();
    max_retry_count_ =
        this->get_parameter("max_retry_count").as_int();
}

void FrontierExplorerNode::create_interfaces()
{
    // map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/map", rclcpp::QoS(10),
    //     std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/exploration_state", 10);

    start_srv_ = this->create_service<std_srvs::srv::Trigger>("/start_exploration",
        std::bind(&FrontierExplorerNode::handle_start, this,
            std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = this->create_service<std_srvs::srv::Trigger>("/stop_exploration",
        std::bind(&FrontierExplorerNode::handle_stop, this,
            std::placeholders::_1, std::placeholders::_2));


    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable() .transient_local();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&FrontierExplorerNode::odom_callback, this, std::placeholders::_1));
    
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

void FrontierExplorerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    if (!msg) {
        RCLCPP_INFO(this->get_logger(), "Received null map message");
        return;
    }

    if (!map_msg_) {
        RCLCPP_INFO(
            this->get_logger(),
            "First map received: width=%u, height=%u, resolution=%.3f",
            msg->info.width,
            msg->info.height,
            msg->info.resolution);
    }
    else if (
        map_msg_->info.width != msg->info.width ||
        map_msg_->info.height != msg->info.height)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Map size changed: old=(%u, %u), new=(%u, %u)",
            map_msg_->info.width,
            map_msg_->info.height,
            msg->info.width,
            msg->info.height);
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Receiving map updates..."
    );

    map_msg_ = msg;
}

void FrontierExplorerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_msg_ = msg;
}

bool FrontierExplorerNode::update_robot_grid_position()
{
    if (!map_msg_ || !odom_msg_) {
        return false;
    }

    const double origin_x = map_msg_->info.origin.position.x;
    const double origin_y = map_msg_->info.origin.position.y;
    const double resolution = map_msg_->info.resolution;

    const double robot_x = odom_msg_->pose.pose.position.x;
    const double robot_y = odom_msg_->pose.pose.position.y;

    const int col = static_cast<int>((robot_x - origin_x) / resolution);
    const int row = static_cast<int>((robot_y - origin_y) / resolution);

    if (!in_bounds(*map_msg_, row, col)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Robot grid position out of map bounds.");
        return false;
    }

    robot_grid_ = GridCell{row, col};
    return true;
}

void FrontierExplorerNode::send_navigation_goal(const GridCell & goal_cell
        ,const geometry_msgs::msg::PoseStamped & pose)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not ready.");
        return;
    }
    
    NavigateToPose::Goal goal;
    goal.pose = pose;
    
    RCLCPP_INFO(
        this->get_logger(),
        "Sending frontier goal: x=%.3f, y=%.3f",
        pose.pose.position.x, pose.pose.position.y);

    auto options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
    options.goal_response_callback =
        std::bind(&FrontierExplorerNode::goal_response_callback, this,
              std::placeholders::_1);
    options.result_callback =
        std::bind(&FrontierExplorerNode::result_callback, this,
              std::placeholders::_1);

    current_goal_grid_ = goal_cell;
    selector_.set_last_goal(goal_cell);
    is_navigating_ = true;
    nav_client_->async_send_goal(goal, options);
}

void FrontierExplorerNode::goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Frontier goal rejected.");
        is_navigating_ = false;

        if (current_goal_grid_.has_value()) {
        selector_.mark_goal_failed(current_goal_grid_.value());
        }
        return;
    }

    goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Frontier goal accepted.");
}

void FrontierExplorerNode::result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
{
    is_navigating_ = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Frontier goal succeeded.");
              if (current_goal_grid_.has_value()) {
                selector_.mark_goal_succeeded(current_goal_grid_.value());
                }
        break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Frontier goal aborted.");
                if (current_goal_grid_.has_value()) {
                    selector_.mark_goal_failed(current_goal_grid_.value());
                }

        break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Frontier goal canceled.");
            if (current_goal_grid_.has_value()) {
                selector_.mark_goal_failed(current_goal_grid_.value());
            }
        break;
        default:
            RCLCPP_WARN(this->get_logger(), "Frontier goal unknown result.");
            if (current_goal_grid_.has_value()) {
                selector_.mark_goal_failed(current_goal_grid_.value());
            }
        break;
    }
}

void FrontierExplorerNode::explore_timer_callback()
{
    if (!running_) {
        RCLCPP_DEBUG(this->get_logger(), "[explore_timer_callback] controll is not running");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "tick: enter");
    if (is_navigating_) {
        RCLCPP_DEBUG(this->get_logger(), "Still navigating, skip exploration tick.");
        return;
    }

    if (!map_msg_) {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No map data available.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "tick: before update_robot_grid_position");

    if (!update_robot_grid_position()) {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Robot grid position unavailable.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "tick: before detect_frontier_cells");
    const auto frontier_cells = detector_.detect_frontier_cells(*map_msg_);
    RCLCPP_INFO(this->get_logger(), "tick: before cluster_frontiers");
    const auto clusters = detector_.cluster_frontiers(*map_msg_, frontier_cells);

    RCLCPP_INFO(
        this->get_logger(),
        "Detected frontier cells: %zu, clusters: %zu",
        frontier_cells.size(), clusters.size());

    RCLCPP_INFO(this->get_logger(), "tick: before choose_best_frontier");
    const auto best_frontier = selector_.choose_best_frontier(
        clusters,
        robot_grid_.value(),
        map_msg_->info.resolution);

    RCLCPP_INFO(this->get_logger(), "tick: before grid_to_goal_pose");
    if (!best_frontier.has_value()) {
        RCLCPP_WARN(
            this->get_logger(),
            "No valid frontier selected. robot_grid=(%d, %d), clusters=%zu",
            robot_grid_->row,
            robot_grid_->col,
            clusters.size());
        return;
    }
    else{
        RCLCPP_INFO(
            this->get_logger(),
            "Chosen frontier: row=%d, col=%d, robot=(%d, %d)",
            best_frontier->row,
            best_frontier->col,
            robot_grid_->row,
            robot_grid_->col);
    }

    const auto goal_pose = grid_to_goal_pose( *map_msg_, best_frontier.value(), this->now());
    RCLCPP_INFO(this->get_logger(), "tick: before send_navigation_goal");
    send_navigation_goal(best_frontier.value(), goal_pose);
}


std::string FrontierExplorerNode::state_to_string() const
{
    switch (state_) {
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

    std_msgs::msg::String msg;
    msg.data = state_to_string();
    state_pub_->publish(msg);
}

void FrontierExplorerNode::handle_start( const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    running_ = true;
    state_ = ExplorationState::RUNNING;
    publish_state();

    response->success = true;
    response->message = "Exploration started.";
}

void FrontierExplorerNode::handle_stop( const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    running_ = false;
    state_ = ExplorationState::STOPPED;
    publish_state();

    response->success = true;
    response->message = "Exploration stopped.";
}

}  // namespace frontier_explorer