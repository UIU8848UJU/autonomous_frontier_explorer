#include "frontier_explorer_node.hpp"
#include "map_utils.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include <tf2/utils.hpp> //给 getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr double kEpsilon = 1e-6;

double distance_to_map_edge_m(
    const nav_msgs::msg::OccupancyGrid & map,
    const frontier_explorer::GridCell & cell)
{
    if (map.info.width == 0 || map.info.height == 0) {
        return std::numeric_limits<double>::infinity();
    }

    const int max_row = static_cast<int>(map.info.height) - 1;
    const int max_col = static_cast<int>(map.info.width) - 1;

    const int dist_top = std::max(0, cell.row);
    const int dist_bottom = std::max(0, max_row - cell.row);
    const int dist_left = std::max(0, cell.col);
    const int dist_right = std::max(0, max_col - cell.col);

    const int min_cells = std::min(std::min(dist_top, dist_bottom), std::min(dist_left, dist_right));
    if (min_cells < 0) {
        return 0.0;
    }

    return static_cast<double>(min_cells) * map.info.resolution;
}

bool near_map_edge(
    const nav_msgs::msg::OccupancyGrid & map,
    const frontier_explorer::GridCell & cell,
    double tolerance_m)
{
    if (tolerance_m <= kEpsilon) {
        return false;
    }

    const double distance = distance_to_map_edge_m(map, cell);
    return std::isfinite(distance) && distance <= tolerance_m;
}
}  // namespace


namespace frontier_explorer
{
namespace
{
constexpr size_t kStatePublisherDepth = 10;
}

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: Node("frontier_explorer_node", options),
  map_costmap_(this->get_logger()),
  global_costmap_(this->get_logger()),
  detector_(params_.runtime.obstacle_search_radius_cells, this->get_logger()),
  selector_(
      params_.pruner.min_goal_distance_m,
      params_.selection.max_retry_count,
      params_.scorer.weights,
      params_.pruner.min_cluster_size,
      params_.selection.max_cluster_retry_count,
      params_.pruner.candidate_unknown_margin_cells,
      params_.selection.defer_small_clusters,
      params_.selection.small_cluster_size_threshold,
      this->get_logger())
{
    declare_params();
    load_params();
    apply_params();
    create_interfaces();
    marker_publisher_ = std::make_unique<FrontierMarkerPublisher>(this, this->get_logger(), "map");

    explore_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(params_.runtime.explore_period_sec),
        std::bind(&FrontierExplorerNode::explore_timer_callback, this));

    set_state(ExplorationState::IDLE);
    publish_state();

    LOG_INFO_ONCE(this->get_logger(), "FrontierExplorerNode started.");
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
    this->declare_parameter<double>(
        "frontier_decision.weight_distance",
        params_.scorer.weights.weight_distance);
    this->declare_parameter<double>(
        "frontier_decision.weight_cluster_size",
        params_.scorer.weights.weight_cluster_size);
    this->declare_parameter<double>(
        "frontier_decision.weight_clearance",
        params_.scorer.weights.weight_clearance);
    this->declare_parameter<double>(
        "frontier_decision.weight_revisit_penalty",
        params_.scorer.weights.weight_revisit_penalty);
    this->declare_parameter<double>(
        "frontier_decision.weight_retry_penalty",
        params_.scorer.weights.weight_retry_penalty);
    this->declare_parameter<double>(
        "frontier_decision.weight_unknown_risk_penalty",
        params_.scorer.weights.weight_unknown_risk_penalty);
    this->declare_parameter<double>(
        "frontier_decision.weight_information_gain",
        params_.scorer.weights.weight_information_gain);
    this->declare_parameter<bool>(
        "frontier_decision.enable_clearance_score",
        params_.scorer.weights.enable_clearance_score);
    this->declare_parameter<bool>(
        "frontier_decision.enable_revisit_penalty",
        params_.scorer.weights.enable_revisit_penalty);
    this->declare_parameter<bool>(
        "frontier_decision.enable_unknown_risk_penalty",
        params_.scorer.weights.enable_unknown_risk_penalty);
    this->declare_parameter<bool>(
        "frontier_decision.enable_information_gain_score",
        params_.scorer.weights.enable_information_gain_score);
    this->declare_parameter<int>(
        "frontier_decision.candidate_unknown_margin_cells",
        params_.pruner.candidate_unknown_margin_cells);
    this->declare_parameter<double>(
        "frontier_decision.candidate_max_unknown_ratio",
        params_.scorer.weights.unknown_risk_threshold);
    this->declare_parameter<int>(
        "map_stale_timeout_ms", static_cast<int>(params_.runtime.map_stale_timeout.count()));
    this->declare_parameter<int>(
        "max_frontier_failures", params_.runtime.max_frontier_failures);
    this->declare_parameter<double>("edge_tolerance_m", params_.runtime.edge_tolerance_m);
    this->declare_parameter<std::string>("map_topic", params_.runtime.map_topic);
    this->declare_parameter<std::string>(
        "global_costmap_topic",
        params_.runtime.global_costmap_topic);
    this->declare_parameter<bool>(
        "use_global_costmap_for_safety",
        params_.runtime.use_global_costmap_for_safety);
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
    params_.scorer.weights.weight_distance =
        this->get_parameter("frontier_decision.weight_distance").as_double();
    params_.scorer.weights.weight_cluster_size =
        this->get_parameter("frontier_decision.weight_cluster_size").as_double();
    params_.scorer.weights.weight_clearance =
        this->get_parameter("frontier_decision.weight_clearance").as_double();
    params_.scorer.weights.weight_revisit_penalty =
        this->get_parameter("frontier_decision.weight_revisit_penalty").as_double();
    params_.scorer.weights.weight_retry_penalty =
        this->get_parameter("frontier_decision.weight_retry_penalty").as_double();
    params_.scorer.weights.weight_unknown_risk_penalty =
        this->get_parameter("frontier_decision.weight_unknown_risk_penalty").as_double();
    params_.scorer.weights.weight_information_gain =
        this->get_parameter("frontier_decision.weight_information_gain").as_double();
    params_.scorer.weights.enable_clearance_score =
        this->get_parameter("frontier_decision.enable_clearance_score").as_bool();
    params_.scorer.weights.enable_revisit_penalty =
        this->get_parameter("frontier_decision.enable_revisit_penalty").as_bool();
    params_.scorer.weights.enable_unknown_risk_penalty =
        this->get_parameter("frontier_decision.enable_unknown_risk_penalty").as_bool();
    params_.scorer.weights.enable_information_gain_score =
        this->get_parameter("frontier_decision.enable_information_gain_score").as_bool();
    params_.pruner.candidate_unknown_margin_cells =
        this->get_parameter("frontier_decision.candidate_unknown_margin_cells").as_int();
    params_.scorer.weights.unknown_risk_threshold =
        this->get_parameter("frontier_decision.candidate_max_unknown_ratio").as_double();
    params_.runtime.map_stale_timeout =
        std::chrono::milliseconds(this->get_parameter("map_stale_timeout_ms").as_int());
    params_.runtime.max_frontier_failures =
        this->get_parameter("max_frontier_failures").as_int();
    params_.runtime.edge_tolerance_m =
        this->get_parameter("edge_tolerance_m").as_double();
    params_.runtime.map_topic =
        this->get_parameter("map_topic").as_string();
    params_.runtime.global_costmap_topic =
        this->get_parameter("global_costmap_topic").as_string();
    params_.runtime.use_global_costmap_for_safety =
        this->get_parameter("use_global_costmap_for_safety").as_bool();
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

    params_.selection.max_retry_count =
        std::max(1, params_.selection.max_retry_count);
    params_.selection.max_cluster_retry_count =
        std::max(1, params_.selection.max_cluster_retry_count);
    params_.scorer.weights.unknown_risk_threshold =
        std::clamp(params_.scorer.weights.unknown_risk_threshold, 0.0, 1.0);

    params_.pruner.min_cluster_size =
        static_cast<std::size_t>(params_.runtime.min_frontier_cluster_size);
    params_.selection.small_cluster_size_threshold = std::max<std::size_t>(
        params_.pruner.min_cluster_size + 1U,
        params_.selection.small_cluster_size_threshold);

    detector_ = FrontierDetector(params_.runtime.obstacle_search_radius_cells, this->get_logger());
    selector_ = FrontierSelector(
        params_.pruner.min_goal_distance_m,
        params_.selection.max_retry_count,
        params_.scorer.weights,
        params_.pruner.min_cluster_size,
        params_.selection.max_cluster_retry_count,
        params_.pruner.candidate_unknown_margin_cells,
        params_.selection.defer_small_clusters,
        params_.selection.small_cluster_size_threshold,
        this->get_logger());
}

void FrontierExplorerNode::create_interfaces()
{

    // 控制面
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(kStatePublisherDepth)).reliable();
    state_pub_ = this->create_publisher<robot_interfaces::msg::ExplorationState>(
        "/exploration_state", state_qos);

    start_srv_ = this->create_service<std_srvs::srv::Trigger>("/start_exploration",
        std::bind(&FrontierExplorerNode::handle_start, this,
            std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = this->create_service<std_srvs::srv::Trigger>("/stop_exploration",
        std::bind(&FrontierExplorerNode::handle_stop, this,
            std::placeholders::_1, std::placeholders::_2));


    // map->odom
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable() .transient_local();

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
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&FrontierExplorerNode::odom_callback, this, std::placeholders::_1));
    
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

void FrontierExplorerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    if (!msg) {
        RCLCPP_INFO_WITH_CONTEXT(this->get_logger(), "Received null map message");
        return;
    }

    if (!map_msg_) {
        RCLCPP_WARN_WITH_CONTEXT(
            this->get_logger(),
            "map have'n updating, waitting recovery: width=%u, height=%u, resolution=%.3f",
            msg->info.width,
            msg->info.height,
            msg->info.resolution);
    }
    else if (
        map_msg_->info.width != msg->info.width ||
        map_msg_->info.height != msg->info.height)
    {
        RCLCPP_WARN_WITH_CONTEXT(
            this->get_logger(),
            "Map size changed: old=(%u, %u), new=(%u, %u)",
            map_msg_->info.width,
            map_msg_->info.height,
            msg->info.width,
            msg->info.height);
    }

    RCLCPP_INFO_THROTTLE_WITH_CONTEXT(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Receiving map updates...new map(%u, %u)",            
        msg->info.width,
        msg->info.height
        );
    map_msg_ = msg;
    map_costmap_.updateFromOccupancyGrid(*msg);
    last_map_update_time_ = this->now();
}

void FrontierExplorerNode::global_costmap_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Received null global costmap message");
        return;
    }

    if (!global_costmap_.updateFromOccupancyGrid(*msg)) {
        RCLCPP_WARN_THROTTLE_WITH_CONTEXT(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Failed to update global costmap adapter.");
        return;
    }
}

void FrontierExplorerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_msg_ = msg;
}

bool FrontierExplorerNode::update_robot_grid_position()
{
    if (!map_msg_ || !odom_msg_) {
        RCLCPP_WARN_WITH_CONTEXT(
            this->get_logger(), "have not recive map_msg or odm_msg");
        return false;
    }

    const double robot_x = odom_msg_->pose.pose.position.x;
    const double robot_y = odom_msg_->pose.pose.position.y;

    unsigned int col = 0U;
    unsigned int row = 0U;
    if (!map_costmap_.worldToMap(robot_x, robot_y, col, row)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Robot grid position out of map bounds.");
        return false;
    }

    robot_grid_ = GridCell{static_cast<int>(row), static_cast<int>(col)};
    return true;
}

void FrontierExplorerNode::send_navigation_goal(const GridCell & goal_cell
        ,const geometry_msgs::msg::PoseStamped & pose)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "navigate_to_pose action server not ready.");
        return;
    }
    
    NavigateToPose::Goal goal;
    goal.pose = pose;
    
    RCLCPP_INFO_WITH_CONTEXT(
        this->get_logger(),
        "Sending frontier goal: x=%.3f, y=%.3f",
        pose.pose.position.x, pose.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    
    options.goal_response_callback =
        std::bind(&FrontierExplorerNode::goal_response_callback, this,
              std::placeholders::_1);
    options.feedback_callback  = 
        std::bind(&FrontierExplorerNode::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
        std::bind(&FrontierExplorerNode::result_callback, this,
              std::placeholders::_1);

    current_goal_grid_ = goal_cell;
    initial_goal_distance_ = 0.0;
    goal_progress_.store(0.0f, std::memory_order_relaxed);
    selector_.set_last_goal(goal_cell);
    is_navigating_ = true;
    nav_client_->async_send_goal(goal, options);
}

void FrontierExplorerNode::goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Frontier goal rejected.");
        is_navigating_ = false;

        if (current_goal_grid_.has_value()) {
            selector_.mark_goal_failed(current_goal_grid_.value());
            if (marker_publisher_ && map_costmap_.isReady()) {
                marker_publisher_->publishBlacklist(selector_.blacklisted_goals(), map_costmap_);
            }
        }
        return;
    }

    goal_handle_ = goal_handle;
    RCLCPP_INFO_WITH_CONTEXT(this->get_logger(), "Frontier goal accepted.");
}

void FrontierExplorerNode::result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
{
    is_navigating_ = false;
    last_progress_time_.reset();
    last_progress_distance_ = 0.0;
    initial_goal_distance_ = 0.0;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            set_state(ExplorationState::RUNNING);
  
            if (current_goal_grid_.has_value()) {
                selector_.mark_goal_succeeded(current_goal_grid_.value());
                if (marker_publisher_ && map_costmap_.isReady()) {
                    marker_publisher_->publishBlacklist(
                        selector_.blacklisted_goals(),
                        map_costmap_);
                }
            }
            RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Frontier goal SUCCEEDED, state is:%s",
                state_to_string().c_str());
    
        break;
        case rclcpp_action::ResultCode::ABORTED:
            set_state(ExplorationState::STOPPED);
            if (current_goal_grid_.has_value()) {
                selector_.mark_goal_failed(current_goal_grid_.value());
                if (marker_publisher_ && map_costmap_.isReady()) {
                    marker_publisher_->publishBlacklist(
                        selector_.blacklisted_goals(),
                        map_costmap_);
                }
            }
            RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Frontier goal ABORTED,  state is:%s", 
                state_to_string().c_str());

        break;
        case rclcpp_action::ResultCode::CANCELED:
            set_state(ExplorationState::STOPPED);
            if (current_goal_grid_.has_value()) {
                selector_.mark_goal_failed(current_goal_grid_.value());
                if (marker_publisher_ && map_costmap_.isReady()) {
                    marker_publisher_->publishBlacklist(
                        selector_.blacklisted_goals(),
                        map_costmap_);
                }
            }
            RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Frontier goal CANCELED,  state is:%s.",
                state_to_string().c_str());
    
        break;
        default:
            set_state(ExplorationState::STOPPED);
            if (current_goal_grid_.has_value()) {
                selector_.mark_goal_failed(current_goal_grid_.value());
                if (marker_publisher_ && map_costmap_.isReady()) {
                    marker_publisher_->publishBlacklist(
                        selector_.blacklisted_goals(),
                        map_costmap_);
                }
            }
            RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Frontier goal unknown result. state is:%s",
                state_to_string().c_str());
        break;
    }
    publish_state();
}


// 异步反馈回调
void FrontierExplorerNode::feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                                             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    if (!feedback) {
        return;
    }

    if (!current_goal_grid_.has_value() || !map_msg_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Navigation feedback received before goal or map is available.");
        return;
    }

    // 传回来的机器坐标
    const auto & global_robot_pos = feedback->current_pose.pose;
    if (!map_costmap_.inBounds(current_goal_grid_->col, current_goal_grid_->row)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Current goal is outside of the available map.");
        return;
    }
    geometry_msgs::msg::Point global_goal_pos;
    map_costmap_.mapToWorld(
        static_cast<unsigned int>(current_goal_grid_->col),
        static_cast<unsigned int>(current_goal_grid_->row),
        global_goal_pos.x,
        global_goal_pos.y);
    global_goal_pos.z = 0.0;

    const double theta = tf2::getYaw(global_robot_pos.orientation);

    RCLCPP_INFO_THROTTLE_WITH_CONTEXT(this->get_logger(), *this->get_clock(), 
                3000, "robot:x=%.3f y=%.3f theta=%.3f, target:robot:x=%.3f y=%.3f",
                global_robot_pos.position.x, global_robot_pos.position.y, theta,
                global_goal_pos.x, global_goal_pos.y);

    // 更新内部 robot_grid_（当前网格坐标）
    unsigned int robot_col = 0U;
    unsigned int robot_row = 0U;
    if (!map_costmap_.worldToMap(
            global_robot_pos.position.x,
            global_robot_pos.position.y,
            robot_col,
            robot_row))
    {
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Position out of map bounds!");
        return;  
    }
    robot_grid_ = GridCell{static_cast<int>(robot_row), static_cast<int>(robot_col)};

    // 计算到目标点的距离
    double dist_to_goal = distance_in_meters(
        global_robot_pos.position,
        global_goal_pos);

    if (initial_goal_distance_ <= 0.0) {
        initial_goal_distance_ = std::max(dist_to_goal, 1e-3);
    }
    double progress_ratio = 1.0 - dist_to_goal / initial_goal_distance_;
    progress_ratio = std::clamp(progress_ratio, 0.0, 1.0);
    goal_progress_.store(static_cast<float>(progress_ratio), std::memory_order_relaxed);
    RCLCPP_DEBUG_WITH_CONTEXT(
        this->get_logger(),
        "goal progress: %.1f%% (remaining %.3fm)",
        progress_ratio * 100.0,
        dist_to_goal);

    if (dist_to_goal < 0.05) {
        set_state(ExplorationState::COMPLETED);
    } else {
        set_state(ExplorationState::RUNNING);
    }

    // fallback 处理：如果长时间没有明显进展
    auto now = this->now();
    if (!last_progress_time_) {
        last_progress_time_ = now;
        last_progress_distance_ = dist_to_goal;
    } else {
        double delta = std::abs(dist_to_goal - last_progress_distance_);
        if (delta < 0.01 && (now - *last_progress_time_).seconds() > 5.0) {
            RCLCPP_WARN_WITH_CONTEXT(this->get_logger(),
                        "Goal seems stuck! triggering fallback.");
            set_state(ExplorationState::STUCK);
            last_progress_time_ = now;
            last_progress_distance_ = dist_to_goal;  // 重置防止重复触发
        } else if (delta >= 0.01) {
            last_progress_time_ = now;
            last_progress_distance_ = dist_to_goal;
        }
    }

    publish_state();  // 每次反馈更新状态
}

void FrontierExplorerNode::explore_timer_callback()
{
    if(get_state() != ExplorationState::RUNNING)
    {
        RCLCPP_DEBUG(this->get_logger(), "controll is not running");
        publish_state();
        return;
    }

    if (is_navigating_) {
        RCLCPP_DEBUG(this->get_logger(), "Still navigating, skip exploration tick.");
        publish_state();
        return;
    }

    if (!map_msg_) {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No map data available.");
        publish_state();
        return;
    }

    const auto now = this->now();
    if (last_map_update_time_.nanoseconds() > 0) {
        const auto elapsed = now - last_map_update_time_;
        if (elapsed > rclcpp::Duration(params_.runtime.map_stale_timeout)) {
            RCLCPP_WARN_THROTTLE_WITH_CONTEXT(
                this->get_logger(), *this->get_clock(), 2000,
                "Map has not updated for %.2f seconds, marking STUCK.",
                elapsed.seconds());
            set_state(ExplorationState::STUCK, "map_stale");
            publish_state();
            return;
        }
    }

    if (!update_robot_grid_position()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Robot grid position unavailable.");
        publish_state();
        return;
    }

    if (!map_costmap_.isReady()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Costmap adapter is not ready.");
        return;
    }

    const auto frontier_cells = detector_.detect_frontier_cells(map_costmap_);
    const auto clusters = detector_.cluster_frontiers(map_costmap_, frontier_cells);
    if (marker_publisher_) {
        marker_publisher_->publishRawFrontiers(clusters, map_costmap_);
    }

    RCLCPP_INFO_THROTTLE_WITH_CONTEXT(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Detected frontier cells: %zu, raw clusters: %zu, min_cluster_size=%zu",
        frontier_cells.size(),
        clusters.size(),
        params_.pruner.min_cluster_size);

    const CostmapAdapter * safety_costmap =
        (params_.runtime.use_global_costmap_for_safety && global_costmap_.isReady()) ?
        &global_costmap_ : &map_costmap_;

    const auto best_frontier = selector_.choose_best_frontier(
        clusters,
        robot_grid_.value(),
        map_costmap_.getResolution(),
        map_costmap_,
        safety_costmap);
    if (marker_publisher_) {
        std::vector<FrontierCandidate> scored_candidates;
        scored_candidates.reserve(selector_.last_scored_candidates().size());
        for (const auto & scored : selector_.last_scored_candidates()) {
            scored_candidates.push_back(scored.candidate);
        }
        marker_publisher_->publishCandidates(scored_candidates, map_costmap_);
        marker_publisher_->publishScoredCandidates(
            selector_.last_scored_candidates(),
            map_costmap_);
        marker_publisher_->publishBlacklist(selector_.blacklisted_goals(), map_costmap_);
    }

    if (!best_frontier.has_value()) {
        if (marker_publisher_) {
            marker_publisher_->publishRejectedFrontiers(clusters, map_costmap_);
        }
        RCLCPP_WARN_WITH_CONTEXT(
            this->get_logger(),
            "No valid frontier selected. robot_grid=(%d, %d), clusters=%zu",
            robot_grid_->row,
            robot_grid_->col,
            clusters.size());
        ++consecutive_frontier_failures_;
        if (consecutive_frontier_failures_ >=
            static_cast<std::size_t>(params_.runtime.max_frontier_failures))
        {
            const bool near_edge = robot_grid_.has_value() &&
                near_map_edge(
                    *map_msg_,
                    robot_grid_.value(),
                    params_.runtime.edge_tolerance_m);
            const std::string reason = near_edge ?
                "no_frontier_near_edge" : "no_valid_frontier";
            RCLCPP_WARN_WITH_CONTEXT(
                this->get_logger(),
                "Frontier selection failed %zu times (near_edge=%s).",
                consecutive_frontier_failures_,
                near_edge ? "true" : "false");
            set_state(ExplorationState::STUCK, reason);
            publish_state();
            consecutive_frontier_failures_ = 0;
        } else {
            publish_state();
        }
        return;
    }
    else{
        consecutive_frontier_failures_ = 0;
        if (marker_publisher_) {
            marker_publisher_->publishSelectedGoal(
                best_frontier.value(),
                robot_grid_.value(),
                map_costmap_);
        }
        RCLCPP_INFO_WITH_CONTEXT(
            this->get_logger(),
            "Chosen frontier: row=%d, col=%d, robot=(%d, %d)", 
            best_frontier->row,
            best_frontier->col,
            robot_grid_->row,
            robot_grid_->col);
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    map_costmap_.mapToWorld(
        static_cast<unsigned int>(best_frontier->col),
        static_cast<unsigned int>(best_frontier->row),
        goal_pose.pose.position.x,
        goal_pose.pose.position.y);
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;
    send_navigation_goal(best_frontier.value(), goal_pose);
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
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "state_pub_ is null");
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
}

void FrontierExplorerNode::handle_start( const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (marker_publisher_) {
        marker_publisher_->clearAll();
    }
    set_state(ExplorationState::RUNNING);
    publish_state();

    response->success = true;
    response->message = "Exploration started.";
}

void FrontierExplorerNode::handle_stop( const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    set_state(ExplorationState::STOPPED);
    if (marker_publisher_) {
        marker_publisher_->clearAll();
    }
    publish_state();

    response->success = true;
    response->message = "Exploration stopped.";
}



}  // namespace frontier_explorer
