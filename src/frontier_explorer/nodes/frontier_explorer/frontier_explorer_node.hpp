#pragma once

// 日志宏
#define __CLASS_NAME__ "FrontierExplorerNode"

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"   //目标位姿
#include "nav2_msgs/action/navigate_to_pose.hpp"    //Nav2 的导航 action
#include "nav_msgs/msg/occupancy_grid.hpp"  //地图
#include "nav_msgs/msg/odometry.hpp"    //里程计
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "types.h"
#include "frontier_detector.hpp"
#include "frontier_selector.hpp"
#include "map_utils.hpp"
// 状态节点
#include "std_srvs/srv/trigger.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"

#include "friendly_logging/logging.h"

namespace frontier_explorer
{

class FrontierExplorerNode : public rclcpp::Node
{
public:
    //  action起别名后面方便用
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit FrontierExplorerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

    // init
    void declare_params();
    void create_interfaces();

    // 回调callback
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void explore_timer_callback();

    bool update_robot_grid_position();

    void send_navigation_goal(const GridCell & goal_cell
            ,const geometry_msgs::msg::PoseStamped & pose);

    void goal_response_callback(
            const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /// @brief 控制面部分
    void publish_state();
        std::string state_to_string() const;

    void handle_start(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handle_stop(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void set_state(ExplorationState new_state, const std::string & detail = {});
    ExplorationState get_state() const;
    std::string state_to_string(ExplorationState state) const;
    std::string state_detail() const;

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr explore_timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    std::optional<GridCell> robot_grid_;
    std::optional<GridCell> current_goal_grid_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;

    FrontierDetector detector_;
    FrontierSelector selector_;

    bool is_navigating_{false};

    double explore_period_sec_{3.0};
    int obstacle_search_radius_cells_{2};
    int min_frontier_cluster_size_{5};
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    std::string selection_strategy_{"nearest"};
    FrontierScoringWeights frontier_decision_weights_{};
    int candidate_unknown_margin_cells_{2};
    double candidate_max_unknown_ratio_{0.4};
    std::chrono::milliseconds map_stale_timeout_{std::chrono::milliseconds(5000)};
    std::size_t max_frontier_failures_{3};
    double edge_tolerance_m_{0.3};
    
    /// @note:后续可以设计为多机控制
    std::atomic<ExplorationState> state_{ExplorationState::IDLE};
    mutable std::mutex state_mutex_;
    std::string state_detail_;

    std::optional<rclcpp::Time> last_progress_time_;
    double last_progress_distance_{0.0};
    double initial_goal_distance_{0.0};
    std::atomic<float> goal_progress_{0.0f};
    rclcpp::Time last_map_update_time_;
    std::size_t consecutive_frontier_failures_{0};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Publisher<robot_interfaces::msg::ExplorationState>::SharedPtr state_pub_;
};



}  // namespace frontier_explorer
