#pragma once

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
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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

    void publish_state();
    std::string state_to_string() const;

    void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

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
    
    ExplorationState state_{ExplorationState::IDLE};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    bool running_{true};
};



}  // namespace frontier_explorer