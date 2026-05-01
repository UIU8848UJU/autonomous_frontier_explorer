#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/srv/get_next_frontier_goal.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"

namespace frontier_explorer
{

/// @brief: Exploration BT 节点共享运行上下文，集中持有 ROS client、当前目标和流程标志
struct ExplorationBtContext
{
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp::Node * node{nullptr};
    rclcpp::Logger logger{rclcpp::get_logger("frontier_explorer.bt")};
    rclcpp::Client<robot_interfaces::srv::GetNextFrontierGoal>::SharedPtr get_next_client;
    rclcpp::Client<robot_interfaces::srv::MarkFrontierFailed>::SharedPtr mark_failed_client;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;

    mutable std::mutex mutex;
    std::optional<geometry_msgs::msg::PoseStamped> current_goal;
    bool exploration_complete{false};
    bool navigation_failed{false};
    bool stop_requested{false};

    double service_retry_delay_sec{2.0};
    std::string last_detail{"IDLE"};

    /// @brief: 获取当前 ROS 时间
    /// @return: 当前节点时间；节点不可用时返回系统默认 clock 时间
    rclcpp::Time now() const;
};

}  // namespace frontier_explorer
