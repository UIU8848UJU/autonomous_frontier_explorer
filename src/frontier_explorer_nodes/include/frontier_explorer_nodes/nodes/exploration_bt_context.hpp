#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/tree_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/navigate_to_pose.hpp"
#include "robot_interfaces/srv/check_goal_feasibility.hpp"
#include "robot_interfaces/srv/get_frontier_candidates.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"

namespace frontier_explorer
{

/// @brief: BT blackboard 中保存 ExplorationBtContext 的键名
constexpr const char * kExplorationBtContextBlackboardKey = "exploration_bt_context";

/// @brief: Exploration BT 节点共享运行上下文，集中持有 ROS client、当前目标和流程标志
struct ExplorationBtContext
{
    using NavigateToPose = robot_interfaces::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /// @brief: BT 层缓存的 frontier 候选目标
    struct FrontierCandidate
    {
        geometry_msgs::msg::PoseStamped goal;
        float score{0.0F};
        float distance_m{0.0F};
        float clearance_m{0.0F};
        float unknown_ratio{0.0F};
        uint32_t cluster_size{0U};
        uint32_t retry_count{0U};
        bool reachable{false};
        bool reachability_checked{false};
        bool feasible{false};
        bool footprint_valid{false};
        float path_length_m{0.0F};
        float footprint_cost{0.0F};
        std::string feasibility_detail;
    };

    rclcpp::Node * node{nullptr};
    rclcpp::Logger logger{rclcpp::get_logger("frontier_explorer.bt")};
    rclcpp::Client<robot_interfaces::srv::GetFrontierCandidates>::SharedPtr get_candidates_client;
    rclcpp::Client<robot_interfaces::srv::MarkFrontierFailed>::SharedPtr mark_failed_client;
    rclcpp::Client<robot_interfaces::srv::CheckGoalFeasibility>::SharedPtr feasibility_client;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;

    mutable std::mutex mutex;
    std::optional<geometry_msgs::msg::PoseStamped> current_goal;
    std::vector<FrontierCandidate> frontier_candidates;
    bool exploration_complete{false};
    bool navigation_failed{false};
    bool stop_requested{false};

    double service_retry_delay_sec{2.0};
    uint32_t max_frontier_candidates{8U};
    uint32_t max_feasibility_recoverable_retries{2U};
    double feasible_path_length_weight{0.6};
    std::string last_detail{"IDLE"};

    /// @brief: 获取当前 ROS 时间
    /// @return: 当前节点时间；节点不可用时返回系统默认 clock 时间
    rclcpp::Time now() const;
};

/// @brief: 从 BT blackboard 读取 Exploration BT 共享上下文
/// @param config BT 节点配置
/// @return: Exploration BT 共享上下文
std::shared_ptr<ExplorationBtContext> get_exploration_bt_context(
    const BT::NodeConfiguration & config);

}  // namespace frontier_explorer
