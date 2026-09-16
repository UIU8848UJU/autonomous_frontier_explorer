#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/tree_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/navigate_to_pose.hpp"
#include "robot_interfaces/msg/frontier_candidate.hpp"
#include "robot_interfaces/srv/check_goal_feasibility.hpp"
#include "robot_interfaces/srv/get_frontier_candidates.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"

namespace exploration
{

/// @brief: BT 黑板中保存 ExplorationBtContext 的键名
constexpr const char * kExplorationBtContextBlackboardKey = "exploration_bt_context";

/// @brief: 探索 BT 节点共享运行上下文，集中持有 ROS 客户端、当前目标和流程标志
struct ExplorationBtContext
{
    using NavigateToPose = robot_interfaces::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /// @brief: BT 缓存直接使用 ROS 公共候选消息，避免再次定义平行数据模型。
    using FrontierCandidate = robot_interfaces::msg::FrontierCandidate;

    /// @brief 可行性服务结果的短期缓存；缓存只在同一地图和机器人起点区域内复用。
    struct CachedFeasibilityResult
    {
        bool success{false};
        bool feasible{false};
        bool reachable{false};
        bool footprint_valid{false};
        bool recoverable{false};
        uint16_t result_code{0U};
        std::string message;
        float path_length_m{0.0F};
        float footprint_cost{0.0F};
        rclcpp::Time created_at;
    };

    rclcpp::Node * node{nullptr};
    rclcpp::Logger logger{rclcpp::get_logger("exploration.bt")};
    rclcpp::Client<robot_interfaces::srv::GetFrontierCandidates>::SharedPtr get_candidates_client;
    rclcpp::Client<robot_interfaces::srv::MarkFrontierFailed>::SharedPtr mark_failed_client;
    rclcpp::Client<robot_interfaces::srv::CheckGoalFeasibility>::SharedPtr feasibility_client;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;

    mutable std::mutex mutex;
    std::optional<geometry_msgs::msg::PoseStamped> current_goal;
    std::vector<FrontierCandidate> frontier_candidates;
    std::vector<FrontierCandidate> prefetched_candidates;
    uint64_t latest_map_revision{0U};
    uint64_t latest_costmap_revision{0U};
    // 每次导航开始都递增；导航移动后旧起点区域的缓存不能继续使用。
    uint64_t feasibility_start_region_revision{0U};
    std::unordered_map<std::string, CachedFeasibilityResult> feasibility_cache;
    uint64_t prefetched_map_revision{0U};
    rclcpp::Time prefetched_generated_at;
    bool exploration_complete{false};
    bool navigation_failed{false};
    bool navigation_active{false};
    bool navigation_finished{false};
    bool navigation_result_success{false};
    bool prefetch_ready{false};
    bool prefetched_exploration_complete{false};
    bool stop_requested{false};

    double service_retry_delay_sec{2.0};
    uint32_t max_frontier_candidates{8U};
    uint32_t feasibility_top_k{3U};
    uint32_t max_feasibility_recoverable_retries{2U};
    bool enable_candidate_prefetch{true};
    double prefetch_max_age_sec{1.0};
    bool feasibility_cache_enabled{true};
    double feasibility_cache_ttl_sec{1.0};
    double feasibility_cache_region_size_m{0.5};
    std::string feasibility_planner_id;
    bool enable_active_goal_replacement{false};
    double goal_switch_min_utility_gain{0.25};
    double goal_min_hold_duration_sec{2.0};
    uint32_t max_goal_switches_per_navigation{1U};
    rclcpp::Time navigation_started_at;
    double navigation_distance_remaining{std::numeric_limits<double>::infinity()};
    double active_goal_reached_tolerance_m{0.05};
    uint32_t goal_switch_count{0U};
    std::optional<FrontierCandidate> replacement_candidate;
    double feasible_path_length_weight{0.6};
    std::string last_detail{"IDLE"};

    /// @brief: 获取当前 ROS 时间
    /// @return: 当前节点时间；节点不可用时返回系统默认 clock 时间
    rclcpp::Time now() const;
};

/// @brief: 从 BT 黑板读取探索 BT 共享上下文
/// @param config BT 节点配置
/// @return: 探索 BT 共享上下文
std::shared_ptr<ExplorationBtContext> get_exploration_bt_context(
    const BT::NodeConfiguration & config);

}  // 命名空间 exploration
