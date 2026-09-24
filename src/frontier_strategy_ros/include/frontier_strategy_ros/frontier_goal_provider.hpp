#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "grid_map_ros/costmap_adapter.hpp"
#include "frontier_strategy_core/policy/frontier_strategy_policy.hpp"
#include "frontier_strategy_ros/reachability/frontier_reachability_checker.hpp"
#include "frontier_strategy_core/selector/candidates/frontier_decision_types.hpp"
#include "frontier_strategy_core/types/frontier_types.hpp"
#include "exploration_core/types/exploration_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_strategy_ros/types/frontier_strategy_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_geometry_core/robot_geometry_provider.hpp"
#include "robot_interfaces/msg/frontier_candidate.hpp"

namespace frontier_strategy
{

using exploration_core::ExplorationStatus;
using CostmapAdapter = grid_map_ros::CostmapAdapter;

/// @brief: frontier marker 发布所需的可视化快照
struct FrontierGoalVisualization
{
    std::vector<FrontierCluster> raw_clusters;
    std::vector<FrontierCluster> rejected_clusters;
    std::vector<FrontierCandidate> candidates;
    std::vector<ScoredFrontierCandidate> scored_candidates;
    std::vector<GridCell> blacklisted_goals;
    std::optional<GridCell> selected_goal;
    std::optional<GridCell> robot_grid;
    bool clear_candidate_markers{false};
};

/// @brief: 计算 frontier 候选列表的结果结构
struct FrontierCandidatesResult
{
    bool success{false};
    std::vector<robot_interfaces::msg::FrontierCandidate> candidates;
    uint16_t reason_code{0};
    std::string reason_text;
    uint32_t raw_frontier_count{0U};
    uint32_t candidate_count{0U};
    uint32_t blacklist_count{0U};
    bool exploration_complete{false};
    bool recoverable{false};
    bool cleanup_mode{false};
    ExplorationStatus state{ExplorationStatus::RUNNING};
    std::string state_detail;
    FrontierDecisionDiagnostics diagnostics;
    double detection_ms{0.0};
    double pruning_ms{0.0};
    double ranking_ms{0.0};
    double total_ms{0.0};
    uint64_t map_revision{0U};
    int stable_no_frontier_cycles{0};
    FrontierGoalVisualization visualization;
};

/// @brief: 标记 frontier 导航失败后的策略更新结果
struct FrontierFailureResult
{
    bool success{false};
    uint32_t retry_count{0U};
    bool blacklisted{false};
    std::string message;
    ExplorationStatus state{ExplorationStatus::RUNNING};
    std::string state_detail;
    std::vector<GridCell> blacklisted_goals;
};

/// @brief: ROS 2 适配层的 frontier 目标提供器，负责消息转换、能力编排和 retry/blacklist 状态
class FrontierGoalProvider
{
public:
    /// @brief: 构造 frontier 目标提供器
    /// @param logger ROS2 日志器，仅用于能力层日志
    explicit FrontierGoalProvider(
        const rclcpp::Logger & logger,
        std::shared_ptr<IFrontierRanker> ranker = {});

    /// @brief: 应用参数、机器人几何来源并重建 detector/selector
    /// @param params frontier 策略参数
    /// @param robot_geometry_provider 机器人碰撞几何来源，不可为空
    void configure(
        const FrontierStrategyParams & params,
        std::shared_ptr<const robot_geometry_core::IRobotGeometryProvider>
        robot_geometry_provider);

    /// @brief: 更新用于 frontier 检测的地图
    /// @param msg OccupancyGrid 地图消息
    /// @param stamp 接收地图时的 ROS 时间
    /// @return: true 表示地图成功进入 costmap adapter
    bool update_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg, const rclcpp::Time & stamp);

    /// @brief: 更新用于安全检查的 global costmap
    /// @param msg OccupancyGrid costmap 消息
    /// @return: true 表示 costmap 成功进入 adapter
    bool update_global_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);

    /// @brief: 更新 map frame 下的机器人位姿
    /// @param pose map frame 下的机器人位姿
    void update_robot_pose(const geometry_msgs::msg::PoseStamped & pose);

    /// @brief: 清空机器人位姿，避免 TF 失败时继续使用过期位姿
    void clear_robot_pose();

    /// @brief: 注入 frontier 可达性检查器
    /// @param checker 可达性检查器，可为空
    void set_reachability_checker(
        const std::shared_ptr<FrontierReachabilityChecker> & checker);

    /// @brief: 计算 frontier 候选目标列表，不触发导航、不更新 last goal
    /// @param now 当前 ROS 时间
    /// @param max_candidates 最多返回的候选数量；0 表示返回全部候选
    /// @return: frontier 候选目标列表结果
    FrontierCandidatesResult compute_frontier_candidates(
        const rclcpp::Time & now,
        std::size_t max_candidates = 0U);

    /// @brief: 记录一个 frontier goal 导航失败事件
    /// @param failed_goal 失败目标世界坐标
    /// @return: retry/blacklist 策略更新结果
    FrontierFailureResult mark_frontier_failed(const geometry_msgs::msg::Point & failed_goal);

    /// @brief: 清空 retry/blacklist 策略状态
    /// @return: 被清理的黑名单条目数量
    std::size_t clear_blacklist();

    /// @brief: 获取 frontier 主 costmap
    /// @return: frontier 主 costmap 只读引用
    const CostmapAdapter & map_costmap() const;

    /// @brief: 获取当前黑名单 goal 列表
    /// @return: 黑名单 goal 列表
    std::vector<GridCell> blacklisted_goals() const;

private:
    /// @brief: 更新机器人栅格坐标
    /// @return: true 表示机器人坐标可用
    bool update_robot_grid_position();

    /// @brief 根据有效地图分辨率刷新 Core 使用的 cell 参数。
    /// @param map_resolution 地图分辨率，单位 m/cell
    void refresh_policy_for_map_resolution(double map_resolution);

    FrontierPruningEnvironment make_pruning_environment(
        const CostmapAdapter & frontier_costmap,
        const CostmapAdapter * safety_costmap) const;

private:
    rclcpp::Logger logger_;
    FrontierStrategyParams base_params_;
    FrontierStrategyParams params_;
    CostmapAdapter map_costmap_;
    CostmapAdapter global_costmap_;
    std::shared_ptr<IFrontierRanker> ranker_;
    FrontierStrategyPolicy policy_;
    std::shared_ptr<FrontierReachabilityChecker> reachability_checker_;
    std::shared_ptr<const robot_geometry_core::IRobotGeometryProvider>
        robot_geometry_provider_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    std::optional<geometry_msgs::msg::PoseStamped> robot_pose_;
    std::optional<GridCell> robot_grid_;
    rclcpp::Time last_map_update_time_;
    uint64_t map_fingerprint_{0U};
    uint64_t map_revision_{0U};
    uint64_t no_frontier_revision_{0U};
    int stable_no_frontier_cycles_{0};
    bool has_map_fingerprint_{false};
    std::optional<double> policy_map_resolution_;
    std::size_t consecutive_frontier_failures_{0U};
};

}  // 命名空间 frontier_strategy
