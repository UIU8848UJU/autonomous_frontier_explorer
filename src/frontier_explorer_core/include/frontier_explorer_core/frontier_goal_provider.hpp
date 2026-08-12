#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "frontier_explorer_core/costmap/costmap_adapter.hpp"
#include "frontier_explorer_core/detector/frontier_detector.hpp"
#include "frontier_explorer_core/reachability/frontier_reachability_checker.hpp"
#include "frontier_explorer_core/selector/candidates/frontier_decision_types.hpp"
#include "frontier_explorer_core/selector/frontier_selector.hpp"
#include "frontier_explorer_core/types/frontier_types.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_explorer_core/types/frontier_explorer_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace frontier_explorer
{

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

/// @brief: 单个 frontier 候选目标的服务输出数据
struct FrontierCandidateResult
{
    geometry_msgs::msg::PoseStamped goal;
    float score{0.0F};
    float distance_m{0.0F};
    float clearance_m{0.0F};
    float unknown_ratio{0.0F};
    uint32_t cluster_size{0U};
    uint32_t retry_count{0U};
    bool used_fallback{false};
    bool goal_inset_applied{false};
    bool reachability_checked{false};
    bool reachable{true};
    float path_length_m{0.0F};
    GridCell goal_cell;
};

/// @brief: 计算 frontier 候选列表的结果结构
struct FrontierCandidatesResult
{
    bool success{false};
    std::vector<FrontierCandidateResult> candidates;
    uint16_t reason_code{0};
    std::string reason_text;
    uint32_t raw_frontier_count{0U};
    uint32_t candidate_count{0U};
    uint32_t blacklist_count{0U};
    bool exploration_complete{false};
    bool recoverable{false};
    ExplorationState state{ExplorationState::RUNNING};
    std::string state_detail;
    FrontierGoalVisualization visualization;
};

/// @brief: 标记 frontier 导航失败后的策略更新结果
struct FrontierFailureResult
{
    bool success{false};
    uint32_t retry_count{0U};
    bool blacklisted{false};
    std::string message;
    ExplorationState state{ExplorationState::RUNNING};
    std::string state_detail;
    std::vector<GridCell> blacklisted_goals;
};

/// @brief: 纯 C++ frontier 目标提供器，负责检测、过滤、打分、选择和 retry/blacklist 策略状态
class FrontierGoalProvider
{
public:
    /// @brief: 构造 frontier 目标提供器
    /// @param logger ROS2 日志器，仅用于能力层日志
    explicit FrontierGoalProvider(const rclcpp::Logger & logger);

    /// @brief: 应用参数并重建 detector/selector
    /// @param params frontier explorer 参数
    void configure(const FrontierExplorerParams & params);

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

    /// @brief: 判断栅格是否接近地图边界
    /// @param cell 待判断栅格
    /// @param tolerance_m 边界距离阈值，单位 m
    /// @return: true 表示接近地图边界
    bool near_map_edge(const GridCell & cell, double tolerance_m) const;

private:
    rclcpp::Logger logger_;
    FrontierExplorerParams params_;
    CostmapAdapter map_costmap_;
    CostmapAdapter global_costmap_;
    FrontierDetector detector_;
    FrontierSelector selector_;
    std::shared_ptr<FrontierReachabilityChecker> reachability_checker_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    std::optional<geometry_msgs::msg::PoseStamped> robot_pose_;
    std::optional<GridCell> robot_grid_;
    rclcpp::Time last_map_update_time_;
    std::size_t consecutive_frontier_failures_{0U};
};

}  // namespace frontier_explorer
