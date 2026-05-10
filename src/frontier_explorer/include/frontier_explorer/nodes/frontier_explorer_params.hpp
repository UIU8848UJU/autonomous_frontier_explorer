#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "core/selector/scoring/frontier_scoring_weights.hpp"

namespace frontier_explorer
{

// 节点运行时参数：控制定时器、探测器基础参数和异常判定阈值。
struct ExplorerRuntimeConfig
{
    /// @brief: 探索主循环周期，单位秒
    double explore_period_sec{3.0};
    /// @brief: frontier cell 与障碍物的最小安全搜索半径，单位 cell
    int obstacle_search_radius_cells{1};
    /// @brief: 最小 frontier cluster 尺寸
    int min_frontier_cluster_size{1};
    /// @brief: 地图更新超时时间
    std::chrono::milliseconds map_stale_timeout{std::chrono::milliseconds(5000)};
    /// @brief: 连续 frontier 选择失败阈值
    int max_frontier_failures{3};
    /// @brief: 接近地图边缘的判定距离，单位 m
    double edge_tolerance_m{0.3};
    /// @brief: 单个导航目标认为已经接近到达的距离，单位 m
    double goal_reached_tolerance_m{0.05};
    /// @brief: 用于 unknown frontier 检测的 OccupancyGrid topic
    std::string map_topic{"/map"};
    /// @brief: 用于目标安全检查和 clearance 查询的 global costmap topic
    std::string global_costmap_topic{"/global_costmap/costmap"};
    /// @brief: 是否优先使用 global costmap 做目标安全检查
    bool use_global_costmap_for_safety{true};
    /// @brief: frontier 计算使用的全局坐标系
    std::string global_frame{"map"};
    /// @brief: 机器人底盘坐标系
    std::string robot_base_frame{"base_link"};
    /// @brief: 查询机器人 TF 的超时时间
    std::chrono::milliseconds robot_pose_timeout{std::chrono::milliseconds(200)};
    /// @brief: 是否显示全部候选点；false 时 candidate marker 只显示最终选中目标
    bool show_all_candidate_markers{false};
    /// @brief: 是否使用 Nav2 planner 对候选点做可达性过滤
    bool enable_reachability_filter{true};
    /// @brief: 是否强制要求候选通过可达性检查；false 时不可达只作为诊断和排序提示
    bool require_reachable_goal{false};
    /// @brief: 每轮最多检查多少个评分靠前候选，避免 planner 负载过高
    int max_reachability_checks{6};
    /// @brief: ComputePathToPose action 名称
    std::string compute_path_to_pose_action{"compute_path_to_pose"};
    /// @brief: 等待 ComputePathToPose action server 的超时时间
    std::chrono::milliseconds reachability_server_timeout{std::chrono::milliseconds(200)};
    /// @brief: 单个候选可达性检查超时时间
    std::chrono::milliseconds reachability_check_timeout{std::chrono::milliseconds(500)};
    /// @brief: Nav2 planner_id，空字符串表示使用默认 planner
    std::string reachability_planner_id;
};

// pruner 参数：只描述 frontier 硬过滤和候选修复需要的阈值。
struct FrontierPrunerConfig
{
    double min_goal_distance_m{0.45};
    std::size_t min_cluster_size{1U};
    int candidate_unknown_margin_cells{2};
    int candidate_goal_inset_cells{2};
    double candidate_max_unknown_ratio{0.4};
    bool enable_footprint_filter{true};
    bool allow_unknown_footprint{false};
    double robot_radius{0.1};
    double footprint_padding{0.0};
    int footprint_cost_threshold{static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)};
};

// scorer 参数：只描述打分权重和分项开关。
struct FrontierScoringConfig
{
    FrontierScoringWeights weights{};
};

// selector 策略状态参数：控制 goal / cluster 失败重试策略。
struct FrontierSelectionPolicyConfig
{
    int max_retry_count{2};
    int max_cluster_retry_count{3};
    bool defer_small_clusters{true};
    std::size_t small_cluster_size_threshold{3U};
};

struct FrontierExplorerParams
{
    ExplorerRuntimeConfig runtime{};
    FrontierPrunerConfig pruner{};
    FrontierScoringConfig scorer{};
    FrontierSelectionPolicyConfig selection{};
};

}  // namespace frontier_explorer
