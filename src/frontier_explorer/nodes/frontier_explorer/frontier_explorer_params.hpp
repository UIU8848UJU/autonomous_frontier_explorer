#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include "frontier_decision_types.hpp"

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
    /// @brief: 用于 unknown frontier 检测的 OccupancyGrid topic
    std::string map_topic{"/map"};
    /// @brief: 用于目标安全检查和 clearance 查询的 global costmap topic
    std::string global_costmap_topic{"/global_costmap/costmap"};
    /// @brief: 是否优先使用 global costmap 做目标安全检查
    bool use_global_costmap_for_safety{true};
};

// pruner 参数：只描述 frontier 硬过滤和候选修复需要的阈值。
struct FrontierPrunerConfig
{
    double min_goal_distance_m{0.45};
    std::size_t min_cluster_size{1U};
    int candidate_unknown_margin_cells{2};
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
