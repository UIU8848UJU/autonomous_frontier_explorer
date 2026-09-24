#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "frontier_strategy_core/scoring/frontier_scoring_weights.hpp"

namespace frontier_strategy
{

// 节点运行时参数：控制定时器、探测器基础参数和异常判定阈值。
struct FrontierStrategyRuntimeConfig
{
    /// @brief: frontier cell 与障碍物的最小安全搜索半径，单位 cell
    int obstacle_search_radius_cells{1};
    /// @brief: 最小 frontier cluster 尺寸
    int min_frontier_cluster_size{1};
    /// @brief: 地图更新超时时间
    std::chrono::milliseconds map_stale_timeout{std::chrono::milliseconds(5000)};
    /// @brief: 连续 frontier 选择失败阈值
    int max_frontier_failures{3};
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
    /// @brief: 连续多少次稳定的无 frontier 结果才允许判定完成
    int stable_no_frontier_cycles{3};
    /// @brief: 是否启用收尾模式
    bool cleanup_enabled{true};
    /// @brief: 连续无候选多少轮后进入收尾模式
    int cleanup_trigger_no_candidate_cycles{3};
    /// @brief: 是否在只剩小 cluster 时直接进入收尾模式
    bool cleanup_trigger_only_small_clusters{true};
};

// pruner 参数：只描述 frontier 硬过滤和候选修复需要的阈值。
struct FrontierPrunerConfig
{
    double min_goal_distance_m{0.45};
    std::size_t min_cluster_size{1U};
    std::size_t cleanup_min_cluster_size{1U};
    /// @brief 收尾阶段允许的最小目标距离，单位 m。
    double cleanup_min_goal_distance_m{0.0};
    /// @brief 收尾阶段目标向机器人方向内缩的栅格数。
    int cleanup_goal_inset_cells{0};
    int candidate_unknown_margin_cells{2};
    int candidate_goal_inset_cells{2};
    double candidate_max_unknown_ratio{0.4};
    double cleanup_candidate_max_unknown_ratio{0.4};
    /// @brief 是否启用真实可见未知面积估计与对应评分。
    bool enable_information_gain{true};
    /// @brief 信息增益估计使用的传感器量程，单位 m。
    double information_gain_sensor_range_m{3.0};
    /// @brief 远离 frontier 的候选观测距离，单位 m。
    std::vector<double> viewpoint_retreat_distances_m{0.25, 0.4};
    /// @brief 围绕 frontier 采样的候选观测半径，单位 m。
    std::vector<double> viewpoint_sample_radii_m{0.35, 0.55};
    /// @brief 环形候选观测位姿的角度间隔，单位 degree。
    double viewpoint_angle_step_deg{30.0};
    /// @brief 候选至少需要看到的未知面积，单位 m²；0 表示不设该门槛。
    double minimum_information_gain_m2{0.0};
    bool enable_footprint_filter{true};
    bool allow_unknown_footprint{false};
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

// 地图分辨率自适应参数：只换算几何尺度，不动态放宽安全门禁。
struct FrontierMapAdaptationConfig
{
    bool enabled{true};
    /// @brief frontier 周围障碍搜索半径，单位 m
    double obstacle_clearance_m{0.05};
    /// @brief 正常 frontier cluster 的最小近似长度，单位 m
    double min_frontier_length_m{0.10};
    /// @brief 小 frontier cluster 的近似长度阈值，单位 m
    double small_frontier_length_m{0.25};
    /// @brief 候选点周围 unknown 比例统计半径，单位 m
    double candidate_unknown_margin_m{0.10};
    /// @brief 正常候选点向已知区域内缩距离，单位 m
    double candidate_goal_inset_m{0.15};
    /// @brief 收尾候选点向已知区域内缩距离，单位 m
    double cleanup_goal_inset_m{0.0};
};

struct FrontierStrategyParams
{
    FrontierStrategyRuntimeConfig runtime{};
    FrontierPrunerConfig pruner{};
    FrontierScoringConfig scorer{};
    FrontierSelectionPolicyConfig selection{};
    FrontierMapAdaptationConfig map_adaptation{};
};

}  // 命名空间 frontier_strategy
