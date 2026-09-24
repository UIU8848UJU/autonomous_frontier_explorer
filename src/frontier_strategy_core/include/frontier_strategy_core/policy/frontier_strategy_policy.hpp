#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "frontier_strategy_core/detector/frontier_detector.hpp"
#include "frontier_strategy_core/scoring/frontier_ranker.hpp"
#include "frontier_strategy_core/scoring/frontier_scoring_weights.hpp"
#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "frontier_strategy_core/selector/frontier_selection_policy.hpp"
#include "exploration_core/types/exploration_decision.hpp"
#include "exploration_core/types/exploration_outcome.hpp"

namespace frontier_strategy
{

/// @brief Frontier 策略参数，不包含 ROS 参数类型。
struct FrontierStrategyPolicyConfig
{
    int obstacle_search_radius_cells{2};
    double min_goal_distance_m{0.5};
    int max_retry_count{2};
    int max_cluster_retry_count{3};
    std::size_t min_cluster_size{1U};
    int unknown_margin_cells{2};
    int goal_inset_cells{2};
    double max_unknown_ratio{0.4};
    bool defer_small_clusters{true};
    std::size_t small_cluster_size_threshold{3U};
    bool cleanup_enabled{true};
    std::size_t cleanup_min_cluster_size{1U};
    // 收尾阶段允许目标更靠近机器人，以处理机器人附近残留的小区域。
    double cleanup_min_goal_distance_m{0.0};
    // 收尾阶段默认不再向机器人方向内缩，避免把唯一观测点推离残留 frontier。
    int cleanup_goal_inset_cells{0};
    int cleanup_trigger_no_candidate_cycles{3};
    bool cleanup_trigger_only_small_clusters{true};
    double cleanup_max_unknown_ratio{0.4};
    // 信息增益可见性估计使用的传感器量程，单位 m；0 表示显式关闭。
    double information_gain_sensor_range_m{0.0};
    std::vector<double> viewpoint_retreat_distances_m{0.25, 0.4};
    std::vector<double> viewpoint_sample_radii_m{0.35, 0.55};
    double viewpoint_angle_step_deg{30.0};
    double minimum_information_gain_m2{0.0};
    bool require_reachable_goal{false};
    FrontierScoringWeights scoring_weights{};
};

/// 一次策略评估的完整结果，供 ROS 适配层生成服务响应和可视化数据。
struct FrontierStrategyEvaluation
{
    exploration_core::ExplorationDecision decision;
    std::vector<FrontierCluster> clusters;
    std::vector<GridCell> failed_cluster_ids;
    std::vector<ScoredFrontierCandidate> scored_candidates;
    FrontierDecisionDiagnostics diagnostics;
    bool cleanup_mode{false};
    double detection_ms{0.0};
    double pruning_ms{0.0};
    double ranking_ms{0.0};
};

/// @brief 使用 Frontier 算法完成候选目标检测、筛选和排序的策略组合器。
///
/// 该类只编排 Frontier 算法和纯 C++ 约束，不负责探索执行器、ROS 通信或 Nav2 类型。
class FrontierStrategyPolicy final
{
public:
    explicit FrontierStrategyPolicy(
        FrontierStrategyPolicyConfig config = FrontierStrategyPolicyConfig{},
        std::shared_ptr<IFrontierRanker> ranker = {});

    void reset();

    /// 使用适配层提供的安全和可达性回调完成一次完整策略评估。
    FrontierStrategyEvaluation evaluate(
        const grid_map_core::GridMap & map,
        const GridCell & robot_cell,
        const FrontierPruningEnvironment & environment,
        const FrontierSelectionPolicy::ReachabilityCheck & reachability_check = {});

    void on_outcome(const exploration_core::ExplorationOutcome & outcome);

    void mark_goal_failed(const GridCell & goal);
    void mark_goal_succeeded(const GridCell & goal);
    std::vector<GridCell> blacklisted_goals() const;
    int retry_count_for_goal(const GridCell & goal) const;
    bool is_goal_blacklisted(const GridCell & goal) const;
    std::size_t clear_blacklist();

private:
    bool should_enter_cleanup(const std::vector<FrontierCluster> & clusters) const;

    FrontierStrategyPolicyConfig config_;
    FrontierDetector detector_;
    FrontierPruner pruner_;
    FrontierSelectionPolicy selection_policy_;
    bool cleanup_mode_{false};
    int no_candidate_cycles_{0};
};

}  // namespace frontier_strategy

