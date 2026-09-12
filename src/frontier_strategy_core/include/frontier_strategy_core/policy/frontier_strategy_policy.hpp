#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include "exploration_core/policy/exploration_policy.hpp"
#include "frontier_strategy_core/detector/frontier_detector.hpp"
#include "frontier_strategy_core/scoring/frontier_scoring_weights.hpp"
#include "frontier_strategy_core/selector/filters/frontier_pruner.hpp"
#include "frontier_strategy_core/selector/frontier_selection_policy.hpp"

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
};

/// @brief 使用 Frontier 算法实现探索行为的策略。
///
/// 这个类明确表达“Frontier 是策略、探索是行为”：它实现 exploration_core 的策略接口，
/// 但不把探索执行器、ROS 通信或 Nav2 类型带进算法核心。
class FrontierStrategyPolicy final : public exploration_core::IExplorationPolicy
{
public:
    explicit FrontierStrategyPolicy(
        FrontierStrategyPolicyConfig config = FrontierStrategyPolicyConfig{});

    void reset() override;
    exploration_core::ExplorationDecision decide(
        const exploration_core::ExplorationObservation & observation) override;

    /// 使用适配层提供的安全和可达性回调完成一次完整策略评估。
    FrontierStrategyEvaluation evaluate(
        const grid_map_core::GridMap & map,
        const GridCell & robot_cell,
        const FrontierPruningEnvironment & environment,
        const FrontierSelectionPolicy::ReachabilityCheck & reachability_check = {});

    void on_outcome(const exploration_core::ExplorationOutcome & outcome) override;

    void mark_goal_failed(const GridCell & goal);
    void mark_goal_succeeded(const GridCell & goal);
    std::vector<GridCell> blacklisted_goals() const;
    int retry_count_for_goal(const GridCell & goal) const;
    bool is_goal_blacklisted(const GridCell & goal) const;
    std::size_t clear_blacklist();

private:
    FrontierStrategyPolicyConfig config_;
    FrontierDetector detector_;
    FrontierPruner pruner_;
    FrontierSelectionPolicy selection_policy_;
};

}  // namespace frontier_strategy

