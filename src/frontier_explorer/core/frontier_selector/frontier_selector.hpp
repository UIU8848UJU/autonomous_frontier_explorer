#pragma once

#include <optional>
#include <cstddef>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_decision_types.hpp"
#include "frontier_pruner.hpp"
#include "frontier_scorer.hpp"
#include "frontier_selection_state.hpp"
#include "types.h"

namespace frontier_explorer
{

// frontier 决策编排器。
//
// 职责边界：
// - 调用 FrontierPruner 做硬过滤和候选修复；
// - 调用 FrontierScorer 给候选点打分；
// - 选择 total_score 最高的候选点；
// - 维护跨周期状态，例如 last_goal、失败次数、黑名单等。
//
// 不负责：
// - Nav2 action 发送；
// - 地图 frontier 检测；
// - 具体 score component 的分项计算。
class FrontierSelector
{
public:
    // 主构造函数：使用 YAML 读取出的权重配置驱动打分策略。
    FrontierSelector(
        double min_goal_distance_m,
        int max_retry_count = 2,
        const FrontierScoringWeights & scoring_weights = FrontierScoringWeights{},
        std::size_t min_cluster_size = 1U,
        int max_cluster_retry_count = 3,
        int candidate_unknown_margin_cells = 2,
        bool defer_small_clusters = true,
        std::size_t small_cluster_size_threshold = 3U);

    // 在给定 OccupancyGrid 时选择最佳 frontier，会启用 unknown margin 等地图约束。
    std::optional<GridCell> choose_best_frontier(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid & map);

    // 记录最近一次发给 Nav2 的目标点，用于避免重复选择。
    void set_last_goal(const GridCell & goal);

    // 记录目标失败；失败次数达到阈值后加入目标黑名单。
    void mark_goal_failed(const GridCell & goal);

    // 目标成功后清理该目标的失败计数和黑名单记录。
    void mark_goal_succeeded(const GridCell & goal);

private:
    // 记录 cluster 层面的失败；连续失败过多的 cluster 会被拉黑。
    void mark_cluster_failed(const GridCell & cluster_id);

    // cluster 成功产生可用目标后，清理其失败记录。
    void mark_cluster_succeeded(const GridCell & cluster_id);

    // 从一个候选池中打分并选择最高分候选。
    std::optional<ScoredFrontierCandidate> choose_best_scored_candidate(
        const std::vector<FrontierCandidate> & candidates) const;

private:
    // 基础硬过滤参数。
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::size_t min_cluster_size_{1U};
    bool defer_small_clusters_{true};
    std::size_t small_cluster_size_threshold_{3U};

    // 决策流水线组件：pruner 负责候选生成，scorer 负责打分。
    FrontierPruner pruner_;
    FrontierScorer scorer_;

    // 长期选择状态，由 selector 持有和更新。
    FrontierSelectionState state_;
};

}  // namespace frontier_explorer
