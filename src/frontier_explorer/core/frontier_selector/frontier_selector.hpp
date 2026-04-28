#pragma once

#include <optional>
#include <cstddef>
#include <vector>

#include "costmap_adapter.hpp"
#include "frontier_decision_types.hpp"
#include "frontier_pruner.hpp"
#include "frontier_scorer.hpp"
#include "frontier_selection_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"

namespace frontier_explorer
{

/// @brief: Frontier 决策编排器，负责硬过滤、打分选择和跨周期状态维护
class FrontierSelector
{
public:
    /// @brief: 构造 FrontierSelector，使用 YAML 读取出的权重配置驱动打分策略
    /// @param min_goal_distance_m 目标与机器人最小距离
    /// @param max_retry_count 单个 goal 最大重试次数
    /// @param scoring_weights 打分权重配置
    /// @param min_cluster_size 最小 cluster 尺寸
    /// @param max_cluster_retry_count 单个 cluster 最大重试次数
    /// @param candidate_unknown_margin_cells 候选点 unknown ratio 统计半径
    /// @param defer_small_clusters 是否延后选择小 cluster
    /// @param small_cluster_size_threshold 小 cluster 判定阈值
    /// @param logger ROS2 日志器
    FrontierSelector(
        double min_goal_distance_m,
        int max_retry_count = 2,
        const FrontierScoringWeights & scoring_weights = FrontierScoringWeights{},
        std::size_t min_cluster_size = 1U,
        int max_cluster_retry_count = 3,
        int candidate_unknown_margin_cells = 2,
        bool defer_small_clusters = true,
        std::size_t small_cluster_size_threshold = 3U,
        const rclcpp::Logger & logger = rclcpp::get_logger("frontier_explorer"));

    /// @brief: 在给定 CostmapAdapter 时选择最佳 frontier，会启用 unknown margin 等地图约束
    /// @param clusters 原始 frontier clusters
    /// @param robot_grid 机器人所在栅格
    /// @param resolution 地图分辨率
    /// @param frontier_costmap frontier 检测来源地图适配器
    /// @param safety_costmap 安全检查来源地图适配器，可为空
    /// @return: 选中的目标栅格；无可用目标时返回 std::nullopt
    std::optional<GridCell> choose_best_frontier(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const CostmapAdapter & frontier_costmap,
        const CostmapAdapter * safety_costmap = nullptr);

    /// @brief: 记录最近一次发给 Nav2 的目标点，用于避免重复选择
    /// @param goal 最近目标栅格
    void set_last_goal(const GridCell & goal);

    /// @brief: 记录目标失败；失败次数达到阈值后加入目标黑名单
    /// @param goal 失败目标栅格
    void mark_goal_failed(const GridCell & goal);

    /// @brief: 目标成功后清理该目标的失败计数和黑名单记录
    /// @param goal 成功目标栅格
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
    rclcpp::Logger logger_;
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
