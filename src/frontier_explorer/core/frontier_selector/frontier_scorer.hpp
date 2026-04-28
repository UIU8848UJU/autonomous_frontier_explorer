#pragma once

#include <optional>
#include <vector>

#include "frontier_decision_types.hpp"
#include "score_components/clearance_score.hpp"
#include "score_components/cluster_size_score.hpp"
#include "score_components/distance_score.hpp"
#include "score_components/information_gain_score.hpp"
#include "score_components/retry_penalty_score.hpp"
#include "score_components/unknown_risk_penalty_score.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"

namespace frontier_explorer
{

/// @brief: 统一打分器，调用各 score component 并合成最终加权总分
class FrontierScorer
{
public:
    /// @brief: 构造 FrontierScorer
    /// @param weights 打分权重配置
    /// @param max_retry_count 最大重试次数
    /// @param logger ROS2 日志器
    explicit FrontierScorer(
        FrontierScoringWeights weights = FrontierScoringWeights{},
        int max_retry_count = 2,
        const rclcpp::Logger & logger = rclcpp::get_logger("frontier_explorer"));

    /// @brief: 对候选 frontier 批量打分
    /// @param candidates pruner 产出的候选点
    /// @param last_goal 最近一次目标点
    /// @return: 带分项和总分的候选点列表
    std::vector<ScoredFrontierCandidate> score_candidates(
        const std::vector<FrontierCandidate> & candidates,
        const std::optional<GridCell> & last_goal) const;

    /// @brief: 获取当前打分权重配置
    /// @return: 打分权重只读引用
    const FrontierScoringWeights & weights() const;

private:
    // 正向分项累加，惩罚分项扣减。
    double compute_total_score(const ScoredFrontierCandidate & scored) const;

private:
    rclcpp::Logger logger_;
    FrontierScoringWeights weights_{};
    
    DistanceScore distance_score_;
    ClusterSizeScore cluster_size_score_;
    ClearanceScore clearance_score_;
    RetryPenaltyScore retry_penalty_score_;
    UnknownRiskPenaltyScore unknown_risk_penalty_score_;
    InformationGainScore information_gain_score_;
};

}  // namespace frontier_explorer
