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
#include "types.h"

namespace frontier_explorer
{

// 统一打分器：调用各个独立 score component，并合成最终加权总分。
// 本类持有策略权重；component 不持有权重，只根据 FrontierCandidate
// 中的事实数据计算归一化分值或惩罚值。
class FrontierScorer
{
public:
    explicit FrontierScorer(
        FrontierScoringWeights weights = FrontierScoringWeights{},
        int max_retry_count = 2);

    std::vector<ScoredFrontierCandidate> score_candidates(
        const std::vector<FrontierCandidate> & candidates,
        const std::optional<GridCell> & last_goal) const;

    const FrontierScoringWeights & weights() const;

private:
    // 正向分项累加，惩罚分项扣减。
    double compute_total_score(const ScoredFrontierCandidate & scored) const;

private:
    FrontierScoringWeights weights_{};
    
    DistanceScore distance_score_;
    ClusterSizeScore cluster_size_score_;
    ClearanceScore clearance_score_;
    RetryPenaltyScore retry_penalty_score_;
    UnknownRiskPenaltyScore unknown_risk_penalty_score_;
    InformationGainScore information_gain_score_;
};

}  // namespace frontier_explorer
