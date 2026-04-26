#pragma once

#include <cstddef>

#include "types.h"

namespace frontier_explorer
{

// FrontierPruner 产出的候选目标，只保存打分需要的事实数据；
// 权重和策略决策统一交给 FrontierScorer 处理。
struct FrontierCandidate
{
    GridCell goal{};
    GridCell cluster_centroid{};
    std::size_t cluster_size{0U};
    double distance_m{0.0};

    int retry_count{0};
    double clearance_m{0.0};
    double unknown_ratio{0.0};
    std::size_t source_cluster_index{0U};
    bool used_fallback{false};
};

// 临时兼容旧调用点的类型别名。
using FrontierSelectionCandidate = FrontierCandidate;

// 打分结果。保留每个分项，方便调试和调权重。
struct ScoredFrontierCandidate
{
    FrontierCandidate candidate{};

    double distance_score{0.0};
    double cluster_size_score{0.0};
    double clearance_score{0.0};
    double revisit_penalty{0.0};
    double retry_penalty{0.0};
    double unknown_risk_penalty{0.0};
    double information_gain_score{0.0};

    double total_score{0.0};
};

// YAML 驱动的打分权重配置。score component 只返回归一化分项值；
// 权重只在 FrontierScorer 中统一应用。
struct FrontierScoringWeights
{
    double weight_distance{1.0};
    double weight_cluster_size{1.0};
    double weight_clearance{0.0};
    double weight_revisit_penalty{1.0};
    double weight_retry_penalty{1.0};
    double weight_unknown_risk_penalty{1.0};
    double weight_information_gain{0.0};
    double unknown_risk_threshold{0.4};

    bool enable_clearance_score{false};
    bool enable_revisit_penalty{false};
    bool enable_unknown_risk_penalty{true};
    bool enable_information_gain_score{false};
};

}  // namespace frontier_explorer
