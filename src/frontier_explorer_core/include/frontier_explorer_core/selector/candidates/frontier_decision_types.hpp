#pragma once

#include <cstddef>

#include "frontier_explorer_core/types/frontier_types.hpp"

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
    bool goal_inset_applied{false};
    bool reachability_checked{false};
    bool reachable{true};
    double path_length_m{0.0};
};

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

}  // namespace frontier_explorer
