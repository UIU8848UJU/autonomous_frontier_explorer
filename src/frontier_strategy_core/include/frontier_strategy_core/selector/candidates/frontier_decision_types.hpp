#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "frontier_strategy_core/types/frontier_types.hpp"

namespace frontier_strategy
{

/// @brief 候选生成阶段的稳定拒绝原因，用于诊断门禁瓶颈。
enum class FrontierRejectionReason : std::size_t
{
    MAP_BOUNDARY = 0U,
    CLUSTER_TOO_SMALL,
    GOAL_TOO_CLOSE,
    MAP_CELL_INVALID,
    UNKNOWN_RATIO_TOO_HIGH,
    SAFETY_REJECTED,
    SAME_AS_LAST_GOAL,
    GOAL_RETRY_EXHAUSTED,
    GOAL_BLACKLISTED,
    CLUSTER_RETRY_EXHAUSTED,
    CLUSTER_BLACKLISTED,
    DUPLICATE_GOAL,
    NO_CANDIDATE_GENERATED,
    INFORMATION_GAIN_TOO_LOW,
    COUNT
};

/// @brief 一轮 frontier 决策的统计信息，不依赖 ROS 类型或日志器。
struct FrontierDecisionDiagnostics
{
    std::size_t raw_frontier_cells{0U};
    std::size_t raw_clusters{0U};
    std::size_t generated_candidates{0U};
    std::array<std::size_t, static_cast<std::size_t>(FrontierRejectionReason::COUNT)>
        rejection_counts{};

    void record_rejection(FrontierRejectionReason reason)
    {
        ++rejection_counts[static_cast<std::size_t>(reason)];
    }

    std::size_t rejection_count(FrontierRejectionReason reason) const
    {
        return rejection_counts[static_cast<std::size_t>(reason)];
    }
};

inline const char * frontier_rejection_reason_name(FrontierRejectionReason reason)
{
    switch (reason) {
        case FrontierRejectionReason::MAP_BOUNDARY: return "MAP_BOUNDARY";
        case FrontierRejectionReason::CLUSTER_TOO_SMALL: return "CLUSTER_TOO_SMALL";
        case FrontierRejectionReason::GOAL_TOO_CLOSE: return "GOAL_TOO_CLOSE";
        case FrontierRejectionReason::MAP_CELL_INVALID: return "MAP_CELL_INVALID";
        case FrontierRejectionReason::UNKNOWN_RATIO_TOO_HIGH: return "UNKNOWN_RATIO_TOO_HIGH";
        case FrontierRejectionReason::SAFETY_REJECTED: return "SAFETY_REJECTED";
        case FrontierRejectionReason::SAME_AS_LAST_GOAL: return "SAME_AS_LAST_GOAL";
        case FrontierRejectionReason::GOAL_RETRY_EXHAUSTED: return "GOAL_RETRY_EXHAUSTED";
        case FrontierRejectionReason::GOAL_BLACKLISTED: return "GOAL_BLACKLISTED";
        case FrontierRejectionReason::CLUSTER_RETRY_EXHAUSTED: return "CLUSTER_RETRY_EXHAUSTED";
        case FrontierRejectionReason::CLUSTER_BLACKLISTED: return "CLUSTER_BLACKLISTED";
        case FrontierRejectionReason::DUPLICATE_GOAL: return "DUPLICATE_GOAL";
        case FrontierRejectionReason::NO_CANDIDATE_GENERATED: return "NO_CANDIDATE_GENERATED";
        case FrontierRejectionReason::INFORMATION_GAIN_TOO_LOW: return "INFORMATION_GAIN_TOO_LOW";
        case FrontierRejectionReason::COUNT: break;
    }
    return "UNKNOWN";
}

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
    std::string reachability_reason;
    // 由候选观测位姿发出的二维射线预计能看到的未知栅格数。
    double information_gain{0.0};
    // true 表示本轮已完成射线估计；估计值为零时也必须保留零分，而不是退回旧估计。
    bool information_gain_valid{false};
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

}  // namespace frontier_strategy
