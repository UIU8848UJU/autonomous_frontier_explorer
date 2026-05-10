#pragma once

#include <cstddef>

#include "core/selector/candidates/frontier_decision_types.hpp"
#include "core/types/frontier_types.hpp"

namespace frontier_explorer
{

/// @brief: 可接近导航目标候选，作为评分器输入，后续可独立于 pruner 输出扩展 ML 特征
struct ApproachGoalCandidate
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

    ApproachGoalCandidate() = default;

    explicit ApproachGoalCandidate(const FrontierCandidate & candidate)
    : goal(candidate.goal),
      cluster_centroid(candidate.cluster_centroid),
      cluster_size(candidate.cluster_size),
      distance_m(candidate.distance_m),
      retry_count(candidate.retry_count),
      clearance_m(candidate.clearance_m),
      unknown_ratio(candidate.unknown_ratio),
      source_cluster_index(candidate.source_cluster_index),
      used_fallback(candidate.used_fallback),
      goal_inset_applied(candidate.goal_inset_applied),
      reachability_checked(candidate.reachability_checked),
      reachable(candidate.reachable),
      path_length_m(candidate.path_length_m)
    {
    }

    FrontierCandidate to_frontier_candidate() const
    {
        return FrontierCandidate{
            goal,
            cluster_centroid,
            cluster_size,
            distance_m,
            retry_count,
            clearance_m,
            unknown_ratio,
            source_cluster_index,
            used_fallback,
            goal_inset_applied,
            reachability_checked,
            reachable,
            path_length_m};
    }
};

}  // namespace frontier_explorer
