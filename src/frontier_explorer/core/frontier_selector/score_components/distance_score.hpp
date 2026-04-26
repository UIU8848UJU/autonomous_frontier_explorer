#pragma once

#include "frontier_decision_types.hpp"

namespace frontier_explorer
{

// 距离分：候选点越近，分数越高。
// 输出基于当前候选批次归一化到 [0, 1]，权重由 FrontierScorer 统一处理。
class DistanceScore
{
public:
    double score(
        const FrontierCandidate & candidate,
        double min_distance_m,
        double max_distance_m) const;
};

}  // namespace frontier_explorer
