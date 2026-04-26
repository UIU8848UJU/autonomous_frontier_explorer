#pragma once

#include <cstddef>

#include "frontier_decision_types.hpp"

namespace frontier_explorer
{

// cluster size 分：frontier cluster 越大，分数越高。
// 输出基于当前候选批次归一化到 [0, 1]。
class ClusterSizeScore
{
public:
    double score(
        const FrontierCandidate & candidate,
        std::size_t min_cluster_size,
        std::size_t max_cluster_size) const;
};

}  // namespace frontier_explorer
