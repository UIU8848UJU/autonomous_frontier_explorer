#pragma once

#include "frontier_decision_types.hpp"

namespace frontier_explorer
{

// clearance 分预留项：用于表达候选点与障碍物的安全距离。
// 默认关闭，直到 FrontierPruner 或地图分析逻辑填充 candidate.clearance_m。
class ClearanceScore
{
public:
    double score(
        const FrontierCandidate & candidate,
        double max_clearance_m) const;
};

}  // namespace frontier_explorer
