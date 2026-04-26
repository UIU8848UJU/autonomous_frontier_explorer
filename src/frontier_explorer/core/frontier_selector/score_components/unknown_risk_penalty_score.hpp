#pragma once

#include "frontier_decision_types.hpp"

namespace frontier_explorer
{

// unknown risk 惩罚分：候选点邻域内 unknown 比例越高，惩罚越高。
// 组件本身不做硬过滤，FrontierScorer 会根据权重从总分中扣减。
class UnknownRiskPenaltyScore
{
public:
    explicit UnknownRiskPenaltyScore(double risk_threshold = 0.4);

    double score(const FrontierCandidate & candidate) const;

private:
    double risk_threshold_{0.4};
};

}  // namespace frontier_explorer
