#pragma once

#include "frontier_strategy_core/selector/candidates/frontier_decision_types.hpp"

namespace frontier_strategy
{

// information gain 分：用局部 unknown 密度和 frontier cluster 规模估计候选的信息收益。
// 组件本身不乘权重；enable_information_gain_score 和
// weight_information_gain 只在 FrontierScorer 中生效。
class InformationGainScore
{
public:
    double score(const FrontierCandidate & candidate) const;
};

}  // namespace frontier_strategy
