#pragma once

#include "frontier_decision_types.hpp"

namespace frontier_explorer
{

// information gain 分：当前用 candidate.unknown_ratio 作为轻量信息增益代理。
// 组件本身不乘权重；enable_information_gain_score 和 
// weight_information_gain 只在 FrontierScorer 中生效。
class InformationGainScore
{
public:
    double score(const FrontierCandidate & candidate) const;
};

}  // namespace frontier_explorer
