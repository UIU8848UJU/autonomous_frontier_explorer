#pragma once

#include "frontier_strategy_core/selector/candidates/frontier_decision_types.hpp"

namespace frontier_strategy
{

// information gain 分：根据可见未知面积估计候选的信息收益。
// 组件本身不乘权重；enable_information_gain_score 和
// weight_information_gain 只在 FrontierScorer 中生效。
class InformationGainScore
{
public:
    explicit InformationGainScore(double saturation_area_m2 = 1.0);

    double score(const FrontierCandidate & candidate) const;

private:
    double saturation_area_m2_{1.0};
};

}  // namespace frontier_strategy
