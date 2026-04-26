#pragma once

#include "frontier_decision_types.hpp"

namespace frontier_explorer
{

// retry 惩罚分：目标失败次数越多，惩罚越高，范围为 [0, 1]。
// FrontierScorer 会乘以 weight_retry_penalty 后从总分中扣减。
class RetryPenaltyScore
{
public:
    explicit RetryPenaltyScore(int max_retry_count);

    double score(const FrontierCandidate & candidate) const;

private:
    int max_retry_count_{1};
};

}  // namespace frontier_explorer
