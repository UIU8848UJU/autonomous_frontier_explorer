#include "frontier_strategy_core/scoring/components/retry_penalty_score.hpp"

#include <algorithm>

namespace frontier_strategy
{

RetryPenaltyScore::RetryPenaltyScore(int max_retry_count)
: max_retry_count_(std::max(1, max_retry_count))
{
}

double RetryPenaltyScore::score(const FrontierCandidate & candidate) const
{
    const double normalized =
        static_cast<double>(std::max(0, candidate.retry_count)) /
        static_cast<double>(max_retry_count_);
    return std::clamp(normalized, 0.0, 1.0);
}

}  // namespace frontier_strategy
