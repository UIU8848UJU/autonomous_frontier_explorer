#include "score_components/unknown_risk_penalty_score.hpp"

#include <algorithm>

namespace frontier_explorer
{

UnknownRiskPenaltyScore::UnknownRiskPenaltyScore(double risk_threshold)
: risk_threshold_(std::clamp(risk_threshold, 0.0, 1.0))
{
}

double UnknownRiskPenaltyScore::score(const FrontierCandidate & candidate) const
{
    const double unknown_ratio = std::clamp(candidate.unknown_ratio, 0.0, 1.0);
    if (unknown_ratio <= risk_threshold_) {
        return 0.0;
    }

    if (risk_threshold_ >= 1.0) {
        return 0.0;
    }

    return std::clamp(
        (unknown_ratio - risk_threshold_) / (1.0 - risk_threshold_),
        0.0,
        1.0);
}

}  // namespace frontier_explorer
