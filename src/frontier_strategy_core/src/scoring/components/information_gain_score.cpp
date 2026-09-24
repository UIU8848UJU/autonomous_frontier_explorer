#include "frontier_strategy_core/scoring/components/information_gain_score.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_strategy
{
namespace
{
constexpr double kInsetGoalPenalty = 0.95;
constexpr double kFallbackGoalPenalty = 0.9;
}

InformationGainScore::InformationGainScore(double saturation_area_m2)
: saturation_area_m2_(std::max(1e-6, saturation_area_m2))
{
}

double InformationGainScore::score(const FrontierCandidate & candidate) const
{
    if (candidate.cluster_size == 0U) {
        return 0.0;
    }

    if (!candidate.information_gain_valid || candidate.information_gain <= 0.0) {
        return 0.0;
    }

    const double visible_gain = 1.0 - std::exp(
        -candidate.information_gain / saturation_area_m2_);

    double quality_factor = 1.0;
    if (candidate.used_fallback) {
        quality_factor *= kFallbackGoalPenalty;
    }
    if (candidate.goal_inset_applied) {
        quality_factor *= kInsetGoalPenalty;
    }

    return std::clamp(visible_gain * quality_factor, 0.0, 1.0);
}

}  // namespace frontier_strategy
