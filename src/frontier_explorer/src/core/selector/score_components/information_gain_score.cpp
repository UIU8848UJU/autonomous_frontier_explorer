#include "core/selector/scoring/components/information_gain_score.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_explorer
{
namespace
{
constexpr double kUsefulUnknownRatio = 0.35;
constexpr double kClusterSizeSaturationCells = 40.0;
constexpr double kMinimumClusterContribution = 0.25;
constexpr double kInsetGoalPenalty = 0.95;
constexpr double kFallbackGoalPenalty = 0.9;
}

double InformationGainScore::score(const FrontierCandidate & candidate) const
{
    if (candidate.cluster_size == 0U) {
        return 0.0;
    }

    const double unknown_density = std::clamp(
        candidate.unknown_ratio / kUsefulUnknownRatio,
        0.0,
        1.0);
    if (unknown_density <= 0.0) {
        return 0.0;
    }

    const double cluster_size = static_cast<double>(candidate.cluster_size);
    const double cluster_gain = 1.0 - std::exp(
        -cluster_size / kClusterSizeSaturationCells);
    const double cluster_factor = std::clamp(
        kMinimumClusterContribution +
        (1.0 - kMinimumClusterContribution) * cluster_gain,
        0.0,
        1.0);

    double quality_factor = 1.0;
    if (candidate.used_fallback) {
        quality_factor *= kFallbackGoalPenalty;
    }
    if (candidate.goal_inset_applied) {
        quality_factor *= kInsetGoalPenalty;
    }

    return std::clamp(unknown_density * cluster_factor * quality_factor, 0.0, 1.0);
}

}  // namespace frontier_explorer
