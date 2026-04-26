#include "score_components/distance_score.hpp"

#include <algorithm>

namespace frontier_explorer
{

double DistanceScore::score(
    const FrontierCandidate & candidate,
    double min_distance_m,
    double max_distance_m) const
{
    if (max_distance_m <= min_distance_m) {
        return 1.0;
    }

    const double normalized =
        (candidate.distance_m - min_distance_m) / (max_distance_m - min_distance_m);
    return 1.0 - std::clamp(normalized, 0.0, 1.0);
}

}  // namespace frontier_explorer
