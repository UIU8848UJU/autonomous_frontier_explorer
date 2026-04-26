#include "score_components/clearance_score.hpp"

#include <algorithm>

namespace frontier_explorer
{

double ClearanceScore::score(
    const FrontierCandidate & candidate,
    double max_clearance_m) const
{
    if (max_clearance_m <= 0.0) {
        return 0.0;
    }

    return std::clamp(candidate.clearance_m / max_clearance_m, 0.0, 1.0);
}

}  // namespace frontier_explorer
