#include "score_components/information_gain_score.hpp"

#include <algorithm>

namespace frontier_explorer
{

double InformationGainScore::score(const FrontierCandidate & candidate) const
{
    return std::clamp(candidate.unknown_ratio, 0.0, 1.0);
}

}  // namespace frontier_explorer
