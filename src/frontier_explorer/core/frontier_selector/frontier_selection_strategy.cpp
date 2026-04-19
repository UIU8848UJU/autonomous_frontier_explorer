#include "frontier_selection_strategy.hpp"

#include <limits>

namespace frontier_explorer
{

std::optional<GridCell> NearestFrontierSelectionStrategy::select_goal(
    const std::vector<FrontierSelectionCandidate> & candidates) const
{
    if (candidates.empty()) {
        return std::nullopt;
    }

    double best_distance = std::numeric_limits<double>::max();
    std::optional<GridCell> best_goal;
    for (const auto & candidate : candidates) {
        if (candidate.distance_m < best_distance) {
            best_distance = candidate.distance_m;
            best_goal = candidate.goal;
        }
    }

    return best_goal;
}

const char * NearestFrontierSelectionStrategy::name() const
{
    return "nearest";
}

std::optional<GridCell> LargestClusterFrontierSelectionStrategy::select_goal(
    const std::vector<FrontierSelectionCandidate> & candidates) const
{
    if (candidates.empty()) {
        return std::nullopt;
    }

    const FrontierSelectionCandidate * best_candidate = nullptr;
    for (const auto & candidate : candidates) {
        if (best_candidate == nullptr ||
            candidate.cluster_size > best_candidate->cluster_size ||
            (candidate.cluster_size == best_candidate->cluster_size &&
            candidate.distance_m < best_candidate->distance_m))
        {
            best_candidate = &candidate;
        }
    }

    return best_candidate == nullptr ?
        std::nullopt :
        std::optional<GridCell>(best_candidate->goal);
}

const char * LargestClusterFrontierSelectionStrategy::name() const
{
    return "largest_cluster";
}

std::unique_ptr<IFrontierSelectionStrategy> create_frontier_selection_strategy(
    const std::string & strategy_name)
{
    if (strategy_name == "largest_cluster") {
        return std::make_unique<LargestClusterFrontierSelectionStrategy>();
    }

    return std::make_unique<NearestFrontierSelectionStrategy>();
}

}  // namespace frontier_explorer
