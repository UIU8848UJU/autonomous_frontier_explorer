#include "score_components/cluster_size_score.hpp"

#include <algorithm>

namespace frontier_explorer
{

double ClusterSizeScore::score(
    const FrontierCandidate & candidate,
    std::size_t min_cluster_size,
    std::size_t max_cluster_size) const
{
    if (max_cluster_size <= min_cluster_size) {
        return 1.0;
    }

    const double cluster_size = static_cast<double>(candidate.cluster_size);
    const double min_size = static_cast<double>(min_cluster_size);
    const double max_size = static_cast<double>(max_cluster_size);
    const double normalized = (cluster_size - min_size) / (max_size - min_size);
    return std::clamp(normalized, 0.0, 1.0);
}

}  // namespace frontier_explorer
