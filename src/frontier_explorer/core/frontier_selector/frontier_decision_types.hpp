#pragma once

#include <cstddef>

#include "types.h"

namespace frontier_explorer
{

struct FrontierSelectionCandidate
{
    GridCell goal{};
    GridCell cluster_centroid{};
    std::size_t cluster_size{0U};
    double distance_m{0.0};

    int retry_count{0};
    bool blacklist_hit{false};
    double clearance_m{0.0};
    std::size_t source_cluster_index{0U};
};

struct ScoredFrontierCandidate
{
    FrontierSelectionCandidate candidate{};

    double distance_score{0.0};
    double cluster_size_score{0.0};
    double clearance_score{0.0};
    double revisit_penalty{0.0};
    double retry_penalty{0.0};

    double total_score{0.0};
};

struct FrontierScoringWeights
{
    double weight_distance{1.0};
    double weight_cluster_size{1.0};
    double weight_clearance{0.0};
    double weight_revisit_penalty{1.0};
    double weight_retry_penalty{1.0};

    bool enable_clearance_score{false};
    bool enable_revisit_penalty{false};
};

}  // namespace frontier_explorer
