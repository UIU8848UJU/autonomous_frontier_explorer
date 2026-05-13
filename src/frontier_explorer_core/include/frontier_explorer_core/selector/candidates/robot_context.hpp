#pragma once

#include <cstddef>
#include <optional>

#include "frontier_explorer_core/types/frontier_types.hpp"

namespace frontier_explorer
{

/// @brief: 单次候选打分所需的机器人状态和候选批次统计上下文
struct RobotContext
{
    std::optional<GridCell> last_goal;
    double min_candidate_distance_m{0.0};
    double max_candidate_distance_m{0.0};
    std::size_t min_candidate_cluster_size{0U};
    std::size_t max_candidate_cluster_size{0U};
    double max_candidate_clearance_m{0.0};
};

}  // namespace frontier_explorer
