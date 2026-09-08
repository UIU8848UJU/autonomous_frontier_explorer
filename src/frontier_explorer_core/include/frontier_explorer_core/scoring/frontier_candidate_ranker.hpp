#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "frontier_explorer_core/reachability/frontier_reachability_result.hpp"
#include "frontier_explorer_core/scoring/frontier_scorer.hpp"

namespace frontier_explorer
{

/// @brief: 对候选进行打分、确定性排序，并附加可选的可达性诊断。
///
/// 该策略只依赖领域数据和回调，不负责 ROS 日志、地图访问或长期状态维护。
std::vector<ScoredFrontierCandidate> rank_frontier_candidates(
    const FrontierScorer & scorer,
    const std::vector<FrontierCandidate> & candidates,
    const std::optional<GridCell> & last_goal,
    const std::function<FrontierReachabilityResult(FrontierCandidate &)> &
        reachability_check = {});

}  // namespace frontier_explorer
