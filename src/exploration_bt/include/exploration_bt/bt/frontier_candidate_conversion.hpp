#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#include "exploration_bt/exploration_bt_context.hpp"
#include "robot_interfaces/srv/get_frontier_candidates.hpp"

namespace exploration
{

/// @brief 将 ROS 服务返回的并行数组转换为 BT 使用的候选对象列表。
/// @details 服务接口为了保持轻量使用并行数组；转换集中在这里，避免同步计算和预取各维护一套逻辑。
inline std::vector<ExplorationBtContext::FrontierCandidate> convert_frontier_candidates(
    const robot_interfaces::srv::GetFrontierCandidates::Response & response)
{
    const auto count = std::min({
        response.goals.size(),
        response.scores.size(),
        response.distance_m.size(),
        response.clearance_m.size(),
        response.unknown_ratio.size(),
        response.cluster_sizes.size(),
        response.retry_counts.size()});

    std::vector<ExplorationBtContext::FrontierCandidate> candidates;
    candidates.reserve(count);
    for (std::size_t index = 0U; index < count; ++index) {
        ExplorationBtContext::FrontierCandidate candidate;
        candidate.goal = response.goals[index];
        candidate.score = response.scores[index];
        candidate.distance_m = response.distance_m[index];
        candidate.clearance_m = response.clearance_m[index];
        candidate.unknown_ratio = response.unknown_ratio[index];
        if (index < response.information_gain.size()) {
            candidate.information_gain = response.information_gain[index];
        }
        candidate.cluster_size = response.cluster_sizes[index];
        candidate.retry_count = response.retry_counts[index];
        if (index < response.reachability_checked.size()) {
            candidate.reachability_checked = response.reachability_checked[index];
        }
        if (index < response.reachable.size()) {
            candidate.reachable = response.reachable[index];
        }
        if (index < response.path_length_m.size()) {
            candidate.path_length_m = response.path_length_m[index];
        }
        candidates.push_back(std::move(candidate));
    }
    return candidates;
}

}  // 命名空间 exploration
