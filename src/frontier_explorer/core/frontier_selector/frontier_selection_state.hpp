#pragma once

#include <optional>
#include <unordered_map>
#include <unordered_set>

#include "types.h"

namespace frontier_explorer
{

// FrontierSelector 持有的长期选择状态。
//
// 这些状态跨 explore timer 周期保留，用于避免反复选择失败目标，
// 也用于在 cluster 层面抑制持续失败的 frontier 区域。
struct FrontierSelectionState
{
    // 最近一次下发给 Nav2 的 goal。
    std::optional<GridCell> last_goal_grid;

    // goal 级失败次数。key 为目标 grid cell，value 为累计失败次数。
    std::unordered_map<GridCell, int, GridCellHash> failed_goal_counts;

    // goal 级黑名单。失败次数达到阈值后加入。
    std::unordered_set<GridCell, GridCellHash> blacklist;

    // cluster 级失败次数。key 使用 cluster centroid。
    std::unordered_map<GridCell, int, GridCellHash> failed_cluster_counts;

    // cluster 级黑名单。cluster 持续无法产出可用 goal 时加入。
    std::unordered_set<GridCell, GridCellHash> cluster_blacklist;

    // 清空所有历史状态，用于重新开始探索或重置选择器。
    void clear()
    {
        last_goal_grid.reset();
        failed_goal_counts.clear();
        blacklist.clear();
        failed_cluster_counts.clear();
        cluster_blacklist.clear();
    }
};

}  // namespace frontier_explorer
