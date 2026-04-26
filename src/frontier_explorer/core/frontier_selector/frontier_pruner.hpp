#pragma once

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_decision_types.hpp"
#include "types.h"

namespace frontier_explorer
{

// pruner 运行时需要读取的选择状态。
//
// 这些指针都只读，实际状态所有权仍然在 FrontierSelector 中。
// 使用指针是为了让 pruner 保持轻量，不复制历史状态和黑名单集合。
struct FrontierPruningContext
{
    // 最近一次下发的目标点，用于避免重复选择同一个目标。
    std::optional<GridCell> last_goal;

    // goal 级失败次数和黑名单。
    const std::unordered_map<GridCell, int, GridCellHash> * failed_goal_counts{nullptr};
    const std::unordered_set<GridCell, GridCellHash> * goal_blacklist{nullptr};

    // cluster 级失败次数和黑名单。
    const std::unordered_map<GridCell, int, GridCellHash> * failed_cluster_counts{nullptr};
    const std::unordered_set<GridCell, GridCellHash> * cluster_blacklist{nullptr};
};

// frontier 硬过滤器。
//
// 职责边界：
// - 过滤太小的 cluster；
// - 过滤已拉黑或重试次数超限的 goal / cluster；
// - 过滤距离机器人太近的候选点；
// - centroid 不可用时，在 cluster 内寻找 fallback goal；
// - 基于地图做边界、free cell 等硬约束，并计算 unknown ratio 供 scorer 使用。
//
// 不负责：
// - 候选排序；
// - 分项打分和总分计算；
// - 更新 selector 的长期状态。
class FrontierPruner
{
public:
    FrontierPruner(
        double min_goal_distance_m,
        int max_retry_count,
        int max_cluster_retry_count,
        std::size_t min_cluster_size,
        int unknown_margin_cells);

    // 将原始 frontier clusters 转换成可打分候选。
    //
    // 返回值只包含通过硬过滤的 FrontierCandidate。
    // 如果某个 cluster 的 centroid 和 fallback 都不可用，会把该 cluster id
    // 追加到 failed_cluster_ids，交给 FrontierSelector 更新失败状态。
    std::vector<FrontierCandidate> prune_clusters(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid * map,
        const FrontierPruningContext & context,
        std::vector<GridCell> * failed_cluster_ids = nullptr) const;

private:
    // 判断目标是否与最近一次目标相同。
    bool is_same_as_last_goal(
        const GridCell & goal,
        const std::optional<GridCell> & last_goal) const;

    // 判断目标是否应被硬过滤，例如在黑名单中或重试次数超限。
    bool should_skip_goal(
        const GridCell & goal,
        const FrontierPruningContext & context) const;

    // 查询目标已失败次数；没有历史记录时返回 0。
    int retry_count_of_goal(
        const GridCell & goal,
        const FrontierPruningContext & context) const;

    // 当 cluster centroid 不适合作为 goal 时，从 cluster.cells 中找一个可用替代点。
    std::optional<GridCell> find_fallback_goal_in_cluster(
        const FrontierCluster & cluster,
        const GridCell & robot_grid,
        double resolution,
        const nav_msgs::msg::OccupancyGrid * map,
        const FrontierPruningContext & context) const;

    // 检查候选点是否满足地图硬约束。
    //
    // 如果传入 unknown_ratio，会输出候选点邻域内 unknown cell 的比例，
    // 供后续 information gain score / unknown risk penalty 使用。
    bool pass_map_candidate_constraints(
        const GridCell & cell,
        const nav_msgs::msg::OccupancyGrid * map,
        double * unknown_ratio = nullptr) const;

private:
    // 硬过滤参数。
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::size_t min_cluster_size_{1U};
    int unknown_margin_cells_{2};
};

}  // namespace frontier_explorer
