#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "frontier_explorer_core/selector/candidates/frontier_decision_types.hpp"
#include "frontier_explorer_core/types/frontier_types.hpp"
#include "frontier_explorer_core/types/grid_map.hpp"

namespace frontier_explorer
{

/// @brief pruner 运行时需要读取的选择状态，状态所有权由 selection policy 持有。
struct FrontierPruningContext
{
    std::optional<GridCell> last_goal;
    const std::unordered_map<GridCell, int, GridCellHash> * failed_goal_counts{nullptr};
    const std::unordered_set<GridCell, GridCellHash> * goal_blacklist{nullptr};
    const std::unordered_map<GridCell, int, GridCellHash> * failed_cluster_counts{nullptr};
    const std::unordered_set<GridCell, GridCellHash> * cluster_blacklist{nullptr};
};

/// @brief 平台相关的候选安全检查和 clearance 查询接缝。
///
/// 纯核心只认识 GridMap 和领域数据。轮式机器人可由 ROS facade 注入 Nav2 costmap
/// 与 footprint 检查；双足或飞行平台可以注入自己的可行域和安全距离实现。
struct FrontierPruningEnvironment
{
    const GridMap * frontier_map{nullptr};
    std::function<bool(const GridCell &)> safety_check;
    std::function<std::optional<double>(const GridCell &)> clearance_query;
};

/// @brief ROS-free Frontier 硬过滤器，负责候选生成、地图约束和 fallback。
class FrontierPruner
{
public:
    FrontierPruner(
        double min_goal_distance_m,
        int max_retry_count,
        int max_cluster_retry_count,
        std::size_t min_cluster_size,
        int unknown_margin_cells,
        int goal_inset_cells,
        double max_unknown_ratio);

    std::vector<FrontierCandidate> prune_clusters(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const FrontierPruningEnvironment & environment,
        const FrontierPruningContext & context,
        std::vector<GridCell> * failed_cluster_ids = nullptr) const;

private:
    bool is_same_as_last_goal(
        const GridCell & goal,
        const std::optional<GridCell> & last_goal) const;
    bool should_skip_goal(
        const GridCell & goal,
        const FrontierPruningContext & context) const;
    int retry_count_of_goal(
        const GridCell & goal,
        const FrontierPruningContext & context) const;
    std::optional<GridCell> find_fallback_goal_in_cluster(
        const FrontierCluster & cluster,
        const GridCell & robot_grid,
        double resolution,
        const GridMap * frontier_map,
        const FrontierPruningContext & context) const;
    bool pass_map_candidate_constraints(
        const GridCell & cell,
        const GridMap * frontier_map,
        double * unknown_ratio = nullptr) const;
    bool pass_safety_candidate_constraints(
        const GridCell & cell,
        const FrontierPruningEnvironment & environment) const;
    GridCell inset_goal_toward_robot(
        const GridCell & frontier_goal,
        const GridCell & robot_grid,
        const GridMap * frontier_map,
        const FrontierPruningContext & context) const;
    double compute_clearance_m(
        const GridCell & cell,
        const FrontierPruningEnvironment & environment) const;

    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::size_t min_cluster_size_{1U};
    int unknown_margin_cells_{2};
    int goal_inset_cells_{2};
    double max_unknown_ratio_{0.4};
};

}  // namespace frontier_explorer
