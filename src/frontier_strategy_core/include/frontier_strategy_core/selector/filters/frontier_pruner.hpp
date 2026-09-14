#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "frontier_strategy_core/selector/candidates/frontier_decision_types.hpp"
#include "frontier_strategy_core/types/frontier_types.hpp"
#include "grid_map_core/types/grid_map.hpp"

namespace frontier_strategy
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
/// 纯核心只认识 GridMap 和领域数据。轮式机器人可由 ROS 适配层注入 Nav2 costmap
/// 与 footprint 检查；双足或飞行平台可以注入自己的可行域和安全距离实现。
struct FrontierPruningEnvironment
{
    const grid_map_core::GridMap * frontier_map{nullptr};
    std::function<bool(const GridCell &)> safety_check;
    std::function<std::optional<double>(const GridCell &)> clearance_query;
};

/// @brief 用二维射线估算候选观测位姿能够触达的未知栅格数量。
///
/// 射线从已知自由候选点出发，遇到地图外或障碍停止；未知栅格只计一次，
/// 这样同一未知区域不会因为多条射线重叠而被重复夸大。该函数只依赖地图，
/// 不引入传感器、ROS 或 Nav2 类型。
std::size_t estimate_visible_unknown_cells(
    const grid_map_core::GridMap & map,
    const GridCell & viewpoint,
    double sensor_range_m,
    double ray_step_cells,
    double angle_step_deg);

/// @brief 不依赖 ROS 的 Frontier 硬过滤器，负责候选生成、地图约束和回退处理。
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
        double max_unknown_ratio,
        std::vector<double> retreat_distances_m = {0.25, 0.4},
        std::vector<double> sample_radii_m = {0.35, 0.55},
        double viewpoint_angle_step_deg = 30.0,
        double sensor_range_m = 0.0,
        double information_gain_ray_step_cells = 1.0,
        std::size_t minimum_visible_unknown_cells = 0U);

    std::vector<FrontierCandidate> prune_clusters(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const FrontierPruningEnvironment & environment,
        const FrontierPruningContext & context,
        std::vector<GridCell> * failed_cluster_ids = nullptr,
        FrontierDecisionDiagnostics * diagnostics = nullptr) const;

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
        const grid_map_core::GridMap * frontier_map,
        const FrontierPruningContext & context) const;
    bool pass_map_candidate_constraints(
        const GridCell & cell,
        const grid_map_core::GridMap * frontier_map,
        double * unknown_ratio = nullptr) const;
    bool pass_safety_candidate_constraints(
        const GridCell & cell,
        const FrontierPruningEnvironment & environment) const;
    GridCell inset_goal_toward_robot(
        const GridCell & frontier_goal,
        const GridCell & robot_grid,
        const grid_map_core::GridMap * frontier_map,
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
    std::vector<double> retreat_distances_m_;
    std::vector<double> sample_radii_m_;
    double viewpoint_angle_step_deg_{30.0};
    double sensor_range_m_{0.0};
    double information_gain_ray_step_cells_{1.0};
    std::size_t minimum_visible_unknown_cells_{0U};
};

}  // namespace frontier_strategy
