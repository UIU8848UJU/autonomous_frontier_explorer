#pragma once

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "costmap_adapter.hpp"
#include "frontier_decision_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"

namespace frontier_explorer
{

/// @brief: pruner 运行时需要读取的选择状态，实际状态所有权仍然在 FrontierSelector 中
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

/// @brief: Frontier 硬过滤器，负责候选生成、/map 硬约束、fallback 和 clearance 等候选事实数据计算
class FrontierPruner
{
public:
    /// @brief: 构造 FrontierPruner
    /// @param min_goal_distance_m 目标与机器人最小距离
    /// @param max_retry_count 单个 goal 最大重试次数
    /// @param max_cluster_retry_count 单个 cluster 最大重试次数
    /// @param min_cluster_size 最小 cluster 尺寸
    /// @param unknown_margin_cells 统计 unknown ratio 的邻域半径
    /// @param logger ROS2 日志器
    FrontierPruner(
        double min_goal_distance_m,
        int max_retry_count,
        int max_cluster_retry_count,
        std::size_t min_cluster_size,
        int unknown_margin_cells,
        const rclcpp::Logger & logger = rclcpp::get_logger("frontier_explorer"));

    /// @brief: 将原始 frontier clusters 转换成可打分候选
    /// @param clusters 原始 frontier clusters
    /// @param robot_grid 机器人所在栅格
    /// @param resolution 地图分辨率
    /// @param frontier_costmap frontier 检测来源地图适配器，可为空
    /// @param safety_costmap clearance 评分来源地图适配器，可为空
    /// @param context 选择状态上下文
    /// @param failed_cluster_ids 输出无法产生可用目标的 cluster id
    /// @return: 通过硬过滤的 FrontierCandidate 列表
    std::vector<FrontierCandidate> prune_clusters(
        const std::vector<FrontierCluster> & clusters,
        const GridCell & robot_grid,
        double resolution,
        const CostmapAdapter * frontier_costmap,
        const CostmapAdapter * safety_costmap,
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
        const CostmapAdapter * frontier_costmap,
        const FrontierPruningContext & context) const;

    // 检查候选点是否满足地图硬约束。
    //
    // 如果传入 unknown_ratio，会输出候选点邻域内 unknown cell 的比例，
    // 供后续 information gain score / unknown risk penalty 使用。
    bool pass_map_candidate_constraints(
        const GridCell & cell,
        const CostmapAdapter * frontier_costmap,
        double * unknown_ratio = nullptr) const;

    /// @brief: 查询候选点到最近障碍物的安全距离
    /// @param cell 候选点在 frontier map 中的栅格坐标
    /// @param frontier_costmap frontier 检测来源地图适配器
    /// @param safety_costmap 安全检查来源地图适配器，可为空
    /// @return: 最近障碍距离，单位 m；不可用时返回 0
    double compute_clearance_m(
        const GridCell & cell,
        const CostmapAdapter * frontier_costmap,
        const CostmapAdapter * safety_costmap) const;

private:
    rclcpp::Logger logger_;
    // 硬过滤参数。
    double min_goal_distance_m_{0.5};
    int max_retry_count_{2};
    int max_cluster_retry_count_{3};
    std::size_t min_cluster_size_{1U};
    int unknown_margin_cells_{2};
};

}  // namespace frontier_explorer
