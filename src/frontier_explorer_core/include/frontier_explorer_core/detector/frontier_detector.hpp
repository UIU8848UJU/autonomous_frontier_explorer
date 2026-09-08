#pragma once

#include <cstddef>
#include <vector>

#include "frontier_explorer_core/types/grid_map.hpp"
#include "frontier_explorer_core/types/frontier_types.hpp"

namespace frontier_explorer
{

/// @brief: ROS-free frontier 检测器，基于 GridMap 判断 free、unknown 和障碍邻域
class FrontierDetector
{
public:
    /// @brief: 构造 FrontierDetector
    /// @param obstacle_search_radius_cells frontier 与障碍物的最小安全搜索半径，单位 cell
    explicit FrontierDetector(int obstacle_search_radius_cells);

    /// @brief: 找到满足条件的 frontier cell：当前格子已知且空闲，邻域存在未知点，并且没有贴近障碍
    /// @param costmap 地图适配器
    /// @return: frontier cell 列表
    std::vector<GridCell> detect_frontier_cells(
        const GridMap & map) const;

    /// @brief: 只负责把 frontier cell 聚成簇，不在这里按簇大小过滤
    /// @param costmap 地图适配器，保留用于后续聚类约束扩展
    /// @param frontier_cells 待聚类的 frontier cell
    /// @return: frontier cluster 列表
    std::vector<FrontierCluster> cluster_frontiers(
        const GridMap & map,
        const std::vector<GridCell> & frontier_cells) const;

private:
    /// @brief: 判断 frontier cell 是否远离障碍
    /// @param costmap 地图适配器
    /// @param cell 待检查 cell
    /// @return: true 表示满足安全半径约束
    bool is_frontier_cell_safe(
        const GridMap & map,
        const GridCell & cell) const;

private:
    int obstacle_search_radius_cells_{2};
};

}  // namespace frontier_explorer
