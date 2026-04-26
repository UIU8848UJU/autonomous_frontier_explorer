#pragma once

#include <cstddef>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_utils.hpp"
#include "types.h"

namespace frontier_explorer
{

class FrontierDetector
{
public:
  explicit FrontierDetector(int obstacle_search_radius_cells);

    /// @brief 找到满足条件的 frontier cell：当前格子已知且空闲，邻域存在未知点，并且没有贴近障碍。
    std::vector<GridCell> detect_frontier_cells(
        const nav_msgs::msg::OccupancyGrid & map) const;

    /// @brief 只负责把 frontier cell 聚成簇，不在这里按簇大小过滤。
    ///        簇大小过滤属于 FrontierPruner 的职责，便于运行时通过 YAML 放宽小边界。
    std::vector<FrontierCluster> cluster_frontiers(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<GridCell> & frontier_cells) const;

private:
    bool is_frontier_cell_safe(
        const nav_msgs::msg::OccupancyGrid & map,
        const GridCell & cell) const;

private:
    int obstacle_search_radius_cells_{2};
};

}  // namespace frontier_explorer
