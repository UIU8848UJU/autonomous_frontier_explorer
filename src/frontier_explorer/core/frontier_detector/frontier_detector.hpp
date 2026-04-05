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
  FrontierDetector( int obstacle_search_radius_cells, int min_frontier_cluster_size);

    /// @brief 找到一个格子满足三个条件：当前的格子没有探测过，这个格子周围有未知点，且这个格子是安全的
    std::vector<GridCell> detect_frontier_cells(
        const nav_msgs::msg::OccupancyGrid & map) const;

    std::vector<FrontierCluster> cluster_frontiers(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<GridCell> & frontier_cells) const;

private:
    bool is_frontier_cell_safe(
        const nav_msgs::msg::OccupancyGrid & map,
        const GridCell & cell) const;

private:
    int obstacle_search_radius_cells_{2};
    int min_frontier_cluster_size_{5};
};

}  // namespace frontier_explorer