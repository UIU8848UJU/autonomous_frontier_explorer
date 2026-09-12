#pragma once

#include <vector>

#include "grid_map_core/types/grid_cell.hpp"

namespace frontier_strategy
{

// Frontier 领域对象使用通用地图核心提供的栅格坐标类型。
using GridCell = grid_map_core::GridCell;
using GridCellHash = grid_map_core::GridCellHash;

// 聚类
struct FrontierCluster
{
    std::vector<GridCell> cells;
    GridCell centroid{};
};


}  // namespace frontier_strategy
