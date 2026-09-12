#pragma once

#include <cmath>

#include "frontier_strategy_core/types/frontier_types.hpp"

namespace frontier_strategy
{

// 计算两个 grid cell 间的欧氏距离，单位为米。
inline double grid_distance_in_meters(
    const GridCell & a,
    const GridCell & b,
    double resolution)
{
    const double dr = static_cast<double>(a.row - b.row);
    const double dc = static_cast<double>(a.col - b.col);
    return std::sqrt(dr * dr + dc * dc) * resolution;
}

}  // 命名空间 frontier_strategy
