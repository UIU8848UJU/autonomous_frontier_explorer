#pragma once

#include <cmath>

#include "types.h"

namespace frontier_explorer
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

}  // namespace frontier_explorer
