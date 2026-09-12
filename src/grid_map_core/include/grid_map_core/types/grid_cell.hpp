#pragma once

#include <cstddef>

namespace grid_map_core
{

/// @brief 栅格地图中的整数坐标，row 表示行，col 表示列。
struct GridCell
{
    int row{0};
    int col{0};

    bool operator==(const GridCell & other) const
    {
        return row == other.row && col == other.col;
    }
};

/// @brief 供无序容器使用的栅格坐标哈希。
struct GridCellHash
{
    std::size_t operator()(const GridCell & cell) const
    {
        return (static_cast<std::size_t>(cell.row) << 32) ^
               static_cast<std::size_t>(cell.col);
    }
};

}  // namespace grid_map_core
