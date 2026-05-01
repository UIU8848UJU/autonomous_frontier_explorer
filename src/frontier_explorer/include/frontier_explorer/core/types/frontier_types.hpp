#pragma once

#include <cstddef>
#include <vector>

namespace frontier_explorer
{

struct GridCell
{
    int row;
    int col;

    bool operator==(const GridCell & other) const
    {
        return row == other.row && col == other.col;
    }
};

struct GridCellHash
{
    std::size_t operator()(const GridCell & cell) const
    {
        // 简单哈希用于甄别
        return (static_cast<std::size_t>(cell.row) << 32) ^
            static_cast<std::size_t>(cell.col);
    }
};

// 聚类
struct FrontierCluster
{
    std::vector<GridCell> cells;
    GridCell centroid{};
};


// 枚举各种状态 
enum class ExplorationState
{
    IDLE,
    RUNNING,
    STOPPED,
    COMPLETED,
    STUCK
};

}  // namespace frontier_explorer
