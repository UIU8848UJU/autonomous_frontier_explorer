#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "frontier_explorer_core/types/frontier_types.hpp"

namespace frontier_explorer
{

/// @brief: ROS-free occupancy grid representation used by frontier algorithms
struct GridMap
{
    unsigned int width{0U};
    unsigned int height{0U};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    std::vector<std::int8_t> data;

    bool isReady() const
    {
        return width > 0U && height > 0U && resolution > 0.0 &&
               data.size() == static_cast<std::size_t>(width) * height;
    }

    bool inBounds(int col, int row) const
    {
        return col >= 0 && row >= 0 &&
               col < static_cast<int>(width) && row < static_cast<int>(height);
    }

    std::int8_t value(unsigned int col, unsigned int row) const
    {
        if (!isReady() || !inBounds(static_cast<int>(col), static_cast<int>(row))) {
            return -1;
        }
        return data[static_cast<std::size_t>(row) * width + col];
    }

    bool isFree(unsigned int col, unsigned int row) const
    {
        return value(col, row) == 0;
    }

    bool isUnknown(unsigned int col, unsigned int row) const
    {
        return value(col, row) < 0;
    }

    bool isObstacle(unsigned int col, unsigned int row) const
    {
        return value(col, row) > 50;
    }

    bool mapToWorld(const GridCell & cell, double & world_x, double & world_y) const
    {
        if (!isReady() || !inBounds(cell.col, cell.row)) {
            world_x = 0.0;
            world_y = 0.0;
            return false;
        }

        world_x = origin_x + (static_cast<double>(cell.col) + 0.5) * resolution;
        world_y = origin_y + (static_cast<double>(cell.row) + 0.5) * resolution;
        return true;
    }

    bool worldToMap(double world_x, double world_y, GridCell & cell) const
    {
        if (!isReady() || world_x < origin_x || world_y < origin_y) {
            return false;
        }

        const int col = static_cast<int>(std::floor((world_x - origin_x) / resolution));
        const int row = static_cast<int>(std::floor((world_y - origin_y) / resolution));
        if (!inBounds(col, row)) {
            return false;
        }

        cell = GridCell{row, col};
        return true;
    }

    std::optional<double> distanceToNearestObstacle(
        const GridCell & cell,
        int max_search_radius_cells) const
    {
        if (!isReady() || !inBounds(cell.col, cell.row)) {
            return std::nullopt;
        }

        const int radius = std::max(0, max_search_radius_cells);
        std::optional<double> best_distance;
        for (int row_offset = -radius; row_offset <= radius; ++row_offset) {
            for (int col_offset = -radius; col_offset <= radius; ++col_offset) {
                const int col = cell.col + col_offset;
                const int row = cell.row + row_offset;
                if (!inBounds(col, row) ||
                    !isObstacle(static_cast<unsigned int>(col), static_cast<unsigned int>(row)))
                {
                    continue;
                }

                const double distance = std::hypot(
                    static_cast<double>(col_offset),
                    static_cast<double>(row_offset)) * resolution;
                if (!best_distance.has_value() || distance < best_distance.value()) {
                    best_distance = distance;
                }
            }
        }
        return best_distance;
    }
};

}  // namespace frontier_explorer
