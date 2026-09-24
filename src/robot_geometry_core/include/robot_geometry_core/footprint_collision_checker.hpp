#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "grid_map_core/types/grid_map.hpp"
#include "robot_geometry_core/types/footprint.hpp"
#include "robot_geometry_core/types/pose2d.hpp"

namespace robot_geometry_core
{

/// footprint 碰撞检查参数。地图值采用 OccupancyGrid 语义：-1 unknown，0 free，>50 obstacle。
struct FootprintCollisionConfig
{
    bool enabled{true};
    bool allow_unknown{false};
    std::int8_t occupied_threshold{51};
    Footprint footprint;
};

struct FootprintCollisionResult
{
    bool valid{false};
    std::string reason;
    double max_cost{0.0};
};

inline bool pointInPolygon(double x, double y, const std::vector<Point2D> & polygon);

inline Footprint makeCircularFootprint(double robot_radius, double padding)
{
    const double radius = std::max(0.01, robot_radius) + std::max(0.0, padding);
    constexpr std::size_t kPointCount = 16U;
    constexpr double kPi = 3.14159265358979323846;
    Footprint footprint;
    footprint.points.reserve(kPointCount);
    for (std::size_t index = 0U; index < kPointCount; ++index) {
        const double angle = 2.0 * kPi * static_cast<double>(index) /
            static_cast<double>(kPointCount);
        footprint.points.push_back(Point2D{radius * std::cos(angle), radius * std::sin(angle)});
    }
    return footprint;
}

/// 用外包正多边形近似圆，保证任意方向上的边界都不小于指定安全半径。
inline Footprint makeConservativeCircularFootprint(
    double clearance_radius,
    std::size_t point_count = 32U)
{
    constexpr double kPi = 3.14159265358979323846;
    point_count = std::max<std::size_t>(8U, point_count);
    if (!std::isfinite(clearance_radius) || clearance_radius <= 0.0) {
        throw std::invalid_argument("clearance radius must be positive and finite");
    }
    const double safe_radius = clearance_radius;
    const double vertex_radius = safe_radius / std::cos(kPi / static_cast<double>(point_count));
    Footprint footprint;
    footprint.points.reserve(point_count);
    for (std::size_t index = 0U; index < point_count; ++index) {
        const double angle = 2.0 * kPi * static_cast<double>(index) /
            static_cast<double>(point_count);
        footprint.points.push_back(
            Point2D{vertex_radius * std::cos(angle), vertex_radius * std::sin(angle)});
    }
    return footprint;
}

/// 在纯地图快照上检查 footprint 覆盖区域。
inline FootprintCollisionResult checkFootprint(
    const grid_map_core::GridMap & map,
    const Pose2D & pose,
    const FootprintCollisionConfig & config)
{
    FootprintCollisionResult result;
    if (!config.enabled) {
        result.valid = true;
        result.reason = "footprint_check_disabled";
        return result;
    }
    if (!map.isReady()) {
        result.reason = "footprint_costmap_unavailable";
        return result;
    }
    if (config.footprint.points.size() < 3U) {
        result.reason = "footprint_empty";
        return result;
    }

    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    std::vector<Point2D> oriented;
    oriented.reserve(config.footprint.points.size());
    for (const auto & point : config.footprint.points) {
        const Point2D transformed{
            pose.x + point.x * cos_yaw - point.y * sin_yaw,
            pose.y + point.x * sin_yaw + point.y * cos_yaw};
        oriented.push_back(transformed);
        min_x = std::min(min_x, transformed.x);
        min_y = std::min(min_y, transformed.y);
        max_x = std::max(max_x, transformed.x);
        max_y = std::max(max_y, transformed.y);
    }

    grid_map_core::GridCell min_cell;
    grid_map_core::GridCell max_cell;
    if (!map.worldToMap(min_x, min_y, min_cell) || !map.worldToMap(max_x, max_y, max_cell)) {
        result.reason = "footprint_out_of_costmap";
        return result;
    }

    bool sampled = false;
    const int start_col = std::min(min_cell.col, max_cell.col);
    const int end_col = std::max(min_cell.col, max_cell.col);
    const int start_row = std::min(min_cell.row, max_cell.row);
    const int end_row = std::max(min_cell.row, max_cell.row);
    for (int row = start_row; row <= end_row; ++row) {
        for (int col = start_col; col <= end_col; ++col) {
            double world_x = 0.0;
            double world_y = 0.0;
            map.mapToWorld(grid_map_core::GridCell{row, col}, world_x, world_y);
            if (!pointInPolygon(world_x, world_y, oriented)) {
                continue;
            }
            sampled = true;
            const auto value = map.value(static_cast<unsigned int>(col), static_cast<unsigned int>(row));
            result.max_cost = std::max(result.max_cost, static_cast<double>(std::max<std::int8_t>(0, value)));
            if (value < 0 && !config.allow_unknown) {
                result.reason = "footprint_over_unknown";
                return result;
            }
            if (value >= config.occupied_threshold) {
                result.reason = "footprint_collision";
                return result;
            }
        }
    }

    if (!sampled) {
        result.reason = "footprint_not_sampled";
        return result;
    }
    result.valid = true;
    result.reason = "footprint_valid";
    return result;
}

inline bool pointInPolygon(double x, double y, const std::vector<Point2D> & polygon)
{
    bool inside = false;
    for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
        const auto & current = polygon[i];
        const auto & previous = polygon[j];
        const bool intersects =
            ((current.y > y) != (previous.y > y)) &&
            (x < (previous.x - current.x) * (y - current.y) /
                ((previous.y - current.y) + 1e-12) + current.x);
        if (intersects) {
            inside = !inside;
        }
    }
    return inside;
}

}  // 命名空间 robot_geometry_core
