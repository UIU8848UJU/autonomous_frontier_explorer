#include "frontier_strategy_core/information_gain/information_gain_estimator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace frontier_strategy
{
namespace
{
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
constexpr double kAngleEpsilon = 1e-12;

struct VisibilityEvent
{
    double distance_cells{0.0};
    int row{0};
    int col{0};
    bool obstacle{false};
};

double normalize_angle(double angle)
{
    angle = std::fmod(angle, kTwoPi);
    return angle < 0.0 ? angle + kTwoPi : angle;
}

// 返回障碍栅格从观测点看过去占据的最小角区间；跨越 0 度时拆成两个区间。
std::vector<std::pair<double, double>> angular_intervals(
    const GridCell & viewpoint,
    int row,
    int col)
{
    const double relative_col = static_cast<double>(col - viewpoint.col);
    const double relative_row = static_cast<double>(row - viewpoint.row);
    std::array<double, 4> angles{
        normalize_angle(std::atan2(relative_row - 0.5, relative_col - 0.5)),
        normalize_angle(std::atan2(relative_row - 0.5, relative_col + 0.5)),
        normalize_angle(std::atan2(relative_row + 0.5, relative_col - 0.5)),
        normalize_angle(std::atan2(relative_row + 0.5, relative_col + 0.5)),
    };
    std::sort(angles.begin(), angles.end());

    std::size_t largest_gap_index = 0U;
    double largest_gap = -1.0;
    for (std::size_t index = 0U; index < angles.size(); ++index) {
        const double next = index + 1U < angles.size() ?
            angles[index + 1U] : angles.front() + kTwoPi;
        const double gap = next - angles[index];
        if (gap > largest_gap) {
            largest_gap = gap;
            largest_gap_index = index;
        }
    }

    const std::size_t start_index = (largest_gap_index + 1U) % angles.size();
    const double start = angles[start_index];
    double end = angles[largest_gap_index];
    if (end < start) {
        end += kTwoPi;
    }
    if (end <= kTwoPi) {
        return {{start, end}};
    }
    return {{start, kTwoPi}, {0.0, end - kTwoPi}};
}

bool angle_is_shadowed(
    const std::map<double, double> & shadows,
    double angle)
{
    const auto upper = shadows.upper_bound(angle + kAngleEpsilon);
    if (upper == shadows.begin()) {
        return false;
    }
    const auto interval = std::prev(upper);
    return interval->first - kAngleEpsilon <= angle &&
           interval->second + kAngleEpsilon >= angle;
}

bool interval_is_shadowed(
    const std::map<double, double> & shadows,
    double start,
    double end)
{
    const auto upper = shadows.upper_bound(start + kAngleEpsilon);
    if (upper == shadows.begin()) {
        return false;
    }
    const auto interval = std::prev(upper);
    return interval->first - kAngleEpsilon <= start &&
           interval->second + kAngleEpsilon >= end;
}

void add_shadow_interval(
    std::map<double, double> & shadows,
    double start,
    double end)
{
    auto current = shadows.lower_bound(start);
    if (current != shadows.begin()) {
        auto previous = std::prev(current);
        if (previous->second + kAngleEpsilon >= start) {
            start = previous->first;
            end = std::max(end, previous->second);
            current = shadows.erase(previous);
        }
    }
    while (current != shadows.end() && current->first <= end + kAngleEpsilon) {
        end = std::max(end, current->second);
        current = shadows.erase(current);
    }
    shadows.emplace(start, end);
}

double nearest_cell_distance(int row_offset, int col_offset)
{
    const double row_distance = std::max(0.0, std::abs(static_cast<double>(row_offset)) - 0.5);
    const double col_distance = std::max(0.0, std::abs(static_cast<double>(col_offset)) - 0.5);
    return std::hypot(row_distance, col_distance);
}
}  // namespace

InformationGainEstimator::InformationGainEstimator(double sensor_range_m)
: sensor_range_m_(std::isfinite(sensor_range_m) ? std::max(0.0, sensor_range_m) : 0.0)
{
}

bool InformationGainEstimator::enabled() const
{
    return sensor_range_m_ > 0.0;
}

double InformationGainEstimator::sensor_range_m() const
{
    return sensor_range_m_;
}

InformationGainEstimate InformationGainEstimator::estimate(
    const grid_map_core::GridMap & map,
    const GridCell & viewpoint) const
{
    InformationGainEstimate result;
    if (!enabled() || !map.isReady() || !std::isfinite(map.resolution) ||
        map.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
        map.height > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
        !map.inBounds(viewpoint.col, viewpoint.row) ||
        !map.isFree(
            static_cast<unsigned int>(viewpoint.col),
            static_cast<unsigned int>(viewpoint.row)))
    {
        return result;
    }

    result.valid = true;
    const int map_max_col = static_cast<int>(map.width) - 1;
    const int map_max_row = static_cast<int>(map.height) - 1;
    const int max_useful_radius_cells = std::max({
        viewpoint.col,
        map_max_col - viewpoint.col,
        viewpoint.row,
        map_max_row - viewpoint.row});
    const double requested_radius_cells = sensor_range_m_ / map.resolution;
    const int radius_cells = static_cast<int>(std::ceil(std::min(
        requested_radius_cells,
        static_cast<double>(max_useful_radius_cells))));

    const auto min_col = static_cast<int>(std::max<std::int64_t>(
        0,
        static_cast<std::int64_t>(viewpoint.col) - radius_cells));
    const auto max_col = static_cast<int>(std::min<std::int64_t>(
        map_max_col,
        static_cast<std::int64_t>(viewpoint.col) + radius_cells));
    const auto min_row = static_cast<int>(std::max<std::int64_t>(
        0,
        static_cast<std::int64_t>(viewpoint.row) - radius_cells));
    const auto max_row = static_cast<int>(std::min<std::int64_t>(
        map_max_row,
        static_cast<std::int64_t>(viewpoint.row) + radius_cells));

    std::vector<VisibilityEvent> events;
    events.reserve(static_cast<std::size_t>(max_col - min_col + 1) *
        static_cast<std::size_t>(max_row - min_row + 1));
    for (int row = min_row; row <= max_row; ++row) {
        for (int col = min_col; col <= max_col; ++col) {
            ++result.examined_cells;
            if (row == viewpoint.row && col == viewpoint.col) {
                continue;
            }

            const int row_offset = row - viewpoint.row;
            const int col_offset = col - viewpoint.col;
            const bool obstacle = map.isObstacle(
                static_cast<unsigned int>(col),
                static_cast<unsigned int>(row));
            if (obstacle) {
                const double near_distance_m =
                    nearest_cell_distance(row_offset, col_offset) * map.resolution;
                if (near_distance_m <= sensor_range_m_) {
                    events.push_back(VisibilityEvent{
                        near_distance_m / map.resolution, row, col, true});
                }
                continue;
            }
            if (!map.isUnknown(
                    static_cast<unsigned int>(col),
                    static_cast<unsigned int>(row)))
            {
                continue;
            }

            const double center_distance_cells = std::hypot(
                static_cast<double>(col_offset),
                static_cast<double>(row_offset));
            if (center_distance_cells * map.resolution <= sensor_range_m_) {
                events.push_back(VisibilityEvent{
                    center_distance_cells, row, col, false});
            }
        }
    }

    std::sort(
        events.begin(),
        events.end(),
        [](const VisibilityEvent & lhs, const VisibilityEvent & rhs) {
            if (lhs.distance_cells == rhs.distance_cells) {
                return lhs.obstacle && !rhs.obstacle;
            }
            return lhs.distance_cells < rhs.distance_cells;
        });

    std::map<double, double> shadows;
    for (const auto & event : events) {
        if (event.obstacle) {
            for (const auto & [start, end] : angular_intervals(
                    viewpoint, event.row, event.col))
            {
                if (!interval_is_shadowed(shadows, start, end)) {
                    add_shadow_interval(shadows, start, end);
                }
            }
            continue;
        }

        const double angle = normalize_angle(std::atan2(
            static_cast<double>(event.row - viewpoint.row),
            static_cast<double>(event.col - viewpoint.col)));
        if (!angle_is_shadowed(shadows, angle)) {
            ++result.visible_unknown_cells;
        }
    }

    result.visible_unknown_area_m2 =
        static_cast<double>(result.visible_unknown_cells) * map.resolution * map.resolution;
    return result;
}

}  // 命名空间 frontier_strategy
