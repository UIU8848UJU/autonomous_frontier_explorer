#include "costmap_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_explorer
{

CostmapAdapter::CostmapAdapter(const rclcpp::Logger & logger)
: logger_(rclcpp::Logger(logger).get_child("costmap_adapter"))
{
}

bool CostmapAdapter::updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & map_msg)
{
    const auto width = map_msg.info.width;
    const auto height = map_msg.info.height;
    const auto resolution = map_msg.info.resolution;

    if (width == 0 || height == 0 || resolution <= 0.0F) {
        RCLCPP_WARN(
            logger_,
            "Invalid occupancy grid: width=%u, height=%u, resolution=%.6f",
            width,
            height,
            resolution);
        ready_ = false;
        costmap_.reset();
        return false;
    }

    const auto expected_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (map_msg.data.size() != expected_size) {
        RCLCPP_WARN(
            logger_,
            "Occupancy grid data size mismatch: expected=%zu, actual=%zu",
            expected_size,
            map_msg.data.size());
        ready_ = false;
        costmap_.reset();
        return false;
    }

    const bool should_log_init = !isReady() ||
        costmap_->getSizeInCellsX() != width ||
        costmap_->getSizeInCellsY() != height ||
        std::abs(costmap_->getResolution() - static_cast<double>(resolution)) > 1e-9 ||
        std::abs(costmap_->getOriginX() - map_msg.info.origin.position.x) > 1e-9 ||
        std::abs(costmap_->getOriginY() - map_msg.info.origin.position.y) > 1e-9;

    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        width,
        height,
        static_cast<double>(resolution),
        map_msg.info.origin.position.x,
        map_msg.info.origin.position.y,
        nav2_costmap_2d::NO_INFORMATION);

    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            const auto index = static_cast<std::size_t>(y) * width + x;
            costmap_->setCost(x, y, interpretOccupancyValue(map_msg.data[index]));
        }
    }

    ready_ = true;
    if (should_log_init) {
        RCLCPP_INFO(
            logger_,
            "Costmap initialized: width=%u, height=%u, resolution=%.3f, origin=(%.3f, %.3f)",
            width,
            height,
            resolution,
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y);
    }
    return true;
}

bool CostmapAdapter::isReady() const
{
    return ready_ && costmap_ != nullptr;
}

bool CostmapAdapter::worldToMap(
    double wx,
    double wy,
    unsigned int & mx,
    unsigned int & my) const
{
    if (!isReady()) {
        RCLCPP_DEBUG(logger_, "worldToMap failed because costmap is not ready.");
        return false;
    }

    const bool converted = costmap_->worldToMap(wx, wy, mx, my);
    if (!converted) {
        RCLCPP_DEBUG(
            logger_,
            "worldToMap failed: wx=%.3f, wy=%.3f, origin=(%.3f, %.3f), size=(%u, %u)",
            wx,
            wy,
            getOriginX(),
            getOriginY(),
            getSizeInCellsX(),
            getSizeInCellsY());
    }
    return converted;
}

void CostmapAdapter::mapToWorld(
    unsigned int mx,
    unsigned int my,
    double & wx,
    double & wy) const
{
    if (!isReady()) {
        wx = 0.0;
        wy = 0.0;
        RCLCPP_DEBUG(logger_, "mapToWorld requested before costmap is ready.");
        return;
    }

    costmap_->mapToWorld(mx, my, wx, wy);
}

unsigned char CostmapAdapter::getCost(unsigned int mx, unsigned int my) const
{
    if (!isReady() || !inBounds(static_cast<int>(mx), static_cast<int>(my))) {
        return nav2_costmap_2d::NO_INFORMATION;
    }

    return costmap_->getCost(mx, my);
}

bool CostmapAdapter::inBounds(int mx, int my) const
{
    return isReady() &&
        mx >= 0 &&
        my >= 0 &&
        mx < static_cast<int>(costmap_->getSizeInCellsX()) &&
        my < static_cast<int>(costmap_->getSizeInCellsY());
}

bool CostmapAdapter::isFree(unsigned int mx, unsigned int my) const
{
    return getCost(mx, my) == nav2_costmap_2d::FREE_SPACE;
}

bool CostmapAdapter::isUnknown(unsigned int mx, unsigned int my) const
{
    return getCost(mx, my) == nav2_costmap_2d::NO_INFORMATION;
}

bool CostmapAdapter::isObstacle(unsigned int mx, unsigned int my) const
{
    const auto cost = getCost(mx, my);
    return cost >= nav2_costmap_2d::LETHAL_OBSTACLE &&
        cost != nav2_costmap_2d::NO_INFORMATION;
}

bool CostmapAdapter::hasUnknownNeighbor(unsigned int mx, unsigned int my) const
{
    if (!isReady()) {
        return false;
    }

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
                continue;
            }

            const int nx = static_cast<int>(mx) + dx;
            const int ny = static_cast<int>(my) + dy;
            if (!inBounds(nx, ny)) {
                continue;
            }

            if (isUnknown(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
                return true;
            }
        }
    }

    return false;
}

std::optional<double> CostmapAdapter::distanceToNearestObstacle(
    unsigned int mx,
    unsigned int my,
    int max_search_radius_cells) const
{
    if (!isReady() || !inBounds(static_cast<int>(mx), static_cast<int>(my))) {
        RCLCPP_DEBUG(logger_, "Clearance query failed because cell is unavailable.");
        return std::nullopt;
    }

    const int radius = std::max(0, max_search_radius_cells);
    std::optional<double> best_distance;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = static_cast<int>(mx) + dx;
            const int ny = static_cast<int>(my) + dy;
            if (!inBounds(nx, ny)) {
                continue;
            }

            if (!isObstacle(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
                continue;
            }

            const double distance =
                std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * getResolution();
            if (!best_distance.has_value() || distance < best_distance.value()) {
                best_distance = distance;
            }
        }
    }

    return best_distance;
}

unsigned int CostmapAdapter::getSizeInCellsX() const
{
    return isReady() ? costmap_->getSizeInCellsX() : 0U;
}

unsigned int CostmapAdapter::getSizeInCellsY() const
{
    return isReady() ? costmap_->getSizeInCellsY() : 0U;
}

double CostmapAdapter::getResolution() const
{
    return isReady() ? costmap_->getResolution() : 0.0;
}

double CostmapAdapter::getOriginX() const
{
    return isReady() ? costmap_->getOriginX() : 0.0;
}

double CostmapAdapter::getOriginY() const
{
    return isReady() ? costmap_->getOriginY() : 0.0;
}

const nav2_costmap_2d::Costmap2D & CostmapAdapter::getCostmap() const
{
    if (!isReady()) {
        throw std::runtime_error("CostmapAdapter is not ready");
    }
    return *costmap_;
}

unsigned char CostmapAdapter::interpretOccupancyValue(int8_t occupancy) const
{
    if (occupancy < 0) {
        return nav2_costmap_2d::NO_INFORMATION;
    }

    if (occupancy == 0) {
        return nav2_costmap_2d::FREE_SPACE;
    }

    if (occupancy > 50) {
        return nav2_costmap_2d::LETHAL_OBSTACLE;
    }

    // 当前 frontier 检测只需要 free / unknown / lethal 三类语义。
    // 1..50 的中间概率暂按比例映射为非致命 cost，保留旧逻辑中 >50 才视为障碍的取舍。
    return static_cast<unsigned char>(
        std::clamp(
            static_cast<int>(
                std::round(
                    static_cast<double>(occupancy) / 50.0 *
                    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1))),
            1,
            static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));
}

}  // namespace frontier_explorer
