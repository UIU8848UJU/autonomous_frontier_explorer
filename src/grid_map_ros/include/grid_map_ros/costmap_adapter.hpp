#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "grid_map_core/types/grid_map.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace grid_map_ros
{

/// @brief 将 ROS OccupancyGrid 适配为通用 GridMap 和 Nav2 Costmap2D。
/// @details 该类只负责 ROS 地图格式、坐标和 cost 语义转换，不包含 Frontier 或导航策略。
class CostmapAdapter
{
public:
    explicit CostmapAdapter(const rclcpp::Logger & logger);

    bool updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & map_msg);

    const grid_map_core::GridMap & gridMap() const;

    /// @brief 每次收到输入地图后的单调版本号，用于外部缓存失效。
    uint64_t revision() const;

    bool isReady() const;

    bool worldToMap(
        double wx,
        double wy,
        unsigned int & mx,
        unsigned int & my) const;

    void mapToWorld(
        unsigned int mx,
        unsigned int my,
        double & wx,
        double & wy) const;

    unsigned char getCost(unsigned int mx, unsigned int my) const;
    bool inBounds(int mx, int my) const;
    bool isFree(unsigned int mx, unsigned int my) const;
    bool isUnknown(unsigned int mx, unsigned int my) const;
    bool isObstacle(unsigned int mx, unsigned int my) const;
    bool hasUnknownNeighbor(unsigned int mx, unsigned int my) const;

    std::optional<double> distanceToNearestObstacle(
        unsigned int mx,
        unsigned int my,
        int max_search_radius_cells) const;

    unsigned int getSizeInCellsX() const;
    unsigned int getSizeInCellsY() const;
    double getResolution() const;
    double getOriginX() const;
    double getOriginY() const;

    const nav2_costmap_2d::Costmap2D & getCostmap() const;

private:
    unsigned char interpretOccupancyValue(int8_t occupancy) const;

    rclcpp::Logger logger_;
    std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    grid_map_core::GridMap grid_map_;
    bool ready_{false};
    uint64_t revision_{0U};
};

}  // namespace grid_map_ros
