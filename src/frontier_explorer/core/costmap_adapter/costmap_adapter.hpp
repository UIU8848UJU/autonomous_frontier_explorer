#pragma once

#include <memory>
#include <optional>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace frontier_explorer
{

/// @brief: 基于 Nav2 Costmap2D 的地图适配器，统一封装坐标转换、cost 查询和 frontier 基础判断
class CostmapAdapter
{
public:
    /// @brief: 构造 CostmapAdapter，传入日志器用于调试输出
    /// @param logger ROS2 日志器
    explicit CostmapAdapter(const rclcpp::Logger & logger);

    /// @brief: 使用 OccupancyGrid 更新内部 Costmap2D
    /// @param map_msg 输入的 OccupancyGrid 地图
    /// @return: 更新是否成功
    bool updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & map_msg);

    /// @brief: 判断内部 costmap 是否可用
    /// @return: true 表示已经有有效地图
    bool isReady() const;

    /// @brief: 世界坐标转换为地图栅格坐标
    /// @param wx 世界坐标 x
    /// @param wy 世界坐标 y
    /// @param mx 输出地图栅格 x
    /// @param my 输出地图栅格 y
    /// @return: 转换是否成功
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

    /// @brief: 地图栅格坐标转换为世界坐标
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @param wx 输出世界坐标 x
    /// @param wy 输出世界坐标 y
    void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

    /// @brief: 获取指定栅格的 cost 值
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @return: costmap 中的代价值
    unsigned char getCost(unsigned int mx, unsigned int my) const;

    /// @brief: 判断指定栅格是否在地图范围内
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @return: true 表示在范围内
    bool inBounds(int mx, int my) const;

    /// @brief: 判断指定栅格是否为空闲区域
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @return: true 表示 free
    bool isFree(unsigned int mx, unsigned int my) const;

    /// @brief: 判断指定栅格是否为未知区域
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @return: true 表示 unknown
    bool isUnknown(unsigned int mx, unsigned int my) const;

    /// @brief: 判断指定栅格是否为障碍物
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @return: true 表示 obstacle
    bool isObstacle(unsigned int mx, unsigned int my) const;

    /// @brief: 判断指定 free cell 周围是否存在 unknown 邻居
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @return: true 表示该点是 frontier 候选点
    bool hasUnknownNeighbor(unsigned int mx, unsigned int my) const;

    /// @brief: 获取指定栅格到最近障碍物的距离
    /// @param mx 地图栅格 x
    /// @param my 地图栅格 y
    /// @param max_search_radius_cells 最大搜索半径，单位 cell
    /// @return: 最近障碍距离，单位 m；没有找到障碍时返回 std::nullopt
    std::optional<double> distanceToNearestObstacle(
        unsigned int mx,
        unsigned int my,
        int max_search_radius_cells) const;

    /// @brief: 获取地图宽度，单位为 cell
    /// @return: 地图宽度
    unsigned int getSizeInCellsX() const;

    /// @brief: 获取地图高度，单位为 cell
    /// @return: 地图高度
    unsigned int getSizeInCellsY() const;

    /// @brief: 获取地图分辨率
    /// @return: resolution，单位 m/cell
    double getResolution() const;

    /// @brief: 获取地图原点 x 坐标
    /// @return: 地图原点 x
    double getOriginX() const;

    /// @brief: 获取地图原点 y 坐标
    /// @return: 地图原点 y
    double getOriginY() const;

    /// @brief: 获取内部 Costmap2D 只读引用
    /// @return: Costmap2D 只读引用
    const nav2_costmap_2d::Costmap2D & getCostmap() const;

private:
    /// @brief: 将 OccupancyGrid 的概率值转换为 Nav2 Costmap cost 值
    /// @param occupancy OccupancyGrid 中的原始值，-1 表示 unknown，0 表示 free，100 表示 occupied
    /// @return: Costmap2D 中使用的 cost 值
    unsigned char interpretOccupancyValue(int8_t occupancy) const;

private:
    rclcpp::Logger logger_;
    std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
    bool ready_{false};
};

}  // namespace frontier_explorer
