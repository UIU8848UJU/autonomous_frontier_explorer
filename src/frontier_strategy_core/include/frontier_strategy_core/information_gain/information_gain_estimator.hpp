#pragma once

#include <cstddef>

#include "frontier_strategy_core/types/frontier_types.hpp"
#include "grid_map_core/types/grid_map.hpp"

namespace frontier_strategy
{

/// @brief 信息增益估计结果。
struct InformationGainEstimate
{
    std::size_t visible_unknown_cells{0U};
    double visible_unknown_area_m2{0.0};
    // 本轮检查过的量程包围盒栅格数，用于验证工作量不会随每个未知格重复做射线扫描。
    std::size_t examined_cells{0U};
    bool valid{false};
};

/// @brief 不依赖 ROS 的二维栅格可见未知区域估计器。
///
/// 估计器检查传感器量程内的未知栅格，并使用地图障碍物做视线遮挡判断。
/// 输出使用平方米，避免同一物理区域因地图分辨率变化而获得不同量级的分数。
class InformationGainEstimator
{
public:
    explicit InformationGainEstimator(double sensor_range_m = 0.0);

    InformationGainEstimate estimate(
        const grid_map_core::GridMap & map,
        const GridCell & viewpoint) const;

    bool enabled() const;
    double sensor_range_m() const;

private:
    double sensor_range_m_{0.0};
};

}  // 命名空间 frontier_strategy
