#pragma once

#include <optional>

#include "grid_map_core/types/grid_cell.hpp"
#include "grid_map_core/types/grid_map.hpp"

namespace exploration_core
{

/// @brief 一次策略评估所看到的环境和执行上下文。
///
/// 行为层只关心地图、机器人所在栅格和当前任务是否仍在运行，不关心这些数据来自哪个 ROS 节点。
struct ExplorationObservation
{
    std::optional<grid_map_core::GridMap> map;
    std::optional<grid_map_core::GridCell> robot_cell;
    bool mapping_active{false};
};

}  // namespace exploration_core
