#pragma once

#include "frontier_strategy_ros/types/frontier_strategy_params.hpp"

namespace frontier_strategy
{

/// @brief 根据地图分辨率把米制策略参数换算为 Core 使用的 cell 参数。
/// @param base_params 节点加载并校验后的基础参数
/// @param map_resolution 地图分辨率，单位 m/cell
/// @return 当前地图对应的有效策略参数
FrontierStrategyParams adapt_strategy_params_to_map_resolution(
    const FrontierStrategyParams & base_params,
    double map_resolution);

}  // 命名空间 frontier_strategy
