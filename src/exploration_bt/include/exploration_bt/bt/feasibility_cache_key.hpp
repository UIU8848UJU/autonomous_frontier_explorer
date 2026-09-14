#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace exploration
{

/// @brief 生成可行性缓存键，区分地图、costmap、起点区域、目标区域和 planner。
inline std::string make_feasibility_cache_key(
    uint64_t map_revision,
    uint64_t costmap_revision,
    uint64_t start_region_revision,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id,
    double region_size_m)
{
    const double safe_region_size = std::max(0.01, region_size_m);
    const auto quantize = [safe_region_size](double value) {
        return static_cast<long long>(std::llround(value / safe_region_size));
    };

    std::ostringstream key;
    key << map_revision << ':'
        << costmap_revision << ':'
        << start_region_revision << ':'
        << quantize(goal.pose.position.x) << ':'
        << quantize(goal.pose.position.y) << ':'
        << planner_id;
    return key.str();
}

}  // 命名空间 exploration
