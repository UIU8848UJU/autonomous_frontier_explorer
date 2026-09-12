#pragma once

#include <cstdint>
#include <string>

#include "grid_map_core/types/grid_map.hpp"
#include "navigation_core/path_utils.hpp"

namespace navigation
{
namespace navigation_core
{

/// 路径安全检查参数，阈值采用 OccupancyGrid 的 0~100 语义。
struct PathSafetyCheckerConfig
{
    bool enabled{true};
    bool allow_unknown{false};
    std::int8_t occupied_threshold{51};
};

/// 只依赖纯地图和纯路径的路径安全检查器。
class PathSafetyChecker
{
public:
    void configure(const PathSafetyCheckerConfig & config);

    /// 检查路径是否越过 unknown 或高占用率区域。
    bool isSafe(
        const Path2D & path,
        const grid_map_core::GridMap & map,
        std::string & reason,
        double & max_path_cost) const;

private:
    PathSafetyCheckerConfig config_;
};

}  // namespace navigation_core
}  // namespace navigation
