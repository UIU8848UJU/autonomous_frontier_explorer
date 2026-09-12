#pragma once

#include <string>

#include "grid_map_core/types/grid_map.hpp"
#include "robot_geometry_core/footprint_collision_checker.hpp"

namespace navigation
{
namespace navigation_core
{

/// 使用 robot_geometry_core 检查目标位姿是否具备安全落脚空间。
class FootprintGoalValidator
{
public:
    void configure(const robot_geometry_core::FootprintCollisionConfig & config);

    bool isGoalValid(
        const robot_geometry_core::Pose2D & goal,
        const grid_map_core::GridMap & map,
        std::string & reason,
        double & max_cost) const;

private:
    robot_geometry_core::FootprintCollisionConfig config_;
};

}  // namespace navigation_core
}  // namespace navigation
