#include "navigation_core/footprint_goal_validator.hpp"

namespace navigation
{
namespace navigation_core
{

void FootprintGoalValidator::configure(
    const robot_geometry_core::FootprintCollisionConfig & config)
{
    config_ = config;
}

bool FootprintGoalValidator::isGoalValid(
    const robot_geometry_core::Pose2D & goal,
    const grid_map_core::GridMap & map,
    std::string & reason,
    double & max_cost) const
{
    const auto result = robot_geometry_core::checkFootprint(map, goal, config_);
    max_cost = result.max_cost;
    reason = result.reason;
    return result.valid;
}

}  // namespace navigation_core
}  // namespace navigation
