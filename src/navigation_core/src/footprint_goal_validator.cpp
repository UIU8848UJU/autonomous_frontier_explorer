#include "navigation_core/footprint_goal_validator.hpp"

namespace frontier_explorer
{
namespace navigation_core
{

FootprintGoalValidator::FootprintGoalValidator(const CostmapAdapter & costmap)
: costmap_(costmap)
{
}

void FootprintGoalValidator::configure(const FootprintCollisionCheckerConfig & config)
{
    config_ = config;
}

bool FootprintGoalValidator::isGoalValid(
    const geometry_msgs::msg::PoseStamped & goal,
    std::string & reason,
    double & max_cost) const
{
    max_cost = 0.0;
    const auto result = FootprintCollisionChecker::checkPose(costmap_, goal.pose, config_);
    max_cost = result.max_cost;
    reason = result.reason;
    return result.valid;
}

}  // namespace navigation_core
}  // namespace frontier_explorer
