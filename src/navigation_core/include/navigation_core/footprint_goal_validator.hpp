#pragma once

#include <string>

#include "frontier_explorer_ros/costmap/costmap_adapter.hpp"
#include "frontier_explorer_ros/geometry/footprint_collision_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace frontier_explorer
{
namespace navigation_core
{

/// @brief: ???? footprint ?????
class FootprintGoalValidator
{
public:
    /// @brief: ?????
    /// @param costmap ??????? costmap???????????
    explicit FootprintGoalValidator(const CostmapAdapter & costmap);

    /// @brief: ????
    /// @param config footprint ??????
    void configure(const FootprintCollisionCheckerConfig & config);

    /// @brief: ??????? robot footprint ??????
    /// @param goal ???????
    /// @param reason ??????
    /// @param max_cost ?? footprint ???
    /// @return: true ?? footprint ???
    bool isGoalValid(
        const geometry_msgs::msg::PoseStamped & goal,
        std::string & reason,
        double & max_cost) const;

private:
    const CostmapAdapter & costmap_;
    FootprintCollisionCheckerConfig config_;
};

}  // namespace navigation_core
}  // namespace frontier_explorer
