#pragma once

#include <string>

#include "frontier_explorer_ros/costmap/costmap_adapter.hpp"
#include "nav_msgs/msg/path.hpp"

namespace frontier_explorer
{
namespace navigation_core
{

/// @brief: ????????
struct PathSafetyCheckerConfig
{
    bool enabled{true};
    bool allow_unknown{false};
    unsigned char cost_threshold{253U};
};

/// @brief: ????????????????? unknown ??????
class PathSafetyChecker
{
public:
    /// @brief: ?????????
    /// @param costmap ??????? costmap???????????
    explicit PathSafetyChecker(const CostmapAdapter & costmap);

    /// @brief: ????
    /// @param config ??????
    void configure(const PathSafetyCheckerConfig & config);

    /// @brief: ????????
    /// @param path Nav2 planner ?????
    /// @param reason ??????
    /// @param max_path_cost ??????????
    /// @return: true ??????????????
    bool isSafe(
        const nav_msgs::msg::Path & path,
        std::string & reason,
        double & max_path_cost) const;

private:
    const CostmapAdapter & costmap_;
    PathSafetyCheckerConfig config_;
};

}  // namespace navigation_core
}  // namespace frontier_explorer
