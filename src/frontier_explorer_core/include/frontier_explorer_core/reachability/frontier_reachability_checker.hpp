#pragma once

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace frontier_explorer
{

/// @brief: frontier 候选目标可达性检查结果
struct FrontierReachabilityResult
{
    bool checked{false};
    bool reachable{true};
    double path_length_m{0.0};
    std::string reason;
};

/// @brief: frontier planner 可达性检查接口，只表达能否规划路径，不表达 footprint 落脚碰撞
class FrontierReachabilityChecker
{
public:
    virtual ~FrontierReachabilityChecker() = default;

    /// @brief: 检查从机器人位姿到候选目标是否可规划路径
    /// @param start map frame 下的机器人位姿
    /// @param goal map frame 下的候选目标
    /// @return: 可达性检查结果
    virtual FrontierReachabilityResult check(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace frontier_explorer
