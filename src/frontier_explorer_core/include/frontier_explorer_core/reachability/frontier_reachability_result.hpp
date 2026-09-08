#pragma once

#include <string>

namespace frontier_explorer
{

/// @brief: planner 可达性诊断结果，不携带 ROS 消息类型。
struct FrontierReachabilityResult
{
    bool checked{false};
    bool reachable{true};
    double path_length_m{0.0};
    std::string reason;
};

}  // namespace frontier_explorer
