#pragma once

namespace frontier_explorer
{
namespace exploration_bt_defaults
{

/// @brief: 默认 frontier goal 计算服务名，生产部署时优先通过 YAML 参数覆盖
inline constexpr char kFrontierGoalService[] =
    "/frontier_explorer_node/get_next_frontier_goal";

/// @brief: 默认 frontier 失败标记服务名，生产部署时优先通过 YAML 参数覆盖
inline constexpr char kMarkFailedService[] =
    "/frontier_explorer_node/mark_frontier_failed";

/// @brief: 默认 Nav2 NavigateToPose action 名称
inline constexpr char kNavigateToPoseAction[] = "navigate_to_pose";

/// @brief: 默认 BT tick 周期，单位秒
inline constexpr double kTickPeriodSec = 0.1;

/// @brief: 默认服务不可用时的重试间隔，单位秒
inline constexpr double kServiceRetryDelaySec = 2.0;

}  // namespace exploration_bt_defaults
}  // namespace frontier_explorer
