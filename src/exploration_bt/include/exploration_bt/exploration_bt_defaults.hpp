#pragma once

namespace exploration
{
namespace exploration_bt_defaults
{

/// @brief: 默认前沿候选列表服务名，生产部署时优先通过 YAML 参数覆盖
inline constexpr char kFrontierCandidatesService[] =
    "/frontier_strategy_node/get_frontier_candidates";

/// @brief: 默认前沿失败标记服务名，生产部署时优先通过 YAML 参数覆盖
inline constexpr char kMarkFailedService[] =
    "/frontier_strategy_node/mark_frontier_failed";

/// @brief: 默认目标导航可执行性检查服务名
inline constexpr char kGoalFeasibilityService[] =
    "/navigation_node/check_goal_feasibility";

/// @brief: 默认 NavigationNode 对外导航动作名称
inline constexpr char kNavigationAction[] = "/navigation_node/navigate_to_pose";

/// @brief: 默认 Nav2 NavigateToPose 动作名称，仅由 NavigationNode 内部使用
inline constexpr char kNavigateToPoseAction[] = "navigate_to_pose";

/// @brief: 默认 BT 执行周期，单位秒
inline constexpr double kTickPeriodSec = 0.1;

/// @brief: 默认服务不可用时的重试间隔，单位秒
inline constexpr double kServiceRetryDelaySec = 2.0;

/// @brief: 默认每轮最多请求的前沿候选数
inline constexpr int kMaxFrontierCandidates = 8;

/// @brief: 默认每批进行 Nav2 可行性检查的候选数量
inline constexpr int kFeasibilityTopK = 3;

/// @brief: 是否在当前导航期间准备下一轮候选
inline constexpr bool kEnableCandidatePrefetch = true;

/// @brief: 预取候选允许使用的最大年龄，单位秒
inline constexpr double kPrefetchMaxAgeSec = 1.0;

/// @brief 是否启用同一地图/起点区域内的短期可行性结果缓存
inline constexpr bool kEnableFeasibilityCache = true;

/// @brief 可行性结果缓存有效期，单位秒
inline constexpr double kFeasibilityCacheTtlSec = 1.0;

/// @brief 目标位置量化网格边长，单位 m
inline constexpr double kFeasibilityCacheRegionSizeM = 0.5;

/// @brief 是否允许在导航期间切换到经过复核且明显更优的新目标；默认关闭
inline constexpr bool kEnableActiveGoalReplacement = false;

/// @brief 触发目标切换所需的最小效用提升
inline constexpr double kGoalSwitchMinUtilityGain = 0.25;

/// @brief 当前目标至少保持的时间，单位秒
inline constexpr double kGoalMinHoldDurationSec = 2.0;

/// @brief 单次导航最多允许的目标切换次数
inline constexpr int kMaxGoalSwitchesPerNavigation = 1;

/// @brief 目标替换时使用的到达容差，单位 m
inline constexpr double kActiveGoalReachedToleranceM = 0.05;

/// @brief: 默认单个候选发生可恢复可执行性失败时的重试次数
inline constexpr int kMaxFeasibilityRecoverableRetries = 2;

/// @brief: 默认可执行候选选择时的路径长度惩罚权重
inline constexpr double kFeasiblePathLengthWeight = 0.6;

}  // 命名空间 exploration_bt_defaults
}  // 命名空间 exploration
