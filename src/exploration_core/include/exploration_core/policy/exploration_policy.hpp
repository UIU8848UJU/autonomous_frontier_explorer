#pragma once

#include "exploration_core/types/exploration_decision.hpp"
#include "exploration_core/types/exploration_observation.hpp"
#include "exploration_core/types/exploration_outcome.hpp"

namespace exploration_core
{

/// @brief 探索行为使用的策略抽象。
///
/// Frontier、学习策略和强化学习策略都通过这个接口接入行为执行器，行为层不依赖具体算法。
class IExplorationPolicy
{
public:
    virtual ~IExplorationPolicy() = default;

    /// @brief 开始一次新的探索任务并清理策略内部状态。
    virtual void reset() = 0;

    /// @brief 根据当前观测给出下一步行为意图。
    virtual ExplorationDecision decide(const ExplorationObservation & observation) = 0;

    /// @brief 接收执行器反馈，更新策略内部状态。
    virtual void on_outcome(const ExplorationOutcome & outcome) = 0;
};

}  // namespace exploration_core
