#pragma once

#include "frontier_strategy_core/selector/candidates/approach_goal_candidate.hpp"
#include "frontier_strategy_core/selector/candidates/robot_context.hpp"
#include "frontier_strategy_core/types/frontier_types.hpp"

namespace frontier_strategy
{

/// @brief: frontier 候选目标打分接口，用于解耦选择流程和具体评分策略
class IFrontierCandidateScorer
{
public:
    virtual ~IFrontierCandidateScorer() = default;

    virtual double score(
        const ApproachGoalCandidate & candidate,
        const FrontierCluster & cluster,
        const RobotContext & context) const = 0;
};

}  // 命名空间 frontier_strategy
