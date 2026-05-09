#pragma once

#include "core/selector/approach_goal_candidate.hpp"
#include "core/selector/robot_context.hpp"
#include "core/types/frontier_types.hpp"

namespace frontier_explorer
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

}  // namespace frontier_explorer
