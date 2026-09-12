#include "behaviortree_cpp_v3/bt_factory.h"
#include "exploration_bt/bt/action/compute_frontier_candidates_action.hpp"
#include "exploration_bt/bt/action/mark_frontier_failed_action.hpp"
#include "exploration_bt/bt/action/navigate_to_frontier_action.hpp"
#include "exploration_bt/bt/action/select_feasible_frontier_action.hpp"
#include "exploration_bt/bt/is_exploration_complete_condition.hpp"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<exploration::IsExplorationCompleteCondition>(
        "IsExplorationComplete");
    factory.registerNodeType<exploration::ComputeFrontierCandidatesAction>(
        "ComputeFrontierCandidates");
    factory.registerNodeType<exploration::SelectFeasibleFrontierAction>(
        "SelectFeasibleFrontier");
    factory.registerNodeType<exploration::NavigateToFrontierAction>(
        "NavigateToFrontier");
    factory.registerNodeType<exploration::MarkFrontierFailedAction>(
        "MarkFrontierFailed");
}
