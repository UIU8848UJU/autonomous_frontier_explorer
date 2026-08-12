#include "behaviortree_cpp_v3/bt_factory.h"
#include "frontier_explorer_nodes/nodes/bt/action/compute_frontier_candidates_action.hpp"
#include "frontier_explorer_nodes/nodes/bt/action/mark_frontier_failed_action.hpp"
#include "frontier_explorer_nodes/nodes/bt/action/navigate_to_frontier_action.hpp"
#include "frontier_explorer_nodes/nodes/bt/action/select_feasible_frontier_action.hpp"
#include "frontier_explorer_nodes/nodes/bt/is_exploration_complete_condition.hpp"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<frontier_explorer::IsExplorationCompleteCondition>(
        "IsExplorationComplete");
    factory.registerNodeType<frontier_explorer::ComputeFrontierCandidatesAction>(
        "ComputeFrontierCandidates");
    factory.registerNodeType<frontier_explorer::SelectFeasibleFrontierAction>(
        "SelectFeasibleFrontier");
    factory.registerNodeType<frontier_explorer::NavigateToFrontierAction>(
        "NavigateToFrontier");
    factory.registerNodeType<frontier_explorer::MarkFrontierFailedAction>(
        "MarkFrontierFailed");
}
