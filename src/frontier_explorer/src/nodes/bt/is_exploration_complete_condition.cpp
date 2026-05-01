#include "nodes/bt/is_exploration_complete_condition.hpp"

namespace frontier_explorer
{

IsExplorationCompleteCondition::IsExplorationCompleteCondition(
    const std::string & name,
    const BT::NodeConfiguration & config,
    const std::shared_ptr<ExplorationBtContext> & context)
: BT::ConditionNode(name, config), context_(context)
{
}

BT::NodeStatus IsExplorationCompleteCondition::tick()
{
    std::lock_guard<std::mutex> lock(context_->mutex);
    return context_->exploration_complete ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace frontier_explorer
