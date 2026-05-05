#include "nodes/exploration_bt_context.hpp"

#include <stdexcept>

namespace frontier_explorer
{

rclcpp::Time ExplorationBtContext::now() const
{
    return node ? node->now() : rclcpp::Clock().now();
}

std::shared_ptr<ExplorationBtContext> get_exploration_bt_context(
    const BT::NodeConfiguration & config)
{
    if (!config.blackboard) {
        throw std::runtime_error("Exploration BT blackboard is not available");
    }

    auto context = config.blackboard->get<std::shared_ptr<ExplorationBtContext>>(
        kExplorationBtContextBlackboardKey);
    if (!context) {
        throw std::runtime_error("Exploration BT context is not available");
    }
    return context;
}

}  // namespace frontier_explorer
