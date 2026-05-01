#include "nodes/exploration_bt_context.hpp"

namespace frontier_explorer
{

rclcpp::Time ExplorationBtContext::now() const
{
    return node ? node->now() : rclcpp::Clock().now();
}

}  // namespace frontier_explorer
