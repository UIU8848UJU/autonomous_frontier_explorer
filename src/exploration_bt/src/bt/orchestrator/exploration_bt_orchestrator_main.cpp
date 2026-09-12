#include <memory>

#include "exploration_bt/exploration_bt_orchestrator_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<exploration::ExplorationBtOrchestratorNode>());
    rclcpp::shutdown();
    return 0;
}

