#include "rclcpp/rclcpp.hpp"
#include "exploration_nodes/nodes/frontier_strategy_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<exploration::FrontierStrategyNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    
    rclcpp::shutdown();
    return 0;
}
