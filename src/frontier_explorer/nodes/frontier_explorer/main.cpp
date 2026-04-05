#include "rclcpp/rclcpp.hpp"
#include "frontier_explorer_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<frontier_explorer::FrontierExplorerNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    
    rclcpp::shutdown();
    return 0;
}