#include "exploration_nodes/nodes/navigation_node.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<exploration::NavigationNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
