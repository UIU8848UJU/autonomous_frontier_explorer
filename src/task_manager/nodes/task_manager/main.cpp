#include "rclcpp/rclcpp.hpp"
#include "task_manager/task_manager_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<task_manager::TaskManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}