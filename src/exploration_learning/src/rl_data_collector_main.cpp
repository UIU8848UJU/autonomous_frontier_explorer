#include <memory>

#include "exploration_learning/data_collection/rl_data_collector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<exploration_learning::RlDataCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
