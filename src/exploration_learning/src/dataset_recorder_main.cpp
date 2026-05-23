#include <memory>

#include "exploration_learning/collector/dataset_recorder_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<exploration_learning::collector::DatasetRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
