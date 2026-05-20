#include "exploration_learning/data_collection/rl_data_collector_node.hpp"

namespace exploration_learning
{

RlDataCollectorNode::RlDataCollectorNode(const rclcpp::NodeOptions & options)
: Node("rl_data_collector_node", options)
{
  declare_params();
  load_params();
}

void RlDataCollectorNode::declare_params()
{
  declare_parameter<std::string>("dataset_path", "exploration_dataset");
}

void RlDataCollectorNode::load_params()
{
  dataset_path_ = get_parameter("dataset_path").as_string();
  dataset_writer_.configure(dataset_path_);
}

}  // namespace exploration_learning
