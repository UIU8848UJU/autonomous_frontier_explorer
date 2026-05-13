#pragma once

#include <memory>
#include <string>

#include "exploration_learning/data_collection/episode_recorder.hpp"
#include "exploration_learning/dataset/dataset_writer.hpp"
#include "exploration_learning/reward/reward_calculator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace exploration_learning
{

class RlDataCollectorNode : public rclcpp::Node
{
public:
  explicit RlDataCollectorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_params();
  void load_params();

  std::string dataset_path_;
  EpisodeRecorder episode_recorder_;
  RewardCalculator reward_calculator_;
  DatasetWriter dataset_writer_;
};

}  // namespace exploration_learning
