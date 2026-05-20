#pragma once

#include <memory>
#include <string>

#include "exploration_learning/data_collection/episode_recorder.hpp"
#include "exploration_learning/dataset/dataset_writer.hpp"
#include "exploration_learning/reward/reward_calculator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace exploration_learning
{

/// @brief 探索学习数据采集的 ROS 2 节点入口。
///
/// 该节点是探索系统与学习数据管线之间的集成边界。当前实现只负责加载配置并
/// 连接骨架组件；后续应接入探索状态、地图/frontier 特征、导航结果和动作决策。
class RlDataCollectorNode : public rclcpp::Node
{
public:
  /// @brief 构造数据采集节点并加载运行参数。
  /// @param options launch 或 composition 传入的标准 ROS 2 节点选项。
  explicit RlDataCollectorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// @brief 声明数据采集节点持有的 ROS 参数。
  void declare_params();

  /// @brief 读取 ROS 参数，并应用到 recorder / writer 等组件。
  void load_params();

  /// @brief DatasetWriter 生成数据集产物时使用的根路径或数据集名称。
  std::string dataset_path_;

  /// @brief 跟踪 episode 生命周期和 transition 级采集状态。
  EpisodeRecorder episode_recorder_;

  /// @brief 将探索进展和导航结果转换为标量 reward。
  RewardCalculator reward_calculator_;

  /// @brief 持有数据集输出配置，并在后续扩展中负责持久化行为。
  DatasetWriter dataset_writer_;
};

}  // namespace exploration_learning
