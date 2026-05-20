#pragma once

#include <string>

namespace exploration_learning
{

/// @brief 保存数据集输出配置，并承载后续的数据持久化逻辑。
///
/// DatasetWriter 当前是轻量占位实现。后续应扩展为以稳定磁盘格式写入 transition 行、
/// episode 元数据、地图快照以及 train / validation 划分。
class DatasetWriter
{
public:
  /// @brief 设置生成学习数据时使用的输出位置。
  /// @param dataset_path 由 ROS 参数配置的数据集目录或数据集名称。
  void configure(const std::string & dataset_path);

  /// @brief 返回当前配置的数据集输出位置。
  const std::string & dataset_path() const;

private:
  /// @brief 后续写入样本时使用的目录或逻辑数据集名称。
  std::string dataset_path_;
};

}  // namespace exploration_learning
