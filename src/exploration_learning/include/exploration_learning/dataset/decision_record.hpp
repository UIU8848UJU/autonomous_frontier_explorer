#pragma once

#include <cstdint>
#include <string>

namespace exploration_learning
{

/// @brief 单个 frontier 决策候选的轻量记录结构。
///
/// 该结构体只保存后续数据集采集最基础的决策字段，不绑定 ROS 消息，
/// 也不依赖序列化库。真实写盘格式由 DatasetRecorderNode 或后续 DatasetWriter 决定。
struct DecisionRecord
{
  /// @brief 当前 episode 的唯一标识。
  std::string episode_id;

  /// @brief 当前 episode 内的决策步编号。
  std::uint64_t step_id{0U};

  /// @brief 当前候选 frontier 的编号。
  std::uint64_t candidate_id{0U};

  /// @brief 当前候选是否被最终选中。
  bool selected{false};

  /// @brief frontier 决策层给出的候选评分。
  double score{0.0};

  /// @brief 候选被拒绝时的原因；候选被选中时可为空。
  std::string reject_reason;
};

}  // namespace exploration_learning
