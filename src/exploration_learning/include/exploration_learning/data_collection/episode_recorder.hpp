#pragma once

#include <cstddef>
#include <string>

namespace exploration_learning
{

/// @brief 记录单次探索学习 episode 的生命周期和统计计数。
///
/// EpisodeRecorder 在当前骨架中只保留最小职责：维护 episode 标识和生命周期状态。
/// 后续接入真实采集后，它可以作为 transition 计数和 episode 生命周期辅助工具；
/// 数据落盘统一由 collector::DatasetWriter 承担。
class EpisodeRecorder
{
public:
  /// @brief 开始新的 episode，并重置上一轮 episode 的统计计数。
  /// @param episode_id 用于归组 recorded transitions 的稳定外部标识。
  void start_episode(const std::string & episode_id);

  /// @brief 标记当前 episode 已结束，但不清空 episode 标识。
  void finish_episode();

  /// @brief 返回当前是否有 episode 处于可记录状态。
  bool active() const;

  /// @brief 返回当前或最近一次结束的 episode 标识。
  const std::string & episode_id() const;

  /// @brief 返回当前 episode 已记录的 transition 数量。
  std::size_t transition_count() const;

private:
  /// @brief 为 true 时，输入的 observation/action 应归属到当前 episode。
  bool active_{false};

  /// @brief 用于数据集行和输出文件归组一次 rollout 的 episode 标识。
  std::string episode_id_;

  /// @brief 当前 episode 中已记录的 state-action-reward transition 数量。
  std::size_t transition_count_{0U};
};

}  // namespace exploration_learning
