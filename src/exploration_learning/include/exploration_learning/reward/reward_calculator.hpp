#pragma once

namespace exploration_learning
{

/// @brief 单个探索 transition 的最小 reward 特征集合。
///
/// 该结构体刻意不依赖 ROS 消息，便于 reward 逻辑单元测试和离线数据工具复用。
/// ROS 消息到 reward 特征的转换应放在 RlDataCollectorNode 或专门的 adapter 中。
struct RewardInput
{
  /// @brief 相对上一个 transition 新增的已探索地图面积，单位平方米。
  double explored_area_delta{0.0};

  /// @brief 相对上一个 transition 产生的路径长度，单位米。
  double path_length_delta{0.0};

  /// @brief 当前选中的 frontier / navigation goal 是否已到达。
  bool reached_goal{false};

  /// @brief 当前 transition 是否以碰撞或安全违规结束。
  bool collision{false};
};

/// @brief 根据探索 transition 特征计算标量 reward。
///
/// 当前实现是占位性质的线性 reward：奖励新增探索面积，轻微惩罚行驶距离，
/// 奖励目标到达，并对碰撞施加较大惩罚。该类保持无状态，便于策略和数据集工具共享。
class RewardCalculator
{
public:
  /// @brief 计算单个 transition 的标量 reward。
  /// @param input 从探索 / 导航状态中提取的 transition 特征。
  /// @return 学习数据集中使用的 reward 数值。
  double calculate(const RewardInput & input) const;
};

}  // namespace exploration_learning
