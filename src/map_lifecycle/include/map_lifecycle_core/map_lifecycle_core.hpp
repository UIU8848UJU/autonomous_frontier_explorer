#pragma once

#include <cstdint>
#include <string>
#include "grid_map_core/types/grid_map.hpp"

namespace map_lifecycle
{

/// @brief Core 使用的地图生命周期状态。
enum class MapLifecycleState : std::uint8_t
{
  EMPTY = 0U,
  ACTIVE = 1U,
  READY = 2U,
  SAVING = 3U,
  SAVED = 4U,
  FAILED = 5U,
  UNKNOWN = 255U,
};

/// @brief ROS 无关的探索完成事件。
struct ExplorationCompletedEvent
{
  std::string detail;
};

/// @brief 当前地图统计信息。
struct MapStatistics
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double resolution{0.0};
  double unknown_ratio{1.0};
  // 仅记录进度；探索完成由 ExplorationCompletedEvent 驱动。
  double explored_ratio{0.0};
  double free_ratio{0.0};
  double occupied_ratio{0.0};
  bool valid{false};
};

/// @brief 管理地图状态、地图进度和保存生命周期的核心状态对象。
///
/// 该类不创建 ROS 通信对象，也不发布消息；节点层负责将 ROS 回调转成状态更新，
/// 并根据本类的结果完成通信和日志输出。
class MapLifecycleCore
{
public:
  /// @brief 接收一份最新地图并更新统计信息。
  void update_map(grid_map_core::GridMap map);

  /// @brief 接收探索完成事件。
  void handle_exploration_completed(const ExplorationCompletedEvent & event);

  /// @brief 判断当前是否满足自动保存触发条件。
  bool should_save(bool enable_auto_save) const;

  /// @brief 标记保存请求已发出。
  void mark_save_requested();

  /// @brief 记录一次保存结果。
  void record_save_result(bool success, const std::string & map_url);

  /// @brief 计算地图中 unknown cell 的比例。
  static double calculate_unknown_ratio(const grid_map_core::GridMap & map);

  const grid_map_core::GridMap & latest_map() const;
  const MapStatistics & map_statistics() const;
  MapLifecycleState state() const;
  const std::string & saved_map_url() const;
  bool completion_detected() const;

private:
  grid_map_core::GridMap latest_map_;
  MapStatistics map_stats_;
  MapLifecycleState state_{MapLifecycleState::EMPTY};
  std::string saved_map_url_;
  bool completion_detected_{false};
};

}  // 命名空间 map_lifecycle
