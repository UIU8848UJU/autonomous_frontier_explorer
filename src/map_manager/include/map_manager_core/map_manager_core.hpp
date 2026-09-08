#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace map_manager
{

/// @brief Core 使用的二维栅格地图快照。
struct MapSnapshot
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double resolution{0.0};
  std::vector<std::int8_t> data;
};

/// @brief Core 使用的探索阶段。
enum class ExplorationPhase : std::uint8_t
{
  IDLE = 0U,
  RUNNING = 1U,
  STOPPED = 2U,
  COMPLETED = 3U,
  STUCK = 4U,
  UNKNOWN = 255U,
};

/// @brief Core 使用的探索状态快照。
struct ExplorationState
{
  ExplorationPhase phase{ExplorationPhase::IDLE};
  std::string detail;
};

/// @brief Core 使用的探索完成状态值。
constexpr ExplorationPhase kExplorationCompleted = ExplorationPhase::COMPLETED;

/// @brief 当前地图统计信息。
struct MapStatistics
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double resolution{0.0};
  double unknown_ratio{1.0};
  bool valid{false};
};

/// @brief 管理地图快照、探索完成状态和保存生命周期的核心状态对象。
///
/// 该类不创建 ROS 通信对象，也不发布消息；节点层负责将 ROS 回调转成状态更新，
/// 并根据本类的结果完成通信和日志输出。
class MapManagerCore
{
public:
  /// @brief 接收一份最新地图并更新统计信息。
  void update_map(MapSnapshot map);

  /// @brief 接收最新探索状态。
  void update_exploration_state(ExplorationState state);

  /// @brief 判断当前是否满足自动保存触发条件。
  bool should_save(bool enable_auto_save) const;

  /// @brief 标记本次地图生命周期已经确认探索完成。
  void mark_completion_detected();

  /// @brief 标记保存请求已发出。
  void mark_save_requested();

  /// @brief 记录一次保存结果。
  void record_save_result(bool success, const std::string & map_url);

  /// @brief 计算地图中 unknown cell 的比例。
  static double calculate_unknown_ratio(const MapSnapshot & map);

  const MapSnapshot & latest_map() const;
  const MapStatistics & map_statistics() const;
  ExplorationPhase last_exploration_state() const;
  const std::string & last_exploration_detail() const;
  const std::string & saved_map_url() const;
  bool completion_detected() const;
  bool save_requested() const;
  bool save_succeeded() const;

private:
  MapSnapshot latest_map_;
  MapStatistics map_stats_;
  ExplorationPhase last_exploration_state_{ExplorationPhase::IDLE};
  std::string last_exploration_detail_;
  std::string saved_map_url_;
  bool completion_detected_{false};
  bool save_requested_{false};
  bool save_succeeded_{false};
};

}  // namespace map_manager
