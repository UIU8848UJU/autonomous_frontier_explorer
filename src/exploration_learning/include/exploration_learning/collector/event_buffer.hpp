#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace exploration_learning::collector
{

/// @brief 数据采集底座内部统一事件结构。
struct TopicEvent
{
  /// @brief 事件来源 topic 名称。
  std::string topic_name;

  /// @brief 事件接收时的 ROS 时间戳。
  rclcpp::Time timestamp;

  /// @brief 事件载荷，必须是 JSON object 字符串。
  std::string payload_json;
};

/// @brief 按时间窗口缓存 ROS topic 事件，供 record plugin 查询上下文。
class EventBuffer
{
public:
  /// @brief 构造事件缓存。
  /// @param buffer_duration_sec 缓存保留时间窗口，单位秒。
  explicit EventBuffer(double buffer_duration_sec = 10.0);

  /// @brief 更新缓存保留时间窗口，并清理现有缓存。
  /// @param buffer_duration_sec 新缓存窗口，单位秒。
  void set_buffer_duration(double buffer_duration_sec);

  /// @brief 追加事件，并按事件时间裁剪过期数据。
  /// @param event 待追加的 topic 事件。
  void add_event(const TopicEvent & event);

  /// @brief 查询指定 topic 在时间窗口内的最近事件。
  /// @param topic_name topic 名称。
  /// @return 最近事件；不存在时返回 std::nullopt。
  std::optional<TopicEvent> latest_event(const std::string & topic_name) const;

  /// @brief 查询指定 topic 在缓存内的所有事件。
  /// @param topic_name topic 名称。
  /// @return 事件列表，按写入顺序排列。
  std::vector<TopicEvent> events_for_topic(const std::string & topic_name) const;

  /// @brief 返回当前缓存事件数量。
  /// @return 事件数量。
  std::size_t size() const;

  /// @brief 清空所有缓存事件。
  void clear();

private:
  /// @brief 根据当前时间裁剪过期事件。
  /// @param now 当前事件时间。
  void prune_locked(const rclcpp::Time & now);

  /// @brief 缓存保留窗口。
  rclcpp::Duration buffer_duration_;

  /// @brief topic 事件队列。
  std::deque<TopicEvent> events_;

  /// @brief 保护事件队列的互斥锁。
  mutable std::mutex mutex_;
};

}  // namespace exploration_learning::collector
