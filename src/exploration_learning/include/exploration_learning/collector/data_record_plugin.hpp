#pragma once

#include <string>
#include <vector>

#include "exploration_learning/collector/event_buffer.hpp"

namespace exploration_learning::collector
{

/// @brief 数据 record 插件接口。
///
/// 插件只负责把 TopicEvent 和 EventBuffer 上下文转换为 JSON record。
/// 插件不负责订阅 ROS topic，也不负责写文件。
class IDataRecordPlugin
{
public:
  virtual ~IDataRecordPlugin() = default;

  /// @brief 返回插件名称。
  /// @return 插件名称，例如 frontier_decision。
  virtual std::string name() const = 0;

  /// @brief 处理一个事件并生成零条或多条 JSON record。
  /// @param event 当前事件。
  /// @param buffer 当前事件缓存，可用于查询 map/navigation 上下文。
  /// @return 完整 JSON object 字符串列表。
  virtual std::vector<std::string> handle_event(
    const TopicEvent & event,
    const EventBuffer & buffer) = 0;
};

}  // namespace exploration_learning::collector
