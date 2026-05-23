#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "exploration_learning/collector/data_record_plugin.hpp"

namespace exploration_learning::plugins
{

/// @brief Frontier decision 数据采集插件配置。
struct FrontierDecisionPluginConfig
{
  /// @brief 当前 episode 标识。
  std::string episode_id;

  /// @brief frontier 决策调试 JSON topic。
  std::string decision_topic;

  /// @brief 导航结果调试 JSON topic。
  std::string navigation_result_topic;

  /// @brief 地图摘要事件的内部 topic 名称。
  std::string map_summary_topic;

  /// @brief 探索状态事件的内部 topic 名称。
  std::string exploration_state_topic;
};

/// @brief 将 frontier decision debug event 转换为训练用 decision record。
///
/// 第一版不解析上游 JSON 的内部字段，而是把 frontier decision、最近地图摘要、
/// 最近导航结果和探索状态作为上下文稳定写入，后续可在该插件内替换为强 schema 解析。
class FrontierDecisionPlugin : public collector::IDataRecordPlugin
{
public:
  /// @brief 构造 frontier decision 插件。
  /// @param config 插件运行配置。
  explicit FrontierDecisionPlugin(const FrontierDecisionPluginConfig & config);

  /// @brief 返回插件名称。
  /// @return frontier_decision。
  std::string name() const override;

  /// @brief 处理 topic event 并在 frontier decision 事件到达时生成 record。
  /// @param event 当前事件。
  /// @param buffer 事件缓存。
  /// @return decision record JSON 列表。
  std::vector<std::string> handle_event(
    const collector::TopicEvent & event,
    const collector::EventBuffer & buffer) override;

private:
  /// @brief 从缓存中读取最近上下文并拼装 JSON record。
  /// @param event 当前 frontier decision 事件。
  /// @param buffer 事件缓存。
  /// @return 完整 decision record JSON object 字符串。
  std::string build_decision_record(
    const collector::TopicEvent & event,
    const collector::EventBuffer & buffer);

  /// @brief 插件配置。
  FrontierDecisionPluginConfig config_;

  /// @brief 当前 episode 内递增的 decision id。
  std::uint64_t next_decision_id_{0U};
};

}  // namespace exploration_learning::plugins
