#pragma once

#include <memory>
#include <string>
#include <vector>

#include "exploration_learning/collector/data_record_plugin.hpp"
#include "exploration_learning/plugins/frontier_decision_plugin.hpp"

namespace exploration_learning::collector
{

/// @brief record plugin 工厂配置。
struct DataRecordPluginFactoryConfig
{
  /// @brief 当前 episode 标识。
  std::string episode_id;

  /// @brief frontier 决策调试 JSON topic。
  std::string decision_topic;

  /// @brief 导航结果调试 JSON topic。
  std::string navigation_result_topic;

  /// @brief 地图摘要事件的内部 topic 名称。
  std::string map_summary_topic;

  /// @brief 探索状态 topic。
  std::string exploration_state_topic;
};

/// @brief 创建数据 record plugin。
/// @param plugin_name 配置中指定的插件名。
/// @param config 插件创建配置。
/// @return 插件实例；插件名未知时返回 nullptr。
std::unique_ptr<IDataRecordPlugin> create_data_record_plugin(
  const std::string & plugin_name,
  const DataRecordPluginFactoryConfig & config);

/// @brief 获取当前支持的插件名称列表。
/// @return 已注册插件名称。
std::vector<std::string> registered_data_record_plugins();

}  // namespace exploration_learning::collector
