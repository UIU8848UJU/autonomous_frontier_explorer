#include "exploration_learning/collector/data_record_plugin_factory.hpp"

#include <functional>
#include <map>

namespace exploration_learning::collector
{
namespace
{

using PluginBuilder = std::function<std::unique_ptr<IDataRecordPlugin>(
    const DataRecordPluginFactoryConfig &)>;

const std::map<std::string, PluginBuilder> & plugin_registry()
{
  static const std::map<std::string, PluginBuilder> kRegistry{
    {
      plugins::FrontierDecisionPlugin::plugin_name(),
      [](const DataRecordPluginFactoryConfig & config) {
        plugins::FrontierDecisionPluginConfig plugin_config;
        plugin_config.episode_id = config.episode_id;
        plugin_config.decision_topic = config.decision_topic;
        plugin_config.navigation_result_topic = config.navigation_result_topic;
        plugin_config.map_summary_topic = config.map_summary_topic;
        plugin_config.exploration_state_topic = config.exploration_state_topic;
        return std::make_unique<plugins::FrontierDecisionPlugin>(plugin_config);
      }
    }
  };
  return kRegistry;
}

}  // namespace

std::unique_ptr<IDataRecordPlugin> create_data_record_plugin(
  const std::string & plugin_name,
  const DataRecordPluginFactoryConfig & config)
{
  const auto & registry = plugin_registry();
  const auto it = registry.find(plugin_name);
  if (it == registry.end()) {
    return nullptr;
  }
  return it->second(config);
}

std::vector<std::string> registered_data_record_plugins()
{
  std::vector<std::string> names;
  const auto & registry = plugin_registry();
  names.reserve(registry.size());
  for (const auto & [name, _] : registry) {
    names.push_back(name);
  }
  return names;
}

}  // namespace exploration_learning::collector
