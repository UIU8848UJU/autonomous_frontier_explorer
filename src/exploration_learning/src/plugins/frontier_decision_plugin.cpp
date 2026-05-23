#include "exploration_learning/plugins/frontier_decision_plugin.hpp"

#include <iomanip>
#include <sstream>

#include "exploration_learning/collector/json_utils.hpp"

namespace exploration_learning::plugins
{

FrontierDecisionPlugin::FrontierDecisionPlugin(const FrontierDecisionPluginConfig & config)
: config_(config)
{
}

std::string FrontierDecisionPlugin::name() const
{
  return "frontier_decision";
}

std::vector<std::string> FrontierDecisionPlugin::handle_event(
  const collector::TopicEvent & event,
  const collector::EventBuffer & buffer)
{
  if (event.topic_name != config_.decision_topic) {
    return {};
  }

  return {build_decision_record(event, buffer)};
}

std::string FrontierDecisionPlugin::build_decision_record(
  const collector::TopicEvent & event,
  const collector::EventBuffer & buffer)
{
  const auto navigation_context = buffer.latest_event(config_.navigation_result_topic);
  const auto map_context = buffer.latest_event(config_.map_summary_topic);
  const auto exploration_state_context = buffer.latest_event(config_.exploration_state_topic);

  std::ostringstream json;
  json << std::fixed << std::setprecision(6)
       << "{"
       << "\"record_type\":\"frontier_decision\","
       << "\"schema_version\":1,"
       << "\"episode_id\":\"" << collector::escape_json_string(config_.episode_id) << "\","
       << "\"decision_id\":" << next_decision_id_++ << ","
       << "\"timestamp_sec\":" << event.timestamp.seconds() << ","
       << "\"selected_candidate_id\":\"\","
       << "\"candidates\":[],"
       << "\"frontier_context\":" << event.payload_json << ","
       << "\"map_context\":"
       << (map_context.has_value() ? map_context->payload_json : "null") << ","
       << "\"outcome_context\":"
       << (navigation_context.has_value() ? navigation_context->payload_json : "null") << ","
       << "\"exploration_state_context\":"
       << (exploration_state_context.has_value() ?
  exploration_state_context->payload_json : "null")
       << ","
       << "\"extra\":{"
       << "\"source\":\"frontier_decision_plugin\","
       << "\"candidate_schema_status\":\"raw_debug_json_mvp\""
       << "}"
       << "}";
  return json.str();
}

}  // namespace exploration_learning::plugins
