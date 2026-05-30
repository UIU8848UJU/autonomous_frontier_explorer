#include "exploration_learning/plugins/frontier_decision_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

#include "exploration_learning/collector/json_utils.hpp"

namespace exploration_learning::plugins
{
namespace
{
constexpr double kGoalMatchToleranceM = 1e-3;

std::size_t skip_ws(const std::string & json, std::size_t index)
{
  while (
      index < json.size() 
      && std::isspace(static_cast<unsigned char>(json[index]))) 
  {
    ++index;
  }
  return index;
}

std::optional<std::size_t> find_matching_json_end(
  const std::string & json,
  std::size_t start)
{
  if (start >= json.size()) {
    return std::nullopt;
  }

  const char open = json[start];
  const char close = open == '{' ? '}' : ']';
  if (open != '{' && open != '[') {
    return std::nullopt;
  }

  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (std::size_t index = start; index < json.size(); ++index) {
    const char ch = json[index];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        in_string = false;
      }
      continue;
    }

    if (ch == '"') {
      in_string = true;
      continue;
    }
    if (ch == open) {
      ++depth;
    } else if (ch == close) {
      --depth;
      if (depth == 0) {
        return index;
      }
    }
  }
  return std::nullopt;
}

std::optional<std::string> extract_json_value(
  const std::string & json,
  const std::string & key)
{
  const std::string key_token = "\"" + key + "\"";
  const auto key_pos = json.find(key_token);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }

  const auto colon_pos = json.find(':', key_pos + key_token.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = skip_ws(json, colon_pos + 1U);
  if (value_start >= json.size()) {
    return std::nullopt;
  }

  const char first = json[value_start];
  if (first == '{' || first == '[') {
    const auto end = find_matching_json_end(json, value_start);
    if (!end.has_value()) {
      return std::nullopt;
    }
    return json.substr(value_start, end.value() - value_start + 1U);
  }

  if (first == '"') {
    bool escaped = false;
    for (std::size_t index = value_start + 1U; index < json.size(); ++index) {
      const char ch = json[index];
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        return json.substr(value_start, index - value_start + 1U);
      }
    }
    return std::nullopt;
  }

  std::size_t value_end = value_start;
  while (value_end < json.size() && json[value_end] != ',' && json[value_end] != '}') {
    ++value_end;
  }
  while (value_end > value_start &&
    std::isspace(static_cast<unsigned char>(json[value_end - 1U])))
  {
    --value_end;
  }
  return json.substr(value_start, value_end - value_start);
}

std::optional<std::string> extract_json_string(
  const std::string & json,
  const std::string & key)
{
  const auto value = extract_json_value(json, key);
  if (!value.has_value() || value->size() < 2U || value->front() != '"' ||
    value->back() != '"')
  {
    return std::nullopt;
  }
  return value->substr(1U, value->size() - 2U);
}

std::optional<double> extract_json_number(
  const std::string & json,
  const std::string & key)
{
  const auto value = extract_json_value(json, key);
  if (!value.has_value()) {
    return std::nullopt;
  }

  char * end = nullptr;
  const double parsed = std::strtod(value->c_str(), &end);
  if (end == value->c_str()) {
    return std::nullopt;
  }
  return parsed;
}

bool extract_json_bool_or_default(
  const std::string & json,
  const std::string & key,
  bool default_value)
{
  const auto value = extract_json_value(json, key);
  if (!value.has_value()) {
    return default_value;
  }
  if (*value == "true") {
    return true;
  }
  if (*value == "false") {
    return false;
  }
  return default_value;
}

std::vector<std::string> split_json_object_array(const std::string & array_json)
{
  std::vector<std::string> objects;
  std::size_t index = 0U;
  while (index < array_json.size()) {
    index = skip_ws(array_json, index);
    if (index >= array_json.size()) {
      break;
    }
    if (array_json[index] != '{') {
      ++index;
      continue;
    }
    const auto end = find_matching_json_end(array_json, index);
    if (!end.has_value()) {
      break;
    }
    objects.push_back(array_json.substr(index, end.value() - index + 1U));
    index = end.value() + 1U;
  }
  return objects;
}

std::string candidate_id_from_json(const std::string & candidate_json, std::size_t fallback_index)
{
  const auto value = extract_json_value(candidate_json, "candidate_id");
  if (!value.has_value()) {
    return std::to_string(fallback_index);
  }
  if (value->size() >= 2U && value->front() == '"' && value->back() == '"') {
    return value->substr(1U, value->size() - 2U);
  }
  return *value;
}

}  // namespace

FrontierDecisionPlugin::FrontierDecisionPlugin(const FrontierDecisionPluginConfig & config)
: config_(config)
{
}

std::string FrontierDecisionPlugin::plugin_name()
{
  return "frontier_decision";
}

std::string FrontierDecisionPlugin::name() const
{
  return plugin_name();
}

std::vector<std::string> FrontierDecisionPlugin::handle_event(
  const collector::TopicEvent & event,
  const collector::EventBuffer & buffer)
{
  if (event.topic_name == config_.decision_topic) {
    return handle_decision_event(event, buffer);
  }
  if (event.topic_name == config_.navigation_result_topic) {
    return handle_navigation_event(event, buffer);
  }
  return {};
}

std::vector<std::string> FrontierDecisionPlugin::handle_decision_event(
  const collector::TopicEvent & event,
  const collector::EventBuffer & buffer)
{
  const auto raw_json = extract_raw_json(event.payload_json);
  if (!raw_json.has_value()) {
    latest_decision_context_json_ = event.payload_json;
    return {build_decision_record(event, buffer, "", false, event.payload_json, "null")};
  }

  const auto event_name = extract_json_string(raw_json.value(), "event").value_or("");
  if (event_name == "frontier_candidates") {
    update_candidates_from_frontier_json(raw_json.value());
    latest_decision_context_json_ = event.payload_json;
    return {build_decision_record(event, buffer, "", false, event.payload_json, "null")};
  }

  if (event_name == "next_frontier_goal") {
    const auto selected_candidate_id = match_selected_candidate_id(raw_json.value());
    latest_decision_context_json_ = event.payload_json;
    return {build_decision_record(
        event,
        buffer,
        selected_candidate_id,
        false,
        event.payload_json,
        "null")};
  }

  latest_decision_context_json_ = event.payload_json;
  return {build_decision_record(event, buffer, "", false, event.payload_json, "null")};
}

std::vector<std::string> FrontierDecisionPlugin::handle_navigation_event(
  const collector::TopicEvent & event,
  const collector::EventBuffer & buffer)
{
  const auto raw_json = extract_raw_json(event.payload_json);
  if (!raw_json.has_value()) {
    return {};
  }

  update_candidate_from_navigation_json(raw_json.value());

  const auto event_name = extract_json_string(raw_json.value(), "event").value_or("");
  if (event_name != "navigation_result") {
    return {};
  }

  const auto selected_candidate_id = match_selected_candidate_id(raw_json.value());
  if (selected_candidate_id.empty()) {
    return {};
  }
  return {build_decision_record(
      event,
      buffer,
      selected_candidate_id,
      true,
      latest_decision_context_json_,
      event.payload_json)};
}

std::string FrontierDecisionPlugin::build_decision_record(
  const collector::TopicEvent & event,
  const collector::EventBuffer & buffer,
  const std::string & selected_candidate_id,
  bool has_outcome,
  const std::string & decision_context_json,
  const std::string & outcome_context_json)
{
  const auto map_context = buffer.latest_event(config_.map_summary_topic);
  const auto exploration_state_context = buffer.latest_event(config_.exploration_state_topic);

  std::ostringstream json;
  json << std::fixed << std::setprecision(6)
       << "{"
       << "\"record_type\":\"frontier_decision\","
       << "\"schema_version\":2,"
       << "\"episode_id\":\"" << collector::escape_json_string(config_.episode_id) << "\","
       << "\"decision_id\":" << next_decision_id_++ << ","
       << "\"timestamp_sec\":" << event.timestamp.seconds() << ","
       << "\"selected_candidate_id\":\""
       << collector::escape_json_string(selected_candidate_id) << "\","
       << "\"has_outcome\":" << (has_outcome ? "true" : "false") << ","
       << "\"candidates\":" << candidates_to_json(selected_candidate_id) << ","
       << "\"decision_context\":" << decision_context_json << ","
       << "\"map_context\":"
       << (map_context.has_value() ? map_context->payload_json : "null") << ","
       << "\"outcome_context\":" << outcome_context_json << ","
       << "\"exploration_state_context\":"
       << (exploration_state_context.has_value() ?
  exploration_state_context->payload_json : "null")
       << ","
       << "\"extra\":{"
       << "\"source\":\"frontier_decision_plugin\","
       << "\"candidate_schema_status\":\"parsed_debug_json\","
       << "\"trigger_topic\":\"" << collector::escape_json_string(event.topic_name) << "\""
       << "}"
       << "}";
  return json.str();
}

void FrontierDecisionPlugin::update_candidates_from_frontier_json(const std::string & raw_json)
{
  const auto candidates_value = extract_json_value(raw_json, "candidates");
  if (!candidates_value.has_value()) {
    latest_candidates_.clear();
    return;
  }

  latest_candidates_.clear();
  const auto candidate_objects = split_json_object_array(candidates_value.value());
  latest_candidates_.reserve(candidate_objects.size());
  for (std::size_t index = 0U; index < candidate_objects.size(); ++index) {
    const auto & candidate_json = candidate_objects[index];
    CandidateSnapshot candidate;
    candidate.candidate_id = candidate_id_from_json(candidate_json, index);
    candidate.x = extract_json_number(candidate_json, "x").value_or(0.0);
    candidate.y = extract_json_number(candidate_json, "y").value_or(0.0);
    candidate.score_total = extract_json_number(candidate_json, "score").value_or(0.0);
    candidate.distance_to_robot =
      extract_json_number(candidate_json, "distance_m").value_or(0.0);
    candidate.clearance = extract_json_number(candidate_json, "clearance_m").value_or(0.0);
    candidate.unknown_ratio =
      extract_json_number(candidate_json, "unknown_ratio").value_or(0.0);
    candidate.path_length = extract_json_number(candidate_json, "path_length_m").value_or(0.0);
    candidate.path_length_valid = candidate.path_length > 0.0;
    candidate.reachable = extract_json_bool_or_default(candidate_json, "reachable", true);
    candidate.source_json = candidate_json;
    latest_candidates_.push_back(candidate);
  }
}

void FrontierDecisionPlugin::update_candidate_from_navigation_json(const std::string & raw_json)
{
  const auto matched_id = match_selected_candidate_id(raw_json);
  if (matched_id.empty()) {
    return;
  }

  const auto path_length = extract_json_number(raw_json, "path_length_m");
  const auto reachable = extract_json_bool_or_default(raw_json, "accepted_or_feasible", true);
  for (auto & candidate : latest_candidates_) {
    if (candidate.candidate_id != matched_id) {
      continue;
    }
    if (path_length.has_value() && path_length.value() > 0.0) {
      candidate.path_length = path_length.value();
      candidate.path_length_valid = true;
    }
    candidate.reachable = reachable;
    return;
  }
}

std::string FrontierDecisionPlugin::match_selected_candidate_id(const std::string & raw_json) const
{
  std::optional<double> goal_x;
  std::optional<double> goal_y;

  const auto selected_json = extract_json_value(raw_json, "selected");
  if (selected_json.has_value() && selected_json.value() != "null") {
    goal_x = extract_json_number(selected_json.value(), "x");
    goal_y = extract_json_number(selected_json.value(), "y");
  }

  if (!goal_x.has_value() || !goal_y.has_value()) {
    const auto goal_json = extract_json_value(raw_json, "goal");
    if (goal_json.has_value()) {
      goal_x = extract_json_number(goal_json.value(), "x");
      goal_y = extract_json_number(goal_json.value(), "y");
    }
  }

  if (!goal_x.has_value() || !goal_y.has_value()) {
    return "";
  }

  double best_distance = std::numeric_limits<double>::max();
  std::string best_id;
  for (const auto & candidate : latest_candidates_) {
    const double distance = std::hypot(candidate.x - goal_x.value(), candidate.y - goal_y.value());
    if (distance < best_distance) {
      best_distance = distance;
      best_id = candidate.candidate_id;
    }
  }

  return best_distance <= kGoalMatchToleranceM ? best_id : "";
}

std::optional<std::string> FrontierDecisionPlugin::extract_raw_json(
  const std::string & payload_json) const
{
  return extract_json_value(payload_json, "raw_json");
}

std::string FrontierDecisionPlugin::candidates_to_json(
  const std::string & selected_candidate_id) const
{
  std::ostringstream json;
  json << std::fixed << std::setprecision(6) << "[";
  for (std::size_t index = 0U; index < latest_candidates_.size(); ++index) {
    const auto & candidate = latest_candidates_[index];
    if (index > 0U) {
      json << ",";
    }
    const bool selected =
      !selected_candidate_id.empty() && candidate.candidate_id == selected_candidate_id;
    json << "{"
         << "\"candidate_id\":\"" << collector::escape_json_string(candidate.candidate_id)
         << "\","
         << "\"score_total\":" << candidate.score_total << ","
         << "\"score_breakdown\":{},"
         << "\"distance_to_robot\":" << candidate.distance_to_robot << ","
         << "\"clearance\":" << candidate.clearance << ","
         << "\"unknown_ratio\":" << candidate.unknown_ratio << ","
         << "\"path_length\":";
    if (candidate.path_length_valid) {
      json << candidate.path_length;
    } else {
      json << "null";
    }
    json << ","
         << "\"path_length_valid\":" << (candidate.path_length_valid ? "true" : "false") << ","
         << "\"reachable\":" << (candidate.reachable ? "true" : "false") << ","
         << "\"selected\":" << (selected ? "true" : "false") << ","
         << "\"reject_reason\":\"" << (selected ? "" : "not_selected") << "\","
         << "\"source_candidate\":" << candidate.source_json
         << "}";
  }
  json << "]";
  return json.str();
}

}  // namespace exploration_learning::plugins
