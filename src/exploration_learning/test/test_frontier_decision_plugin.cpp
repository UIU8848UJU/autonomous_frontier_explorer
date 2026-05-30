#include "exploration_learning/plugins/frontier_decision_plugin.hpp"

#include "exploration_learning/collector/event_buffer.hpp"
#include "exploration_learning/collector/json_utils.hpp"
#include "gtest/gtest.h"

namespace exploration_learning::plugins
{

TEST(FrontierDecisionPluginTest, ExpandsCandidatesAndMatchesSelectedGoal)
{
  FrontierDecisionPluginConfig config;
  config.episode_id = "episode_001";
  config.decision_topic = "/frontier_explorer/decision_debug_json";
  config.navigation_result_topic = "/navigation/navigation_result_debug_json";
  config.map_summary_topic = "__map_summary";
  config.exploration_state_topic = "/frontier_explorer/state";

  FrontierDecisionPlugin plugin(config);
  collector::EventBuffer buffer(10.0);

  const std::string candidates_raw =
    R"({"event":"frontier_candidates","success":true,"candidates":[)"
    R"({"candidate_id":0,"x":1.0,"y":2.0,"score":0.7,"distance_m":3.0,)"
    R"("clearance_m":0.5,"unknown_ratio":0.2,"reachable":true,"path_length_m":0.0},)"
    R"({"candidate_id":1,"x":4.0,"y":5.0,"score":0.9,"distance_m":6.0,)"
    R"("clearance_m":0.8,"unknown_ratio":0.1,"reachable":true,"path_length_m":0.0})"
    R"(]})";
  const collector::TopicEvent candidates_event{
    config.decision_topic,
    rclcpp::Time(1, 0, RCL_ROS_TIME),
    collector::wrap_raw_payload(candidates_raw)};
  buffer.add_event(candidates_event);

  const auto candidate_records = plugin.handle_event(candidates_event, buffer);
  ASSERT_EQ(candidate_records.size(), 1U);
  EXPECT_NE(candidate_records.front().find("\"candidates\":["), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"candidate_id\":\"0\""), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"candidate_id\":\"1\""), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"has_outcome\":false"), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"path_length\":null"), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"path_length_valid\":false"), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"decision_context\""), std::string::npos);
  EXPECT_NE(candidate_records.front().find("\"outcome_context\":null"), std::string::npos);

  const std::string feasibility_raw =
    R"({"event":"feasibility_check","success":true,"accepted_or_feasible":true,)"
    R"("goal":{"x":4.0,"y":5.0,"z":0.0},"path_length_m":2.5})";
  const collector::TopicEvent feasibility_event{
    config.navigation_result_topic,
    rclcpp::Time(2, 0, RCL_ROS_TIME),
    collector::wrap_raw_payload(feasibility_raw)};
  buffer.add_event(feasibility_event);

  EXPECT_TRUE(plugin.handle_event(feasibility_event, buffer).empty());

  const std::string navigation_raw =
    R"({"event":"navigation_result","success":true,"accepted_or_feasible":true,)"
    R"("goal":{"x":4.0,"y":5.0,"z":0.0},"path_length_m":0.0})";
  const collector::TopicEvent navigation_event{
    config.navigation_result_topic,
    rclcpp::Time(3, 0, RCL_ROS_TIME),
    collector::wrap_raw_payload(navigation_raw)};
  buffer.add_event(navigation_event);

  const auto selected_records = plugin.handle_event(navigation_event, buffer);
  ASSERT_EQ(selected_records.size(), 1U);
  EXPECT_NE(
    selected_records.front().find("\"selected_candidate_id\":\"1\""),
    std::string::npos);
  EXPECT_NE(selected_records.front().find("\"has_outcome\":true"), std::string::npos);
  EXPECT_NE(selected_records.front().find("\"selected\":true"), std::string::npos);
  EXPECT_NE(selected_records.front().find("\"path_length\":2.500000"), std::string::npos);
  EXPECT_NE(selected_records.front().find("\"path_length_valid\":true"), std::string::npos);
  EXPECT_NE(
    selected_records.front().find(R"("outcome_context":{"raw_json":{"event":"navigation_result")"),
    std::string::npos);
}

}  // namespace exploration_learning::plugins
