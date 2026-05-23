#include "exploration_learning/collector/event_buffer.hpp"

#include "gtest/gtest.h"

namespace exploration_learning::collector
{

TEST(EventBufferTest, KeepsLatestEventPerTopic)
{
  EventBuffer buffer(10.0);
  buffer.add_event(TopicEvent{"topic_a", rclcpp::Time(1, 0, RCL_ROS_TIME), "{\"a\":1}"});
  buffer.add_event(TopicEvent{"topic_b", rclcpp::Time(2, 0, RCL_ROS_TIME), "{\"b\":1}"});
  buffer.add_event(TopicEvent{"topic_a", rclcpp::Time(3, 0, RCL_ROS_TIME), "{\"a\":2}"});

  const auto latest_a = buffer.latest_event("topic_a");
  ASSERT_TRUE(latest_a.has_value());
  EXPECT_EQ(latest_a->payload_json, "{\"a\":2}");
  EXPECT_EQ(buffer.events_for_topic("topic_a").size(), 2U);
  EXPECT_EQ(buffer.size(), 3U);
}

TEST(EventBufferTest, PrunesExpiredEvents)
{
  EventBuffer buffer(2.0);
  buffer.add_event(TopicEvent{"topic_a", rclcpp::Time(1, 0, RCL_ROS_TIME), "{\"a\":1}"});
  buffer.add_event(TopicEvent{"topic_a", rclcpp::Time(2, 0, RCL_ROS_TIME), "{\"a\":2}"});
  buffer.add_event(TopicEvent{"topic_b", rclcpp::Time(4, 500000000, RCL_ROS_TIME), "{\"b\":1}"});

  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_FALSE(buffer.latest_event("topic_a").has_value());
  EXPECT_TRUE(buffer.latest_event("topic_b").has_value());
}

}  // namespace exploration_learning::collector
