#include <cstdint>

#include "gtest/gtest.h"
#include "robot_interfaces/msg/exploration_state.hpp"

#include "adapters/ros_message_converter.hpp"

namespace task_manager
{

TEST(RosMessageConverterTest, MapsKnownExplorationStates)
{
  robot_interfaces::msg::ExplorationState message;
  message.detail = "running";

  message.state = robot_interfaces::msg::ExplorationState::IDLE;
  EXPECT_EQ(adapters::to_core_exploration_event(message).state, ExplorationState::IDLE);

  message.state = robot_interfaces::msg::ExplorationState::RUNNING;
  EXPECT_EQ(adapters::to_core_exploration_event(message).state, ExplorationState::RUNNING);

  message.state = robot_interfaces::msg::ExplorationState::STOPPED;
  EXPECT_EQ(adapters::to_core_exploration_event(message).state, ExplorationState::STOPPED);

  message.state = robot_interfaces::msg::ExplorationState::COMPLETED;
  EXPECT_EQ(
    adapters::to_core_exploration_event(message).state,
    ExplorationState::COMPLETED);

  message.state = robot_interfaces::msg::ExplorationState::STUCK;
  const auto event = adapters::to_core_exploration_event(message);
  EXPECT_EQ(event.state, ExplorationState::STUCK);
  EXPECT_EQ(event.detail, "running");
}

TEST(RosMessageConverterTest, MapsUnknownWireValuesToUnknown)
{
  robot_interfaces::msg::ExplorationState message;
  message.state = static_cast<std::uint8_t>(99U);

  EXPECT_EQ(
    adapters::to_core_exploration_event(message).state,
    ExplorationState::UNKNOWN);
}

}  // namespace task_manager
