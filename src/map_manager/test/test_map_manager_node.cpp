#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "map_manager_node.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"
#include "robot_interfaces/msg/map_manager_state.hpp"

namespace map_manager
{
namespace
{
using namespace std::chrono_literals;
using ExplorationStateMsg = robot_interfaces::msg::ExplorationState;
using MapManagerStateMsg = robot_interfaces::msg::MapManagerState;
using SaveMap = nav2_msgs::srv::SaveMap;

std::string unique_suffix()
{
  return std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
}

template<typename Predicate>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(20ms);
    std::this_thread::sleep_for(2ms);
  }
  executor.spin_some();
  return predicate();
}

void drain_executor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(10ms);
    std::this_thread::sleep_for(2ms);
  }
}

nav_msgs::msg::OccupancyGrid make_map()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = 2U;
  map.info.height = 2U;
  map.info.resolution = 0.05F;
  map.data = {0, -1, 0, 100};
  return map;
}

ExplorationStateMsg make_completed_state()
{
  ExplorationStateMsg state;
  state.state = ExplorationStateMsg::COMPLETED;
  state.detail = "test_completed";
  return state;
}

rclcpp::NodeOptions make_manager_options(
  const std::string & topic_prefix,
  const std::string & save_directory,
  const std::string & service_name)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("map_topic", topic_prefix + "/map"),
        rclcpp::Parameter("exploration_state_topic", topic_prefix + "/exploration_state"),
        rclcpp::Parameter("map_manager_state_topic", topic_prefix + "/manager_state"),
        rclcpp::Parameter("final_map_topic", topic_prefix + "/final_map"),
        rclcpp::Parameter("completion_check_period_sec", 0.5),
        rclcpp::Parameter("save_directory", save_directory),
        rclcpp::Parameter("map_saver_service_name", service_name),
        rclcpp::Parameter("service_wait_timeout_sec", 0.1)
      });
  return options;
}

class MapManagerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(MapManagerNodeTest, PublishesOneSpecificFailureForRejectedSaveAttempt)
{
  const auto suffix = unique_suffix();
  const auto topic_prefix = "/map_manager_failure_" + suffix;
  const auto temp_root =
    std::filesystem::temp_directory_path() / ("map_manager_failure_" + suffix);
  std::filesystem::create_directories(temp_root);
  const auto invalid_directory = temp_root / "regular_file";
  std::ofstream(invalid_directory.string()) << "not a directory";

  auto client_node = std::make_shared<rclcpp::Node>("map_manager_failure_client_" + suffix);
  auto manager = std::make_shared<MapManagerNode>(
    make_manager_options(topic_prefix, invalid_directory.string(), topic_prefix + "/save_map"));

  auto map_publisher = client_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_prefix + "/map", rclcpp::QoS(1).reliable().transient_local());
  auto exploration_publisher = client_node->create_publisher<ExplorationStateMsg>(
    topic_prefix + "/exploration_state", rclcpp::QoS(10).reliable());

  std::vector<std::string> failure_details;
  auto state_subscription = client_node->create_subscription<MapManagerStateMsg>(
    topic_prefix + "/manager_state",
    rclcpp::QoS(10).reliable().transient_local(),
    [&failure_details](const MapManagerStateMsg::SharedPtr message) {
      if (message->state == MapManagerStateMsg::SAVE_FAILED) {
        failure_details.push_back(message->detail);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(manager);
  ASSERT_TRUE(
    spin_until(
      executor,
      [&]() {
        return map_publisher->get_subscription_count() > 0U &&
        exploration_publisher->get_subscription_count() > 0U &&
        state_subscription->get_publisher_count() > 0U;
      },
      2s));

  map_publisher->publish(make_map());
  exploration_publisher->publish(make_completed_state());
  ASSERT_TRUE(spin_until(executor, [&]() {return !failure_details.empty();}, 3s));
  drain_executor(executor, 100ms);

  ASSERT_EQ(failure_details.size(), 1U);
  EXPECT_EQ(failure_details.front(), "save_directory_unavailable");

  executor.remove_node(manager);
  executor.remove_node(client_node);
  std::filesystem::remove_all(temp_root);
}

TEST_F(MapManagerNodeTest, PublishesFinalMapOnceAcrossSaveRetry)
{
  const auto suffix = unique_suffix();
  const auto topic_prefix = "/map_manager_success_" + suffix;
  const auto save_directory =
    std::filesystem::temp_directory_path() / ("map_manager_success_" + suffix);

  auto service_node = std::make_shared<rclcpp::Node>("map_manager_service_" + suffix);
  std::size_t save_request_count = 0U;
  auto save_service = service_node->create_service<SaveMap>(
    topic_prefix + "/save_map",
    [&save_request_count](
      const std::shared_ptr<SaveMap::Request>,
      std::shared_ptr<SaveMap::Response> response)
    {
      ++save_request_count;
      response->result = save_request_count >= 2U;
    });
  auto manager = std::make_shared<MapManagerNode>(
    make_manager_options(topic_prefix, save_directory.string(), topic_prefix + "/save_map"));

  auto map_publisher = service_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_prefix + "/map", rclcpp::QoS(1).reliable().transient_local());
  auto exploration_publisher = service_node->create_publisher<ExplorationStateMsg>(
    topic_prefix + "/exploration_state", rclcpp::QoS(10).reliable());

  std::size_t final_map_count = 0U;
  bool saved = false;
  auto final_map_subscription = service_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_prefix + "/final_map",
    rclcpp::QoS(1).reliable().transient_local(),
    [&final_map_count](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
      EXPECT_EQ(message->data, make_map().data);
      ++final_map_count;
    });
  auto state_subscription = service_node->create_subscription<MapManagerStateMsg>(
    topic_prefix + "/manager_state",
    rclcpp::QoS(10).reliable().transient_local(),
    [&saved](const MapManagerStateMsg::SharedPtr message) {
      saved = saved || message->state == MapManagerStateMsg::SAVED;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(service_node);
  executor.add_node(manager);
  ASSERT_TRUE(
    spin_until(
      executor,
      [&]() {
        return map_publisher->get_subscription_count() > 0U &&
        exploration_publisher->get_subscription_count() > 0U &&
        final_map_subscription->get_publisher_count() > 0U &&
        state_subscription->get_publisher_count() > 0U &&
        save_service != nullptr;
      },
      2s));

  map_publisher->publish(make_map());
  exploration_publisher->publish(make_completed_state());
  ASSERT_TRUE(spin_until(executor, [&]() {return saved;}, 3s));
  drain_executor(executor, 100ms);

  EXPECT_EQ(save_request_count, 2U);
  EXPECT_EQ(final_map_count, 1U);

  executor.remove_node(manager);
  executor.remove_node(service_node);
  std::filesystem::remove_all(save_directory);
}

}  // namespace
}  // namespace map_manager
