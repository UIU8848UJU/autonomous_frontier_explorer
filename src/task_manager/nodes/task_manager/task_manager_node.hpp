#pragma once

#include <chrono>
#include <cstddef>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_interfaces/msg/exploration_state.hpp"
#include "robot_interfaces/msg/map_lifecycle_state.hpp"
#include "robot_interfaces/msg/task_manager_state.hpp"

#include "task_flow/task_flow.hpp"

namespace task_manager
{

using ExplorationStateMsg = robot_interfaces::msg::ExplorationState;
using MapLifecycleStateMsg = robot_interfaces::msg::MapLifecycleState;
using TaskManagerStateMsg = robot_interfaces::msg::TaskManagerState;

class TaskManagerNode : public rclcpp::Node
{
public:
  explicit TaskManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;

  struct InterfaceConfig
  {
    std::string exploration_state_topic{"/exploration_state"};
    std::string map_lifecycle_state_topic{"/map_lifecycle_state"};
    std::string task_manager_state_topic{"/task_manager_state"};
    std::string start_mapping_service_name{"/start_mapping"};
    std::string stop_all_service_name{"/stop_all"};
    std::string start_exploration_service_name{"/start_exploration"};
    std::string stop_exploration_service_name{"/stop_exploration"};
  };

  void declare_parameters();
  void create_interfaces();

  void publish_state();
  TaskManagerStateMsg compose_state_message() const;
  void log_state_transition(TaskManagerState previous_state, const char * reason);
  void handle_tick();
  void check_exploration_timeout();
  void handle_exploration_state(const ExplorationStateMsg::SharedPtr msg);
  void handle_map_lifecycle_state(const MapLifecycleStateMsg::SharedPtr msg);
  void request_start_exploration();
  void request_stop_exploration();
  bool call_trigger_service(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & label);

  void handle_start_mapping(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response);

  void handle_stop_all(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response);

private:
  TaskFlow task_flow_;
  InterfaceConfig interface_config_;

  rclcpp::Publisher<TaskManagerStateMsg>::SharedPtr state_pub_;
  rclcpp::Subscription<ExplorationStateMsg>::SharedPtr exploration_state_sub_;
  rclcpp::Subscription<MapLifecycleStateMsg>::SharedPtr map_lifecycle_state_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Service<Trigger>::SharedPtr start_mapping_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_all_srv_;
  rclcpp::Client<Trigger>::SharedPtr start_exploration_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_exploration_client_;

  // 运行时周期和 QoS 参数由 task_manager.yaml 提供。
  std::chrono::milliseconds heartbeat_period_;
  std::chrono::milliseconds exploration_timeout_;
  std::chrono::milliseconds service_call_timeout_;
  std::size_t state_publisher_depth_{0};
  std::size_t exploration_subscription_depth_{0};
  std::size_t map_lifecycle_subscription_depth_{0};
  std::optional<std::chrono::steady_clock::time_point> last_exploration_msg_time_;
  bool exploration_timeout_reported_{false};
};

}  // 命名空间 task_manager
