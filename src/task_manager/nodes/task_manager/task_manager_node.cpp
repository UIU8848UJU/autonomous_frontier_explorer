#include "task_manager/task_manager_node.hpp"

#include <functional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "adapters/ros_message_converter.hpp"

namespace task_manager
{
namespace
{
int declare_positive_parameter(rclcpp::Node & node, const char * name)
{
  const auto value = node.declare_parameter<int>(name);
  if (value <= 0) {
    throw std::invalid_argument(std::string("Parameter '") + name + "' must be positive.");
  }
  return value;
}
}  // 匿名命名空间

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_manager_node", options)
{
  declare_parameters();
  create_interfaces();
  RCLCPP_INFO(this->get_logger(), "TaskManagerNode ready.");
  publish_state();
}

void TaskManagerNode::declare_parameters()
{
  // 这些参数必须从 YAML 配置读取。集中配置可以避免节点脱离标准
  // launch 启动时悄悄使用不同的 QoS 或超时。
  heartbeat_period_ = std::chrono::milliseconds(
    declare_positive_parameter(*this, "heartbeat_period_ms"));
  exploration_timeout_ = std::chrono::milliseconds(
    declare_positive_parameter(*this, "exploration_state_timeout_ms"));
  service_call_timeout_ = std::chrono::milliseconds(
    declare_positive_parameter(*this, "service_call_timeout_ms"));
  state_publisher_depth_ = static_cast<std::size_t>(
    declare_positive_parameter(*this, "state_publisher_depth"));
  exploration_subscription_depth_ = static_cast<std::size_t>(
    declare_positive_parameter(*this, "exploration_subscription_depth"));
  map_lifecycle_subscription_depth_ = static_cast<std::size_t>(
    declare_positive_parameter(*this, "map_lifecycle_subscription_depth"));

  interface_config_.exploration_state_topic = this->declare_parameter<std::string>(
    "exploration_state_topic", interface_config_.exploration_state_topic);
  interface_config_.map_lifecycle_state_topic = this->declare_parameter<std::string>(
    "map_lifecycle_state_topic", interface_config_.map_lifecycle_state_topic);
  interface_config_.task_manager_state_topic = this->declare_parameter<std::string>(
    "task_manager_state_topic", interface_config_.task_manager_state_topic);
  interface_config_.start_mapping_service_name = this->declare_parameter<std::string>(
    "start_mapping_service_name", interface_config_.start_mapping_service_name);
  interface_config_.stop_all_service_name = this->declare_parameter<std::string>(
    "stop_all_service_name", interface_config_.stop_all_service_name);
  interface_config_.start_exploration_service_name = this->declare_parameter<std::string>(
    "start_exploration_service_name", interface_config_.start_exploration_service_name);
  interface_config_.stop_exploration_service_name = this->declare_parameter<std::string>(
    "stop_exploration_service_name", interface_config_.stop_exploration_service_name);

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters loaded: heartbeat=%ld ms, exploration_timeout=%ld ms, "
    "service_timeout=%ld ms, qos_depths=[%zu,%zu,%zu], topics=[%s,%s,%s], services=[%s,%s]",
    heartbeat_period_.count(),
    exploration_timeout_.count(),
    service_call_timeout_.count(),
    state_publisher_depth_,
    exploration_subscription_depth_,
    map_lifecycle_subscription_depth_,
    interface_config_.exploration_state_topic.c_str(),
    interface_config_.map_lifecycle_state_topic.c_str(),
    interface_config_.task_manager_state_topic.c_str(),
    interface_config_.start_mapping_service_name.c_str(),
    interface_config_.stop_all_service_name.c_str());
}

void TaskManagerNode::create_interfaces()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto state_pub_qos =
    rclcpp::QoS(rclcpp::KeepLast(state_publisher_depth_)).reliable();
  state_pub_ = this->create_publisher<TaskManagerStateMsg>(
    interface_config_.task_manager_state_topic, state_pub_qos);

  const auto explor_sub_qos =
    rclcpp::QoS(rclcpp::KeepLast(exploration_subscription_depth_)).reliable();
  exploration_state_sub_ = this->create_subscription<ExplorationStateMsg>(
    interface_config_.exploration_state_topic, explor_sub_qos,
    std::bind(&TaskManagerNode::handle_exploration_state, this, _1));

  const auto map_lifecycle_sub_qos = rclcpp::QoS(
    rclcpp::KeepLast(map_lifecycle_subscription_depth_)).reliable().transient_local();
  map_lifecycle_state_sub_ = this->create_subscription<MapLifecycleStateMsg>(
    interface_config_.map_lifecycle_state_topic, map_lifecycle_sub_qos,
    std::bind(&TaskManagerNode::handle_map_lifecycle_state, this, _1));

  start_mapping_srv_ = this->create_service<Trigger>(
    interface_config_.start_mapping_service_name,
    std::bind(&TaskManagerNode::handle_start_mapping, this, _1, _2));

  stop_all_srv_ = this->create_service<Trigger>(
    interface_config_.stop_all_service_name,
    std::bind(&TaskManagerNode::handle_stop_all, this, _1, _2));

  start_exploration_client_ = this->create_client<Trigger>(
    interface_config_.start_exploration_service_name);

  stop_exploration_client_ = this->create_client<Trigger>(
    interface_config_.stop_exploration_service_name);

  heartbeat_timer_ = this->create_wall_timer(
    heartbeat_period_,
    std::bind(&TaskManagerNode::handle_tick, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Interfaces ready: state_pub=%s exploration_sub=%s map_lifecycle_sub=%s",
    interface_config_.task_manager_state_topic.c_str(),
    interface_config_.exploration_state_topic.c_str(),
    interface_config_.map_lifecycle_state_topic.c_str());
}

void TaskManagerNode::publish_state()
{
  if (!state_pub_) {
    return;
  }

  state_pub_->publish(compose_state_message());
}

TaskManagerStateMsg TaskManagerNode::compose_state_message() const
{
  TaskManagerStateMsg msg;
  msg.stamp = this->now();

  const auto & context = task_flow_.context();
  msg.state = static_cast<uint8_t>(context.state);
  msg.state_text = to_string(context.state);
  msg.map_ready = context.map_ready;
  msg.exploration_running = context.exploration_running;
  msg.navigation_running = false;
  msg.last_error = context.last_error;
  msg.last_exploration_state = context.last_exploration_state;

  return msg;
}

void TaskManagerNode::log_state_transition(
  TaskManagerState previous_state, const char * reason)
{
  const auto current_state = task_flow_.state();
  if (previous_state == current_state) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[FSM] %s -> %s, reason=%s",
    to_string(previous_state).c_str(),
    to_string(current_state).c_str(),
    reason);
}

void TaskManagerNode::handle_tick()
{
  check_exploration_timeout();
  publish_state();
}

void TaskManagerNode::handle_exploration_state(const ExplorationStateMsg::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null exploration state message.");
    return;
  }

  const auto event = adapters::to_core_exploration_event(*msg);
  const auto previous_state = task_flow_.state();
  task_flow_.update_exploration_state(event);
  last_exploration_msg_time_ = std::chrono::steady_clock::now();
  exploration_timeout_reported_ = false;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Exploration state update: %s",
    task_flow_.context().last_exploration_state.c_str());
  const char * reason = "exploration_state_update";
  switch (event.state) {
    case ExplorationState::RUNNING:
      reason = task_flow_.state() == TaskManagerState::FAILED ?
        "unexpected_mapping_running" : "mapping_running";
      break;
    case ExplorationState::COMPLETED:
      reason = task_flow_.state() == TaskManagerState::FAILED ?
        "unexpected_mapping_completed" : "mapping_completed";
      break;
    case ExplorationState::STOPPED: reason = "exploration_stopped"; break;
    case ExplorationState::IDLE: reason = "exploration_idle"; break;
    case ExplorationState::STUCK: reason = "exploration_stuck"; break;
    default: reason = "unknown_exploration_state"; break;
  }
  log_state_transition(previous_state, reason);
  publish_state();
}

void TaskManagerNode::handle_map_lifecycle_state(const MapLifecycleStateMsg::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(this->get_logger(), "Received null map lifecycle state message.");
    return;
  }

  if (msg->state == msg->SAVED) {
    const auto previous_state = task_flow_.state();
    const bool mapping_done = task_flow_.mark_map_saved();
    if (!mapping_done) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring SAVED map lifecycle event while task state is %s.",
        to_string(task_flow_.context().state).c_str());
      return;
    }
    log_state_transition(previous_state, "map_saved");
    RCLCPP_INFO(
      this->get_logger(),
      "Map saved and mapping marked done: url=%s",
      msg->saved_map_url.c_str());
    request_stop_exploration();
    publish_state();
    return;
  }

  if (msg->state == msg->FAILED) {
    const auto previous_state = task_flow_.state();
    task_flow_.fail(msg->detail.empty() ? "Map save failed." : msg->detail);
    log_state_transition(previous_state, "map_save_failed");
    RCLCPP_WARN(
      this->get_logger(),
      "Map lifecycle reported save failure: %s",
      msg->detail.c_str());
    publish_state();
  }
}

void TaskManagerNode::handle_start_mapping(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  const auto previous_state = task_flow_.state();
  const bool started = task_flow_.start_mapping_flow();
  response->success = started;
  response->message = started ?
    "Mapping flow started." : "Mapping already running.";

  if (started) {
    last_exploration_msg_time_ = std::chrono::steady_clock::now();
    exploration_timeout_reported_ = false;
    request_start_exploration();
  }
  log_state_transition(previous_state, "start_mapping_requested");
  RCLCPP_INFO(
    this->get_logger(),
    "Start mapping service invoked, success=%s",
    started ? "true" : "false");
  publish_state();
}

void TaskManagerNode::handle_stop_all(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  const auto previous_state = task_flow_.state();
  const bool stopped = task_flow_.stop_all();
  response->success = stopped;
  response->message = stopped ? "All flows stopped." : "No active flows.";

  log_state_transition(previous_state, "stop_all_requested");
  RCLCPP_INFO(
    this->get_logger(),
    "Stop-all service invoked, success=%s",
    stopped ? "true" : "false");
  if (stopped) {
    request_stop_exploration();
  }
  publish_state();
}

void TaskManagerNode::check_exploration_timeout()
{
  if (!task_flow_.context().exploration_running) {
    return;
  }

  if (!last_exploration_msg_time_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - *last_exploration_msg_time_ <= exploration_timeout_) {
    return;
  }

  if (exploration_timeout_reported_) {
    return;
  }

  exploration_timeout_reported_ = true;
  const auto previous_state = task_flow_.state();
  task_flow_.fail("Exploration heartbeat timeout");
  log_state_transition(previous_state, "heartbeat_timeout");
  request_stop_exploration();
  RCLCPP_WARN(
    this->get_logger(),
    "Exploration heartbeat timeout detected (>=%ld ms).",
    exploration_timeout_.count());
}

void TaskManagerNode::request_start_exploration()
{
  if (call_trigger_service(start_exploration_client_, "start_exploration")) {
    RCLCPP_INFO(this->get_logger(), "Exploration start requested.");
  }
}

void TaskManagerNode::request_stop_exploration()
{
  call_trigger_service(stop_exploration_client_, "stop_exploration");
}

bool TaskManagerNode::call_trigger_service(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const std::string & label)
{
  if (!client) {
    return false;
  }

  if (!client->service_is_ready()) {
    if (!client->wait_for_service(service_call_timeout_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Service %s not available.",
        label.c_str());
      return false;
    }
  }

  auto request = std::make_shared<Trigger::Request>();
  auto future = client->async_send_request(request);
  const auto status = future.wait_for(service_call_timeout_);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(
      this->get_logger(),
      "Service %s did not respond in time.",
      label.c_str());
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      this->get_logger(),
      "Service %s failed: %s",
      label.c_str(),
      response->message.c_str());
  }
  return response->success;
}

}  // 命名空间 task_manager
