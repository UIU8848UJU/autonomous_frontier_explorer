#include "task_manager/task_manager_node.hpp"

#include <functional>

#define __CLASS_NAME__ "TaskManagerNode"
#include "friendly_logging/logging.h"

using namespace std::chrono_literals;

namespace task_manager
{
namespace
{
constexpr int kDefaultHeartbeatPeriodMs = 1000;
constexpr size_t kStatePublisherDepth = 10;
constexpr size_t kExplorationSubDepth = 10;
}  // namespace

TaskManagerNode::TaskManagerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("task_manager_node", options)
{
    declare_parameters();
    create_interfaces();
    RCLCPP_INFO_WITH_CONTEXT(this->get_logger(), "TaskManagerNode ready.");
    publish_state();
}

void TaskManagerNode::declare_parameters()
{
    const auto period_ms =
        this->declare_parameter<int>("heartbeat_period_ms", kDefaultHeartbeatPeriodMs);
    heartbeat_period_ = std::chrono::milliseconds(period_ms);

    interface_config_.exploration_state_topic = this->declare_parameter<std::string>(
        "exploration_state_topic", interface_config_.exploration_state_topic);
    interface_config_.task_manager_state_topic = this->declare_parameter<std::string>(
        "task_manager_state_topic", interface_config_.task_manager_state_topic);
    interface_config_.start_mapping_service_name = this->declare_parameter<std::string>(
        "start_mapping_service_name", interface_config_.start_mapping_service_name);
    interface_config_.start_navigation_service_name = this->declare_parameter<std::string>(
        "start_navigation_service_name", interface_config_.start_navigation_service_name);
    interface_config_.stop_all_service_name = this->declare_parameter<std::string>(
        "stop_all_service_name", interface_config_.stop_all_service_name);
    interface_config_.start_exploration_service_name = this->declare_parameter<std::string>(
        "start_exploration_service_name", interface_config_.start_exploration_service_name);
    interface_config_.stop_exploration_service_name = this->declare_parameter<std::string>(
        "stop_exploration_service_name", interface_config_.stop_exploration_service_name);
    const auto timeout_ms = this->declare_parameter<int>(
        "exploration_state_timeout_ms",
        static_cast<int>(exploration_timeout_.count()));
    exploration_timeout_ = std::chrono::milliseconds(timeout_ms);

    RCLCPP_INFO_WITH_CONTEXT(
        this->get_logger(),
        "Parameters loaded: heartbeat=%ld ms, topics=[%s,%s], services=[%s,%s,%s]",
        heartbeat_period_.count(),
        interface_config_.exploration_state_topic.c_str(),
        interface_config_.task_manager_state_topic.c_str(),
        interface_config_.start_mapping_service_name.c_str(),
        interface_config_.start_navigation_service_name.c_str(),
        interface_config_.stop_all_service_name.c_str());
}

void TaskManagerNode::create_interfaces()
{
    using std::placeholders::_1;
    using std::placeholders::_2;

    const auto state_pub_qos = rclcpp::QoS(rclcpp::KeepLast(kStatePublisherDepth)).reliable();
    state_pub_ = this->create_publisher<TaskManagerStateMsg>(
        interface_config_.task_manager_state_topic, state_pub_qos);

    const auto explor_sub_qos = rclcpp::QoS(rclcpp::KeepLast(kExplorationSubDepth)).reliable();
    exploration_state_sub_ = this->create_subscription<ExplorationStateMsg>(
        interface_config_.exploration_state_topic, explor_sub_qos,
        std::bind(&TaskManagerNode::handle_exploration_state, this, _1));

    start_mapping_srv_ = this->create_service<Trigger>(
        interface_config_.start_mapping_service_name,
        std::bind(&TaskManagerNode::handle_start_mapping, this, _1, _2));

    start_navigation_srv_ = this->create_service<Trigger>(
        interface_config_.start_navigation_service_name,
        std::bind(&TaskManagerNode::handle_start_navigation, this, _1, _2));

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

    RCLCPP_INFO_WITH_CONTEXT(
        this->get_logger(),
        "Interfaces ready: state_pub=%s exploration_sub=%s",
        interface_config_.task_manager_state_topic.c_str(),
        interface_config_.exploration_state_topic.c_str());
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
    msg.navigation_running = context.navigation_running;
    msg.last_error = context.last_error;
    msg.last_exploration_state = context.last_exploration_state;

    return msg;
}

void TaskManagerNode::handle_tick()
{
    check_exploration_timeout();
    publish_state();
}

void TaskManagerNode::handle_exploration_state(const ExplorationStateMsg::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Received null exploration state message.");
        return;
    }

    task_flow_.update_exploration_state(*msg);
    last_exploration_msg_time_ = std::chrono::steady_clock::now();
    exploration_timeout_reported_ = false;
    RCLCPP_DEBUG_WITH_CONTEXT(
        this->get_logger(),
        "Exploration state update: %s",
        task_flow_.context().last_exploration_state.c_str());
    if (msg->state == msg->COMPLETED &&
        task_flow_.context().state == TaskManagerState::MAPPING_DONE)
    {
        schedule_restart_exploration();
    }
    publish_state();
}

void TaskManagerNode::handle_start_mapping(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
    const bool started = task_flow_.start_mapping_flow();
    response->success = started;
    response->message = started ? 
        "Mapping flow started." : "Mapping already running.";

    if (started) {
        last_exploration_msg_time_ = std::chrono::steady_clock::now();
        exploration_timeout_reported_ = false;
        request_start_exploration();
    }
    RCLCPP_INFO_WITH_CONTEXT(
        this->get_logger(),
        "Start mapping service invoked, success=%s",
        started ? "true" : "false");
    publish_state();
}

void TaskManagerNode::handle_start_navigation(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
    const bool started = task_flow_.start_navigation_flow();
    response->success = started;
    response->message = started ?
         "Navigation flow started." : "Navigation cannot start.";

    if (started) {
        RCLCPP_INFO_WITH_CONTEXT(this->get_logger(), "Navigation flow started.");
    } else {
        RCLCPP_WARN_WITH_CONTEXT(this->get_logger(), "Navigation flow rejected.");
    }
    publish_state();
}

void TaskManagerNode::handle_stop_all(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
    const bool stopped = task_flow_.stop_all();
    response->success = stopped;
    response->message = stopped ? "All flows stopped." : "No active flows.";

    RCLCPP_INFO_WITH_CONTEXT(
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
    task_flow_.stop_all();
    task_flow_.set_error("Exploration heartbeat timeout");
    task_flow_.set_state(TaskManagerState::FAILED);
    request_stop_exploration();
    RCLCPP_WARN_WITH_CONTEXT(
        this->get_logger(),
        "Exploration heartbeat timeout detected (>=%ld ms).",
        exploration_timeout_.count());
}

void TaskManagerNode::request_start_exploration()
{
    if (call_trigger_service(start_exploration_client_, "start_exploration")) {
        RCLCPP_INFO_WITH_CONTEXT(this->get_logger(), "Exploration start requested.");
    }
}

void TaskManagerNode::request_stop_exploration()
{
    call_trigger_service(stop_exploration_client_, "stop_exploration");
}

void TaskManagerNode::schedule_restart_exploration()
{
    if (pending_restart_) {
        return;
    }
    pending_restart_ = true;
    restart_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TaskManagerNode::restart_timer_callback, this));
}

void TaskManagerNode::restart_timer_callback()
{
    if (restart_timer_) {
        restart_timer_->cancel();
    }
    pending_restart_ = false;
    request_start_exploration();
}

bool TaskManagerNode::call_trigger_service(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & label)
{
    if (!client) {
        return false;
    }

    if (!client->service_is_ready()) {
        if (!client->wait_for_service(1s)) {
            RCLCPP_WARN_WITH_CONTEXT(
                this->get_logger(),
                "Service %s not available.",
                label.c_str());
            return false;
        }
    }

    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);
    const auto status = future.wait_for(1s);
    if (status != std::future_status::ready) {
        RCLCPP_WARN_WITH_CONTEXT(
            this->get_logger(),
            "Service %s did not respond in time.",
            label.c_str());
        return false;
    }

    const auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN_WITH_CONTEXT(
            this->get_logger(),
            "Service %s failed: %s",
            label.c_str(),
            response->message.c_str());
    }
    return response->success;
}

}  // namespace task_manager
