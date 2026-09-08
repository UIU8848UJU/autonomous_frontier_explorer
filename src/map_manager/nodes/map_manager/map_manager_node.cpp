#include "map_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>

#include "adapters/ros_message_converter.hpp"

namespace map_manager
{
namespace
{
constexpr std::size_t kMapSubDepth = 1U;
constexpr std::size_t kStateSubDepth = 10U;
constexpr std::size_t kMapManagerStatePubDepth = 10U;
constexpr std::size_t kFinalMapPubDepth = 1U;
}  // namespace

MapManagerNode::MapManagerNode(const rclcpp::NodeOptions & options)
: Node("map_manager_node", options),
  logger_(this->get_logger().get_child("map_manager"))
{
  declare_params();
  load_params();
  apply_params();
  create_interfaces();

  RCLCPP_INFO(logger_, "MapManagerNode started.");
  publish_state(MapManagerStateMsg::IDLE, "started");
}

void MapManagerNode::declare_params()
{
  this->declare_parameter<std::string>("map_topic", config_.map_topic);
  this->declare_parameter<std::string>(
    "exploration_state_topic",
    config_.exploration_state_topic);
  this->declare_parameter<std::string>(
    "map_manager_state_topic",
    config_.map_manager_state_topic);
  this->declare_parameter<std::string>(
    "final_map_topic",
    config_.final_map_topic);
  this->declare_parameter<bool>("enable_auto_save", config_.enable_auto_save);
  this->declare_parameter<double>(
    "completion_check_period_sec",
    config_.completion_check_period_sec);

  this->declare_parameter<std::string>(
    "save_directory",
    nav2_map_saver_config_.save_directory);
  this->declare_parameter<std::string>(
    "saved_map_prefix",
    nav2_map_saver_config_.saved_map_prefix);
  this->declare_parameter<std::string>(
    "map_saver_service_name",
    nav2_map_saver_config_.map_saver_service_name);
  this->declare_parameter<std::string>(
    "image_format",
    nav2_map_saver_config_.image_format);
  this->declare_parameter<std::string>(
    "map_mode",
    nav2_map_saver_config_.map_mode);
  this->declare_parameter<double>("free_thresh", nav2_map_saver_config_.free_thresh);
  this->declare_parameter<double>("occupied_thresh", nav2_map_saver_config_.occupied_thresh);
  this->declare_parameter<double>(
    "service_wait_timeout_sec",
    nav2_map_saver_config_.service_wait_timeout_sec);
}

void MapManagerNode::load_params()
{
  config_.map_topic = this->get_parameter("map_topic").as_string();
  config_.exploration_state_topic =
    this->get_parameter("exploration_state_topic").as_string();
  config_.map_manager_state_topic =
    this->get_parameter("map_manager_state_topic").as_string();
  config_.final_map_topic =
    this->get_parameter("final_map_topic").as_string();
  config_.enable_auto_save = this->get_parameter("enable_auto_save").as_bool();
  config_.completion_check_period_sec =
    this->get_parameter("completion_check_period_sec").as_double();

  nav2_map_saver_config_.save_directory = this->get_parameter("save_directory").as_string();
  nav2_map_saver_config_.saved_map_prefix = this->get_parameter("saved_map_prefix").as_string();
  nav2_map_saver_config_.map_saver_service_name =
    this->get_parameter("map_saver_service_name").as_string();
  nav2_map_saver_config_.map_topic = config_.map_topic;
  nav2_map_saver_config_.image_format = this->get_parameter("image_format").as_string();
  nav2_map_saver_config_.map_mode = this->get_parameter("map_mode").as_string();
  nav2_map_saver_config_.free_thresh = this->get_parameter("free_thresh").as_double();
  nav2_map_saver_config_.occupied_thresh = this->get_parameter("occupied_thresh").as_double();
  nav2_map_saver_config_.service_wait_timeout_sec =
    this->get_parameter("service_wait_timeout_sec").as_double();
}

void MapManagerNode::apply_params()
{
  config_.completion_check_period_sec =
    std::max(0.5, config_.completion_check_period_sec);

  nav2_map_saver_config_.free_thresh =
    std::clamp(nav2_map_saver_config_.free_thresh, 0.0, 1.0);
  nav2_map_saver_config_.occupied_thresh =
    std::clamp(nav2_map_saver_config_.occupied_thresh, 0.0, 1.0);
  nav2_map_saver_config_.service_wait_timeout_sec =
    std::max(0.1, nav2_map_saver_config_.service_wait_timeout_sec);

  RCLCPP_INFO(
    logger_,
    "Map manager params: map_topic=%s, state_topic=%s, auto_save=%s, "
    "manager_state_topic=%s, final_map_topic=%s",
    config_.map_topic.c_str(),
    config_.exploration_state_topic.c_str(),
    config_.enable_auto_save ? "true" : "false",
    config_.map_manager_state_topic.c_str(),
    config_.final_map_topic.c_str());
  RCLCPP_INFO(
    logger_,
    "Map save params: directory=%s, prefix=%s, service=%s",
    nav2_map_saver_config_.save_directory.c_str(),
    nav2_map_saver_config_.saved_map_prefix.c_str(),
    nav2_map_saver_config_.map_saver_service_name.c_str());
}

void MapManagerNode::create_interfaces()
{
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(kMapSubDepth)).reliable().transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    config_.map_topic,
    map_qos,
    std::bind(&MapManagerNode::map_callback, this, std::placeholders::_1));

  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(kStateSubDepth)).reliable();
  exploration_state_sub_ = this->create_subscription<ExplorationStateMsg>(
    config_.exploration_state_topic,
    state_qos,
    std::bind(&MapManagerNode::exploration_state_callback, this, std::placeholders::_1));

  state_pub_ = this->create_publisher<MapManagerStateMsg>(
    config_.map_manager_state_topic,
    rclcpp::QoS(rclcpp::KeepLast(kMapManagerStatePubDepth)).reliable().transient_local());

  final_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    config_.final_map_topic,
    rclcpp::QoS(rclcpp::KeepLast(kFinalMapPubDepth)).reliable().transient_local());

  nav2_map_saver_ = std::make_unique<Nav2MapSaver>(*this, logger_, nav2_map_saver_config_);

  completion_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(config_.completion_check_period_sec),
    std::bind(&MapManagerNode::completion_timer_callback, this));
}

void MapManagerNode::publish_state(std::uint8_t state, const std::string & detail)
{
  if (!state_pub_) {
    return;
  }

  const auto & map_stats = core_.map_statistics();
  MapManagerStateMsg msg;
  msg.stamp = this->now();
  msg.state = state;
  msg.state_text = map_manager_state_to_string(state);
  msg.map_ready = core_.save_succeeded();
  msg.save_requested = core_.save_requested();
  msg.save_succeeded = core_.save_succeeded();
  msg.map_url = core_.saved_map_url();
  msg.detail = detail;
  msg.unknown_ratio = map_stats.unknown_ratio;
  msg.width = map_stats.width;
  msg.height = map_stats.height;
  msg.resolution = map_stats.resolution;
  state_pub_->publish(msg);
}

void MapManagerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(logger_, "Received null map message.");
    return;
  }

  latest_map_ = msg;
  core_.update_map(adapters::to_core_map(*msg));

  if (!core_.save_succeeded()) {
    publish_state(MapManagerStateMsg::MAP_RECEIVED, "map_updated");
  }

  RCLCPP_INFO_THROTTLE(
    logger_,
    *this->get_clock(),
    5000,
    "Received map: width=%u, height=%u, resolution=%.3f, unknown_ratio=%.6f",
    core_.map_statistics().width,
    core_.map_statistics().height,
    core_.map_statistics().resolution,
    core_.map_statistics().unknown_ratio);
}

void MapManagerNode::exploration_state_callback(const ExplorationStateMsg::SharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(logger_, "Received null exploration state message.");
    return;
  }

  core_.update_exploration_state(adapters::to_core_exploration_state(*msg));
  RCLCPP_DEBUG(
    logger_,
    "Exploration state updated: state=%s, detail=%s",
    exploration_state_to_string(core_.last_exploration_state()).c_str(),
    core_.last_exploration_detail().c_str());
}

void MapManagerNode::completion_timer_callback()
{
  if (!core_.should_save(config_.enable_auto_save)) {
    return;
  }

  if (!core_.completion_detected()) {
    core_.mark_completion_detected();
    RCLCPP_INFO(
      logger_,
      "Exploration completed, saving map: unknown_ratio=%.6f",
      core_.map_statistics().unknown_ratio);
    publish_state(MapManagerStateMsg::COMPLETION_DETECTED, "completion_detected");
    publish_final_map("completion_detected");
  }
  trigger_save();
}

void MapManagerNode::trigger_save()
{
  if (!nav2_map_saver_) {
    RCLCPP_ERROR(logger_, "Cannot save map because Nav2MapSaver is not initialized.");
    return;
  }

  core_.mark_save_requested();
  RCLCPP_WARN(logger_, "Triggering map save.");
  publish_state(MapManagerStateMsg::SAVING, "saving");
  nav2_map_saver_->save_current_map(
    [this](const Nav2MapSaveResult & result) {
      core_.record_save_result(result.success, result.map_url);
      if (!result.success) {
        RCLCPP_ERROR(
          logger_,
          "Auto save failed: reason=%s",
          result.message.c_str());
        publish_state(MapManagerStateMsg::SAVE_FAILED, result.message);
        return;
      }

      RCLCPP_INFO(
        logger_,
        "Auto save finished: map_url=%s",
        result.map_url.c_str());
      publish_state(MapManagerStateMsg::SAVED, "saved");
    });
}

void MapManagerNode::publish_final_map(const std::string & detail)
{
  if (!final_map_pub_) {
    RCLCPP_WARN(logger_, "Cannot publish final map because publisher is not initialized.");
    return;
  }

  if (!latest_map_) {
    RCLCPP_WARN(logger_, "Cannot publish final map because no map has been received.");
    return;
  }

  final_map_pub_->publish(*latest_map_);
  RCLCPP_INFO(
    logger_,
    "Published final map: topic=%s, width=%u, height=%u, reason=%s",
    config_.final_map_topic.c_str(),
    latest_map_->info.width,
    latest_map_->info.height,
    detail.c_str());
}

std::string MapManagerNode::exploration_state_to_string(ExplorationPhase state) const
{
  switch (state) {
    case ExplorationPhase::IDLE:
      return "IDLE";
    case ExplorationPhase::RUNNING:
      return "RUNNING";
    case ExplorationPhase::STOPPED:
      return "STOPPED";
    case ExplorationPhase::COMPLETED:
      return "COMPLETED";
    case ExplorationPhase::STUCK:
      return "STUCK";
    default:
      return "UNKNOWN";
  }
}

std::string MapManagerNode::map_manager_state_to_string(std::uint8_t state) const
{
  switch (state) {
    case MapManagerStateMsg::IDLE:
      return "IDLE";
    case MapManagerStateMsg::MAP_RECEIVED:
      return "MAP_RECEIVED";
    case MapManagerStateMsg::COMPLETION_DETECTED:
      return "COMPLETION_DETECTED";
    case MapManagerStateMsg::SAVING:
      return "SAVING";
    case MapManagerStateMsg::SAVED:
      return "SAVED";
    case MapManagerStateMsg::SAVE_FAILED:
      return "SAVE_FAILED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace map_manager
