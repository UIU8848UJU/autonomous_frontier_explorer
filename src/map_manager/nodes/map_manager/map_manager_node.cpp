#include "map_manager_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
    this->declare_parameter<int>(
        "completion_no_frontier_rounds",
        config_.completion_no_frontier_rounds);
    this->declare_parameter<double>(
        "completion_unknown_delta_threshold",
        config_.completion_unknown_delta_threshold);
    this->declare_parameter<double>(
        "completion_check_window_sec",
        config_.completion_check_window_sec);
    this->declare_parameter<double>(
        "completion_check_period_sec",
        config_.completion_check_period_sec);

    this->declare_parameter<std::string>(
        "save_directory",
        storage_config_.save_directory);
    this->declare_parameter<std::string>(
        "saved_map_prefix",
        storage_config_.saved_map_prefix);
    this->declare_parameter<std::string>(
        "map_saver_service_name",
        storage_config_.map_saver_service_name);
    this->declare_parameter<std::string>(
        "image_format",
        storage_config_.image_format);
    this->declare_parameter<std::string>(
        "map_mode",
        storage_config_.map_mode);
    this->declare_parameter<double>("free_thresh", storage_config_.free_thresh);
    this->declare_parameter<double>("occupied_thresh", storage_config_.occupied_thresh);
    this->declare_parameter<double>(
        "service_wait_timeout_sec",
        storage_config_.service_wait_timeout_sec);
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
    config_.completion_no_frontier_rounds =
        static_cast<int>(this->get_parameter("completion_no_frontier_rounds").as_int());
    config_.completion_unknown_delta_threshold =
        this->get_parameter("completion_unknown_delta_threshold").as_double();
    config_.completion_check_window_sec =
        this->get_parameter("completion_check_window_sec").as_double();
    config_.completion_check_period_sec =
        this->get_parameter("completion_check_period_sec").as_double();

    storage_config_.save_directory = this->get_parameter("save_directory").as_string();
    storage_config_.saved_map_prefix = this->get_parameter("saved_map_prefix").as_string();
    storage_config_.map_saver_service_name =
        this->get_parameter("map_saver_service_name").as_string();
    storage_config_.map_topic = config_.map_topic;
    storage_config_.image_format = this->get_parameter("image_format").as_string();
    storage_config_.map_mode = this->get_parameter("map_mode").as_string();
    storage_config_.free_thresh = this->get_parameter("free_thresh").as_double();
    storage_config_.occupied_thresh = this->get_parameter("occupied_thresh").as_double();
    storage_config_.service_wait_timeout_sec =
        this->get_parameter("service_wait_timeout_sec").as_double();
}

void MapManagerNode::apply_params()
{
    config_.completion_no_frontier_rounds =
        std::max(1, config_.completion_no_frontier_rounds);
    config_.completion_unknown_delta_threshold =
        std::clamp(config_.completion_unknown_delta_threshold, 0.0, 1.0);
    config_.completion_check_window_sec =
        std::max(1.0, config_.completion_check_window_sec);
    config_.completion_check_period_sec =
        std::max(0.5, config_.completion_check_period_sec);

    storage_config_.free_thresh = std::clamp(storage_config_.free_thresh, 0.0, 1.0);
    storage_config_.occupied_thresh = std::clamp(storage_config_.occupied_thresh, 0.0, 1.0);
    storage_config_.service_wait_timeout_sec =
        std::max(0.1, storage_config_.service_wait_timeout_sec);

    RCLCPP_INFO(
        logger_,
        "Map manager params: map_topic=%s, state_topic=%s, auto_save=%s, "
        "manager_state_topic=%s, final_map_topic=%s, no_frontier_rounds=%d, "
        "unknown_delta=%.6f, window=%.1fs",
        config_.map_topic.c_str(),
        config_.exploration_state_topic.c_str(),
        config_.enable_auto_save ? "true" : "false",
        config_.map_manager_state_topic.c_str(),
        config_.final_map_topic.c_str(),
        config_.completion_no_frontier_rounds,
        config_.completion_unknown_delta_threshold,
        config_.completion_check_window_sec);
    RCLCPP_INFO(
        logger_,
        "Map save params: directory=%s, prefix=%s, service=%s",
        storage_config_.save_directory.c_str(),
        storage_config_.saved_map_prefix.c_str(),
        storage_config_.map_saver_service_name.c_str());
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

    map_storage_ = std::make_unique<MapStorage>(*this, logger_, storage_config_);

    completion_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(config_.completion_check_period_sec),
        std::bind(&MapManagerNode::completion_timer_callback, this));
}

void MapManagerNode::publish_state(std::uint8_t state, const std::string & detail)
{
    if (!state_pub_) {
        return;
    }

    MapManagerStateMsg msg;
    msg.stamp = this->now();
    msg.state = state;
    msg.state_text = map_manager_state_to_string(state);
    msg.map_ready = save_succeeded_;
    msg.save_requested = save_requested_;
    msg.save_succeeded = save_succeeded_;
    msg.map_url = saved_map_url_;
    msg.detail = detail;
    msg.unknown_ratio = map_stats_.unknown_ratio;
    msg.width = map_stats_.width;
    msg.height = map_stats_.height;
    msg.resolution = map_stats_.resolution;
    state_pub_->publish(msg);
}

void MapManagerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(logger_, "Received null map message.");
        return;
    }

    latest_map_ = msg;
    const auto previous_unknown_ratio = map_stats_.unknown_ratio;
    map_stats_.width = msg->info.width;
    map_stats_.height = msg->info.height;
    map_stats_.resolution = msg->info.resolution;
    map_stats_.unknown_ratio = calculate_unknown_ratio(*msg);
    map_stats_.valid = msg->info.width > 0U && msg->info.height > 0U && !msg->data.empty();

    update_unknown_history(this->now(), map_stats_.unknown_ratio);
    if (!save_succeeded_) {
        publish_state(MapManagerStateMsg::MAP_RECEIVED, "map_updated");
    }

    RCLCPP_INFO_THROTTLE(
        logger_,
        *this->get_clock(),
        5000,
        "Received map: width=%u, height=%u, resolution=%.3f, unknown_ratio=%.6f",
        map_stats_.width,
        map_stats_.height,
        map_stats_.resolution,
        map_stats_.unknown_ratio);

    if (std::abs(previous_unknown_ratio - map_stats_.unknown_ratio) >
        config_.completion_unknown_delta_threshold)
    {
        RCLCPP_INFO(
            logger_,
            "Unknown ratio updated: old=%.6f, new=%.6f",
            previous_unknown_ratio,
            map_stats_.unknown_ratio);
    }
}

void MapManagerNode::exploration_state_callback(const ExplorationStateMsg::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(logger_, "Received null exploration state message.");
        return;
    }

    last_exploration_state_ = msg->state;
    last_exploration_detail_ = msg->detail;

    if (is_no_frontier_state(*msg)) {
        ++no_frontier_rounds_;
        RCLCPP_WARN(
            logger_,
            "No valid frontier state observed: rounds=%d/%d, detail=%s",
            no_frontier_rounds_,
            config_.completion_no_frontier_rounds,
            msg->detail.c_str());
    } else if (msg->state == msg->RUNNING || msg->state == msg->IDLE) {
        if (no_frontier_rounds_ > 0) {
            RCLCPP_INFO(
                logger_,
                "Reset no-frontier rounds because exploration state is %s.",
                exploration_state_to_string(msg->state).c_str());
        }
        no_frontier_rounds_ = 0;
    }
}

void MapManagerNode::completion_timer_callback()
{
    if (!config_.enable_auto_save || save_requested_ || save_succeeded_) {
        return;
    }

    if (!should_mark_completed()) {
        RCLCPP_DEBUG(
            logger_,
            "Completion check not satisfied: has_map=%s, no_frontier=%d/%d, "
            "unknown_stable=%s, state=%s",
            map_stats_.valid ? "true" : "false",
            no_frontier_rounds_,
            config_.completion_no_frontier_rounds,
            is_unknown_ratio_stable() ? "true" : "false",
            exploration_state_to_string(last_exploration_state_).c_str());
        return;
    }

    RCLCPP_WARN(
        logger_,
        "Exploration completed by engineering rule: no_frontier_rounds=%d, "
        "unknown_ratio=%.6f, state=%s, detail=%s",
        no_frontier_rounds_,
        map_stats_.unknown_ratio,
        exploration_state_to_string(last_exploration_state_).c_str(),
        last_exploration_detail_.c_str());
    publish_state(MapManagerStateMsg::COMPLETION_DETECTED, "completion_detected");
    publish_final_map("completion_detected");
    trigger_save();
}

double MapManagerNode::calculate_unknown_ratio(const nav_msgs::msg::OccupancyGrid & map) const
{
    if (map.data.empty()) {
        return 1.0;
    }

    const auto unknown_count = std::count(map.data.begin(), map.data.end(), -1);
    return static_cast<double>(unknown_count) / static_cast<double>(map.data.size());
}

void MapManagerNode::update_unknown_history(
    const rclcpp::Time & stamp,
    double unknown_ratio)
{
    unknown_history_.emplace_back(stamp, unknown_ratio);
    const auto window = rclcpp::Duration::from_seconds(config_.completion_check_window_sec);
    while (!unknown_history_.empty() && stamp - unknown_history_.front().first > window) {
        unknown_history_.pop_front();
    }
}

bool MapManagerNode::is_unknown_ratio_stable() const
{
    if (unknown_history_.size() < 2U) {
        return false;
    }

    const auto window = rclcpp::Duration::from_seconds(config_.completion_check_window_sec);
    if (unknown_history_.back().first - unknown_history_.front().first < window) {
        return false;
    }

    auto minmax = std::minmax_element(
        unknown_history_.begin(),
        unknown_history_.end(),
        [](const auto & lhs, const auto & rhs) {
            return lhs.second < rhs.second;
        });

    const double delta = minmax.second->second - minmax.first->second;
    return delta <= config_.completion_unknown_delta_threshold;
}

bool MapManagerNode::is_no_frontier_state(const ExplorationStateMsg & msg) const
{
    if (msg.state != msg.STUCK) {
        return false;
    }

    return msg.detail.find("no_valid_frontier") != std::string::npos ||
        msg.detail.find("no_frontier") != std::string::npos;
}

bool MapManagerNode::should_mark_completed() const
{
    if (!map_stats_.valid) {
        return false;
    }

    if (last_exploration_state_ == ExplorationStateMsg::RUNNING) {
        return false;
    }

    if (no_frontier_rounds_ < config_.completion_no_frontier_rounds) {
        return false;
    }

    return is_unknown_ratio_stable();
}

void MapManagerNode::trigger_save()
{
    if (!map_storage_) {
        RCLCPP_ERROR(logger_, "Cannot save map because MapStorage is not initialized.");
        return;
    }

    save_requested_ = true;
    RCLCPP_WARN(logger_, "Triggering map save.");
    publish_state(MapManagerStateMsg::SAVING, "saving");
    const bool request_sent = map_storage_->save_current_map(
        [this](const MapSaveResult & result) {
            save_succeeded_ = result.success;
            saved_map_url_ = result.map_url;
            if (!result.success) {
                save_requested_ = false;
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
            publish_final_map("saved");
            publish_state(MapManagerStateMsg::SAVED, "saved");
        });

    if (!request_sent) {
        save_requested_ = false;
        publish_state(MapManagerStateMsg::SAVE_FAILED, "save_request_not_sent");
    }
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

std::string MapManagerNode::exploration_state_to_string(std::uint8_t state) const
{
    switch (state) {
        case ExplorationStateMsg::IDLE:
            return "IDLE";
        case ExplorationStateMsg::RUNNING:
            return "RUNNING";
        case ExplorationStateMsg::STOPPED:
            return "STOPPED";
        case ExplorationStateMsg::COMPLETED:
            return "COMPLETED";
        case ExplorationStateMsg::STUCK:
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
