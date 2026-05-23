#include "exploration_learning/collector/dataset_recorder_node.hpp"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

#include "exploration_learning/collector/json_utils.hpp"
#include "exploration_learning/plugins/frontier_decision_plugin.hpp"

namespace exploration_learning::collector
{
namespace
{
constexpr int kDefaultSubscriptionDepth = 10;
constexpr int kOccupiedThreshold = 50;
constexpr const char * kMapSummaryTopic = "__map_summary";
}  // namespace

DatasetRecorderNode::DatasetRecorderNode(const rclcpp::NodeOptions & options)
: Node("dataset_recorder_node", options),
  logger_(get_logger()),
  event_buffer_(10.0),
  writer_(logger_)
{
  declare_params();
  load_params();
  initialize_pipeline();
  create_subscriptions();
}

DatasetRecorderNode::~DatasetRecorderNode()
{
  writer_.close();
}

void DatasetRecorderNode::declare_params()
{
  declare_parameter<std::string>("output_dir", "./datasets");
  declare_parameter<std::string>("episode_id", "");
  declare_parameter<std::string>("episode_prefix", "frontier_episode");
  declare_parameter<int>("writer_flush_every_n", 1);
  declare_parameter<double>("event_buffer_duration_sec", 10.0);
  declare_parameter<std::string>("plugin_name", "frontier_decision");

  declare_parameter<std::string>("map_topic", "/map");
  declare_parameter<std::string>(
    "decision_topic",
    "/frontier_explorer/decision_debug_json");
  declare_parameter<std::string>(
    "navigation_result_topic",
    "/navigation/navigation_result_debug_json");
  declare_parameter<std::string>("exploration_state_topic", "/frontier_explorer/state");

  declare_parameter<bool>("record_map", true);
  declare_parameter<bool>("record_decision", true);
  declare_parameter<bool>("record_navigation_result", true);
  declare_parameter<bool>("record_exploration_state", true);
}

void DatasetRecorderNode::load_params()
{
  output_dir_ = get_parameter("output_dir").as_string();
  episode_id_ = get_parameter("episode_id").as_string();
  episode_prefix_ = get_parameter("episode_prefix").as_string();
  writer_flush_every_n_ = std::max(1, static_cast<int>(
    get_parameter("writer_flush_every_n").as_int()));
  event_buffer_duration_sec_ = std::max(0.1, get_parameter(
    "event_buffer_duration_sec").as_double());
  plugin_name_ = get_parameter("plugin_name").as_string();

  map_topic_ = get_parameter("map_topic").as_string();
  decision_topic_ = get_parameter("decision_topic").as_string();
  navigation_result_topic_ = get_parameter("navigation_result_topic").as_string();
  exploration_state_topic_ = get_parameter("exploration_state_topic").as_string();

  record_map_ = get_parameter("record_map").as_bool();
  record_decision_ = get_parameter("record_decision").as_bool();
  record_navigation_result_ = get_parameter("record_navigation_result").as_bool();
  record_exploration_state_ = get_parameter("record_exploration_state").as_bool();

  if (episode_id_.empty()) {
    episode_id_ = make_episode_id();
  }
  event_buffer_.set_buffer_duration(event_buffer_duration_sec_);
}

void DatasetRecorderNode::initialize_pipeline()
{
  writer_.open(output_dir_, episode_id_, writer_flush_every_n_);

  std::ostringstream metadata;
  metadata << std::fixed << std::setprecision(6)
           << "{"
           << "\"episode_id\":\"" << escape_json_string(episode_id_) << "\","
           << "\"created_timestamp_sec\":" << current_timestamp_sec() << ","
           << "\"node_name\":\"" << escape_json_string(get_name()) << "\","
           << "\"plugin_name\":\"" << escape_json_string(plugin_name_) << "\","
           << "\"topics\":{"
           << "\"map\":\"" << escape_json_string(map_topic_) << "\","
           << "\"decision\":\"" << escape_json_string(decision_topic_) << "\","
           << "\"navigation_result\":\"" << escape_json_string(navigation_result_topic_) << "\","
           << "\"exploration_state\":\"" << escape_json_string(exploration_state_topic_) << "\""
           << "},"
           << "\"parameters\":{"
           << "\"output_dir\":\"" << escape_json_string(output_dir_) << "\","
           << "\"writer_flush_every_n\":" << writer_flush_every_n_ << ","
           << "\"event_buffer_duration_sec\":" << event_buffer_duration_sec_ << ","
           << "\"record_map\":" << (record_map_ ? "true" : "false") << ","
           << "\"record_decision\":" << (record_decision_ ? "true" : "false") << ","
           << "\"record_navigation_result\":"
           << (record_navigation_result_ ? "true" : "false") << ","
           << "\"record_exploration_state\":"
           << (record_exploration_state_ ? "true" : "false")
           << "}"
           << "}";
  writer_.write_metadata(metadata.str());

  if (plugin_name_ == "frontier_decision") {
    plugins::FrontierDecisionPluginConfig config;
    config.episode_id = episode_id_;
    config.decision_topic = decision_topic_;
    config.navigation_result_topic = navigation_result_topic_;
    config.map_summary_topic = kMapSummaryTopic;
    config.exploration_state_topic = exploration_state_topic_;
    plugin_ = std::make_unique<plugins::FrontierDecisionPlugin>(config);
  } else {
    RCLCPP_WARN(
      logger_,
      "Unknown dataset recorder plugin '%s'; no records will be emitted.",
      plugin_name_.c_str());
  }

  RCLCPP_INFO(
    logger_,
    "DatasetRecorderNode started: episode_id=%s output_dir=%s plugin=%s records=%s metadata=%s",
    episode_id_.c_str(),
    output_dir_.c_str(),
    plugin_name_.c_str(),
    writer_.records_file_path().c_str(),
    writer_.metadata_file_path().c_str());
}

void DatasetRecorderNode::create_subscriptions()
{
  if (record_map_) {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(kDefaultSubscriptionDepth)).reliable(),
      std::bind(&DatasetRecorderNode::map_callback, this, std::placeholders::_1));
  }

  if (record_decision_) {
    decision_sub_ = create_subscription<std_msgs::msg::String>(
      decision_topic_,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      std::bind(&DatasetRecorderNode::decision_callback, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(logger_, "Decision topic recording is disabled; plugin may emit no records.");
  }

  if (record_navigation_result_) {
    navigation_result_sub_ = create_subscription<std_msgs::msg::String>(
      navigation_result_topic_,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      std::bind(
        &DatasetRecorderNode::navigation_result_callback,
        this,
        std::placeholders::_1));
  }

  if (record_exploration_state_) {
    exploration_state_sub_ =
      create_subscription<robot_interfaces::msg::ExplorationState>(
      exploration_state_topic_,
      rclcpp::QoS(rclcpp::KeepLast(kDefaultSubscriptionDepth)).reliable(),
      std::bind(
        &DatasetRecorderNode::exploration_state_callback,
        this,
        std::placeholders::_1));
  }
}

void DatasetRecorderNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::size_t unknown_cells = 0U;
  std::size_t occupied_cells = 0U;
  std::size_t free_cells = 0U;

  for (const auto cell : msg->data) {
    if (cell < 0) {
      ++unknown_cells;
    } else if (cell > kOccupiedThreshold) {
      ++occupied_cells;
    } else {
      ++free_cells;
    }
  }

  const std::size_t known_cells = occupied_cells + free_cells;
  std::ostringstream payload;
  payload << std::fixed << std::setprecision(9)
          << "{"
          << "\"width\":" << msg->info.width << ","
          << "\"height\":" << msg->info.height << ","
          << "\"resolution\":" << msg->info.resolution << ","
          << "\"known_cells\":" << known_cells << ","
          << "\"unknown_cells\":" << unknown_cells << ","
          << "\"occupied_cells\":" << occupied_cells << ","
          << "\"free_cells\":" << free_cells << ","
          << "\"known_area_m2\":"
          << static_cast<double>(known_cells) * msg->info.resolution * msg->info.resolution
          << "}";

  ingest_event(TopicEvent{kMapSummaryTopic, now(), payload.str()});
}

void DatasetRecorderNode::decision_callback(const std_msgs::msg::String::SharedPtr msg)
{
  ingest_event(TopicEvent{decision_topic_, now(), wrap_raw_payload(msg->data)});
}

void DatasetRecorderNode::navigation_result_callback(const std_msgs::msg::String::SharedPtr msg)
{
  ingest_event(TopicEvent{navigation_result_topic_, now(), wrap_raw_payload(msg->data)});
}

void DatasetRecorderNode::exploration_state_callback(
  const robot_interfaces::msg::ExplorationState::SharedPtr msg)
{
  std::ostringstream payload;
  payload << "{"
          << "\"state\":" << static_cast<int>(msg->state) << ","
          << "\"detail\":\"" << escape_json_string(msg->detail) << "\","
          << "\"stamp_sec\":" << rclcpp::Time(msg->stamp).seconds()
          << "}";
  ingest_event(TopicEvent{exploration_state_topic_, now(), payload.str()});
}

void DatasetRecorderNode::ingest_event(const TopicEvent & event)
{
  event_buffer_.add_event(event);

  if (!plugin_) {
    return;
  }

  const auto records = plugin_->handle_event(event, event_buffer_);
  for (const auto & record : records) {
    writer_.append_decision_record(record);
  }
}

std::string DatasetRecorderNode::make_episode_id() const
{
  std::ostringstream episode_id;
  episode_id << episode_prefix_ << "_" << static_cast<std::uint64_t>(now().nanoseconds());
  return episode_id.str();
}

double DatasetRecorderNode::current_timestamp_sec() const
{
  return now().seconds();
}

}  // namespace exploration_learning::collector
