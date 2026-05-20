#include "exploration_learning/data_collection/dataset_recorder_node.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace exploration_learning
{
namespace
{
constexpr int kDefaultSubscriptionDepth = 10;
constexpr int kOccupiedThreshold = 50;
}  // namespace

DatasetRecorderNode::DatasetRecorderNode(const rclcpp::NodeOptions & options)
: Node("dataset_recorder_node", options)
{
  declare_params();
  load_params();
  open_output_file();
  create_subscriptions();
}

DatasetRecorderNode::~DatasetRecorderNode()
{
  std::lock_guard<std::mutex> lock(file_mutex_);
  if (records_file_.is_open()) {
    records_file_.flush();
    records_file_.close();
  }
}

void DatasetRecorderNode::declare_params()
{
  declare_parameter<std::string>("episode_id", "manual_episode");
  declare_parameter<std::string>(
    "dataset_output_dir",
    "./datasets/frontier_exploration/raw");
  declare_parameter<bool>("record_map", true);
  declare_parameter<bool>("record_decision", true);
  declare_parameter<bool>("record_navigation_result", true);
  declare_parameter<int>("flush_every_n_records", 1);
}

void DatasetRecorderNode::load_params()
{
  episode_id_ = get_parameter("episode_id").as_string();
  dataset_output_dir_ = get_parameter("dataset_output_dir").as_string();
  record_map_ = get_parameter("record_map").as_bool();
  record_decision_ = get_parameter("record_decision").as_bool();
  record_navigation_result_ = get_parameter("record_navigation_result").as_bool();
  flush_every_n_records_ = std::max(1, static_cast<int>(
    get_parameter("flush_every_n_records").as_int()));
}

void DatasetRecorderNode::open_output_file()
{
  const std::filesystem::path episode_dir =
    std::filesystem::path(dataset_output_dir_) / episode_id_;
  std::filesystem::create_directories(episode_dir);

  const std::filesystem::path records_path = episode_dir / "records.jsonl";
  records_file_path_ = records_path.string();
  records_file_.open(records_path, std::ios::out | std::ios::app);
  if (!records_file_.is_open()) {
    throw std::runtime_error("failed to open dataset records file: " + records_file_path_);
  }

  RCLCPP_INFO(
    get_logger(),
    "Dataset recorder writing jsonl records to: %s",
    records_file_path_.c_str());
}

void DatasetRecorderNode::create_subscriptions()
{
  if (record_map_) {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(rclcpp::KeepLast(kDefaultSubscriptionDepth)).reliable(),
      std::bind(&DatasetRecorderNode::map_callback, this, std::placeholders::_1));
  }

  if (record_decision_) {
    decision_debug_sub_ = create_subscription<std_msgs::msg::String>(
      "/frontier_explorer/decision_debug_json",
      rclcpp::QoS(rclcpp::KeepLast(kDefaultSubscriptionDepth)).reliable(),
      std::bind(&DatasetRecorderNode::decision_debug_callback, this, std::placeholders::_1));
  }

  if (record_navigation_result_) {
    navigation_result_debug_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/navigation_result_debug_json",
      rclcpp::QoS(rclcpp::KeepLast(kDefaultSubscriptionDepth)).reliable(),
      std::bind(
        &DatasetRecorderNode::navigation_result_callback,
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
  payload << "{"
          << "\"width\":" << msg->info.width << ","
          << "\"height\":" << msg->info.height << ","
          << "\"resolution\":" << std::setprecision(9) << msg->info.resolution << ","
          << "\"known_cells\":" << known_cells << ","
          << "\"unknown_cells\":" << unknown_cells << ","
          << "\"occupied_cells\":" << occupied_cells << ","
          << "\"free_cells\":" << free_cells
          << "}";

  write_record("map_summary", payload.str());
}

void DatasetRecorderNode::decision_debug_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::ostringstream payload;
  payload << "{\"raw\":\"" << escape_json_string(msg->data) << "\"}";
  write_record("decision_debug_json", payload.str());
}

void DatasetRecorderNode::navigation_result_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::ostringstream payload;
  payload << "{\"raw\":\"" << escape_json_string(msg->data) << "\"}";
  write_record("navigation_result_debug_json", payload.str());
}

void DatasetRecorderNode::write_record(
  const std::string & record_type,
  const std::string & payload_json)
{
  std::lock_guard<std::mutex> lock(file_mutex_);
  if (!records_file_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Dataset records file is not open.");
    return;
  }

  records_file_ << "{"
                << "\"timestamp\":" << std::fixed << std::setprecision(6)
                << current_timestamp_sec() << ","
                << "\"episode_id\":\"" << escape_json_string(episode_id_) << "\","
                << "\"record_type\":\"" << escape_json_string(record_type) << "\","
                << "\"payload\":" << payload_json
                << "}\n";

  ++records_since_flush_;
  if (records_since_flush_ >= static_cast<std::size_t>(flush_every_n_records_)) {
    records_file_.flush();
    records_since_flush_ = 0U;
  }
}

std::string DatasetRecorderNode::escape_json_string(const std::string & value) const
{
  std::ostringstream escaped;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          escaped << "\\u"
                  << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(static_cast<unsigned char>(ch))
                  << std::dec << std::setfill(' ');
        } else {
          escaped << ch;
        }
        break;
    }
  }
  return escaped.str();
}

double DatasetRecorderNode::current_timestamp_sec() const
{
  return now().seconds();
}

}  // namespace exploration_learning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<exploration_learning::DatasetRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
