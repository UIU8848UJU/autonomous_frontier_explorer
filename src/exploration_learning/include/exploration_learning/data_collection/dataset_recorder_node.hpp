#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace exploration_learning
{

/// @brief frontier exploration 数据集记录节点。
///
/// DatasetRecorderNode 只负责在 ROS 图内接收数据并写入 jsonl 文件。
/// 它不负责启动 Gazebo、Nav2 或批量 episode；批量编排由工作空间根目录下的
/// scripts/run_dataset_collection.py 负责。
class DatasetRecorderNode : public rclcpp::Node
{
public:
  /// @brief 构造数据集记录节点，声明参数、创建目录、打开文件并建立订阅。
  /// @param options ROS 2 节点选项，供 launch 或组件化加载时传入。
  explicit DatasetRecorderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief 析构节点并关闭 jsonl 输出文件。
  ~DatasetRecorderNode() override;

private:
  /// @brief 声明节点运行参数。
  void declare_params();

  /// @brief 从 ROS 参数服务器读取节点运行参数。
  void load_params();

  /// @brief 创建 episode 输出目录并打开 records.jsonl。
  void open_output_file();

  /// @brief 创建 map、frontier decision 和 navigation result 的订阅。
  void create_subscriptions();

  /// @brief 处理 /map 消息并写入 map summary 记录。
  /// @param msg OccupancyGrid 地图消息。
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// @brief 处理 frontier 决策调试 JSON 字符串。
  /// @param msg std_msgs/String 消息，data 字段保存上游输出的 JSON 或文本。
  void decision_debug_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief 处理 navigation result 调试 JSON 字符串。
  /// @param msg std_msgs/String 消息，data 字段保存上游输出的 JSON 或文本。
  void navigation_result_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief 写入一行 jsonl 记录。
  /// @param record_type 记录类型，例如 map_summary、decision_debug_json。
  /// @param payload_json 已经格式化好的 JSON payload 字符串。
  void write_record(const std::string & record_type, const std::string & payload_json);

  /// @brief 将普通字符串转义为 JSON string 内容。
  /// @param value 待转义字符串。
  /// @return 可安全写入 JSON string 的转义结果，不包含外层引号。
  std::string escape_json_string(const std::string & value) const;

  /// @brief 获取当前 ROS 时间戳，单位秒。
  /// @return 当前节点时钟时间，转换为 double 秒。
  double current_timestamp_sec() const;

  /// @brief 当前 episode 标识。
  std::string episode_id_;

  /// @brief 数据集根输出目录。
  std::string dataset_output_dir_;

  /// @brief 是否记录 /map 的摘要信息。
  bool record_map_{true};

  /// @brief 是否记录 frontier 决策调试信息。
  bool record_decision_{true};

  /// @brief 是否记录导航结果调试信息。
  bool record_navigation_result_{true};

  /// @brief 每写入多少条记录 flush 一次文件。
  int flush_every_n_records_{1};

  /// @brief 当前 episode 的 records.jsonl 完整路径。
  std::string records_file_path_;

  /// @brief jsonl 输出文件流。
  std::ofstream records_file_;

  /// @brief 保护文件写入和记录计数的互斥锁。
  mutable std::mutex file_mutex_;

  /// @brief 已写入但可能尚未 flush 的记录数量。
  std::size_t records_since_flush_{0U};

  /// @brief /map 订阅。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  /// @brief frontier 决策调试 JSON 订阅。
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_debug_sub_;

  /// @brief navigation result 调试 JSON 订阅。
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_debug_sub_;
};

}  // namespace exploration_learning
