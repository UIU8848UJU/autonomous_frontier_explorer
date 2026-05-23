#pragma once

#include <memory>
#include <string>

#include "exploration_learning/collector/dataset_writer.hpp"
#include "exploration_learning/collector/event_buffer.hpp"
#include "exploration_learning/collector/data_record_plugin.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace exploration_learning::collector
{

/// @brief 通用数据采集节点。
///
/// DatasetRecorderNode 负责参数加载、ROS topic 订阅、事件缓存、插件调度和 JSONL 落盘。
/// 它不负责启动仿真、不修改在线 frontier 决策、不执行模型训练。
class DatasetRecorderNode : public rclcpp::Node
{
public:
  /// @brief 构造数据采集节点。
  /// @param options ROS 2 节点选项。
  explicit DatasetRecorderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief 析构时刷新并关闭输出文件。
  ~DatasetRecorderNode() override;

private:
  /// @brief 声明 ROS 参数。
  void declare_params();

  /// @brief 加载 ROS 参数并完成边界修正。
  void load_params();

  /// @brief 初始化 DatasetWriter 和 plugin。
  void initialize_pipeline();

  /// @brief 创建 ROS topic 订阅。
  void create_subscriptions();

  /// @brief 处理地图消息并生成 map_summary 事件。
  /// @param msg OccupancyGrid 地图消息。
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// @brief 处理 frontier decision debug JSON。
  /// @param msg std_msgs/String 消息。
  void decision_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief 处理 navigation result debug JSON。
  /// @param msg std_msgs/String 消息。
  void navigation_result_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief 处理探索状态消息。
  /// @param msg ExplorationState 消息。
  void exploration_state_callback(
    const robot_interfaces::msg::ExplorationState::SharedPtr msg);

  /// @brief 接收一个事件，写入缓存并交给插件生成 record。
  /// @param event topic 事件。
  void ingest_event(const TopicEvent & event);

  /// @brief 根据当前时间和前缀生成 episode id。
  /// @return episode id 字符串。
  std::string make_episode_id() const;

  /// @brief 获取当前时间戳，单位秒。
  /// @return 当前 ROS 时间秒数。
  double current_timestamp_sec() const;

  /// @brief 日志器。
  rclcpp::Logger logger_;

  /// @brief 输出目录。
  std::string output_dir_;

  /// @brief 当前 episode id。
  std::string episode_id_;

  /// @brief episode id 前缀。
  std::string episode_prefix_;

  /// @brief writer flush 阈值。
  int writer_flush_every_n_{1};

  /// @brief event buffer 时间窗口，单位秒。
  double event_buffer_duration_sec_{10.0};

  /// @brief 启用插件名称。
  std::string plugin_name_;

  /// @brief map topic。
  std::string map_topic_;

  /// @brief frontier decision topic。
  std::string decision_topic_;

  /// @brief navigation result topic。
  std::string navigation_result_topic_;

  /// @brief exploration state topic。
  std::string exploration_state_topic_;

  /// @brief 是否记录 map summary。
  bool record_map_{true};

  /// @brief 是否记录 frontier decision。
  bool record_decision_{true};

  /// @brief 是否记录 navigation result。
  bool record_navigation_result_{true};

  /// @brief 是否记录 exploration state。
  bool record_exploration_state_{true};

  /// @brief 事件缓存。
  EventBuffer event_buffer_;

  /// @brief JSONL writer。
  DatasetWriter writer_;

  /// @brief 当前启用的 record plugin。
  std::unique_ptr<IDataRecordPlugin> plugin_;

  /// @brief map 订阅。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  /// @brief frontier decision 订阅。
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr decision_sub_;

  /// @brief navigation result 订阅。
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_sub_;

  /// @brief exploration state 订阅。
  rclcpp::Subscription<robot_interfaces::msg::ExplorationState>::SharedPtr exploration_state_sub_;
};

}  // namespace exploration_learning::collector
