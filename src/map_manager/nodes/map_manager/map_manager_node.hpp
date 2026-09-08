#pragma once

#include <memory>
#include <string>

#include "map_manager_core/map_manager_core.hpp"
#include "adapters/nav2_map_saver.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"
#include "robot_interfaces/msg/map_manager_state.hpp"

namespace map_manager
{

/// @brief: 地图管理节点参数，集中描述订阅 topic、完成判定和自动保存策略
struct MapManagerConfig
{
  /// @brief: 当前地图 topic
  std::string map_topic{"/map"};
  /// @brief: 探索状态 topic
  std::string exploration_state_topic{"/exploration_state"};
  /// @brief: 地图管理状态输出 topic
  std::string map_manager_state_topic{"/map_manager_state"};
  /// @brief: 探索完成后发布最终地图的 topic
  std::string final_map_topic{"/map_manager/final_map"};
  /// @brief: 是否启用探索完成后的自动保存
  bool enable_auto_save{true};
  /// @brief: 完成判定定时器周期，单位秒
  double completion_check_period_sec{1.0};
};

/// @brief: 统一管理当前地图、探索完成判定和地图自动保存
class MapManagerNode : public rclcpp::Node
{
public:
  /// @brief: 构造地图管理节点
  /// @param options ROS2 节点选项
  explicit MapManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ExplorationStateMsg = robot_interfaces::msg::ExplorationState;
  using MapManagerStateMsg = robot_interfaces::msg::MapManagerState;

  /// @brief: 声明节点参数
  void declare_params();

  /// @brief: 从 ROS 参数服务器加载节点参数
  void load_params();

  /// @brief: 对参数做边界修正
  void apply_params();

  /// @brief: 创建订阅、定时器和 Nav2 保存适配器
  void create_interfaces();

  /// @brief: 发布地图管理状态
  /// @param state 地图管理状态枚举值
  /// @param detail 状态说明
  void publish_state(std::uint8_t state, const std::string & detail = {});

  /// @brief: 处理 /map 更新并维护地图统计信息
  /// @param msg OccupancyGrid 地图消息
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /// @brief: 处理探索状态更新
  /// @param msg ExplorationState 状态消息
  void exploration_state_callback(const ExplorationStateMsg::SharedPtr msg);

  /// @brief: 周期检查探索是否完成，并在满足条件时触发保存
  void completion_timer_callback();

  /// @brief: 触发地图保存请求
  void trigger_save();

  /// @brief: 发布当前缓存的最终地图
  /// @param detail 发布原因说明
  void publish_final_map(const std::string & detail);

  /// @brief: 将探索状态码转换为可读字符串
  /// @param state Core 探索阶段
  /// @return: 状态名称
  std::string exploration_state_to_string(ExplorationPhase state) const;

  /// @brief: 将地图管理状态码转换为可读字符串
  /// @param state MapManagerState.state 字段
  /// @return: 状态名称
  std::string map_manager_state_to_string(std::uint8_t state) const;

  rclcpp::Logger logger_;
  MapManagerConfig config_;
  Nav2MapSaverConfig nav2_map_saver_config_;
  MapManagerCore core_;
  std::unique_ptr<Nav2MapSaver> nav2_map_saver_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<ExplorationStateMsg>::SharedPtr exploration_state_sub_;
  rclcpp::Publisher<MapManagerStateMsg>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr final_map_pub_;
  rclcpp::TimerBase::SharedPtr completion_timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
};

}  // namespace map_manager
