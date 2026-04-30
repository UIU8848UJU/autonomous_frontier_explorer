#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <string>

#include "map_storage.hpp"
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
    /// @brief: 是否启用探索完成后的自动保存
    bool enable_auto_save{true};
    /// @brief: 连续无有效 frontier 的检查次数阈值
    int completion_no_frontier_rounds{5};
    /// @brief: unknown ratio 在窗口内允许的最大变化量
    double completion_unknown_delta_threshold{0.002};
    /// @brief: unknown ratio 稳定性检查窗口，单位秒
    double completion_check_window_sec{20.0};
    /// @brief: 完成判定定时器周期，单位秒
    double completion_check_period_sec{2.0};
};

/// @brief: 当前地图统计信息
struct MapStatistics
{
    /// @brief: 地图宽度，单位 cell
    std::uint32_t width{0U};
    /// @brief: 地图高度，单位 cell
    std::uint32_t height{0U};
    /// @brief: 地图分辨率，单位 m/cell
    double resolution{0.0};
    /// @brief: unknown cell 占总 cell 的比例
    double unknown_ratio{1.0};
    /// @brief: 是否已经收到有效地图
    bool valid{false};
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

    /// @brief: 创建订阅、定时器和存储辅助对象
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

    /// @brief: 计算地图 unknown ratio
    /// @param map 输入地图
    /// @return: unknown cell 占比
    double calculate_unknown_ratio(const nav_msgs::msg::OccupancyGrid & map) const;

    /// @brief: 记录 unknown ratio 采样，用于稳定性判定
    /// @param stamp 当前节点时间
    /// @param unknown_ratio 当前 unknown 比例
    void update_unknown_history(const rclcpp::Time & stamp, double unknown_ratio);

    /// @brief: 判断 unknown ratio 是否在配置窗口内足够稳定
    /// @return: true 表示地图 unknown 比例已经稳定
    bool is_unknown_ratio_stable() const;

    /// @brief: 判断探索状态是否表示没有可用 frontier
    /// @param msg ExplorationState 状态消息
    /// @return: true 表示当前状态可计为一次无 frontier
    bool is_no_frontier_state(const ExplorationStateMsg & msg) const;

    /// @brief: 判断是否满足探索完成条件
    /// @return: true 表示可以认为探索完成
    bool should_mark_completed() const;

    /// @brief: 触发地图保存请求
    void trigger_save();

    /// @brief: 将探索状态码转换为可读字符串
    /// @param state ExplorationState.state 字段
    /// @return: 状态名称
    std::string exploration_state_to_string(std::uint8_t state) const;

    /// @brief: 将地图管理状态码转换为可读字符串
    /// @param state MapManagerState.state 字段
    /// @return: 状态名称
    std::string map_manager_state_to_string(std::uint8_t state) const;

    rclcpp::Logger logger_;
    MapManagerConfig config_;
    MapStorageConfig storage_config_;
    std::unique_ptr<MapStorage> map_storage_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<ExplorationStateMsg>::SharedPtr exploration_state_sub_;
    rclcpp::Publisher<MapManagerStateMsg>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr completion_timer_;

    MapStatistics map_stats_;
    std::deque<std::pair<rclcpp::Time, double>> unknown_history_;
    int no_frontier_rounds_{0};
    std::uint8_t last_exploration_state_{ExplorationStateMsg::IDLE};
    std::string last_exploration_detail_;
    std::string saved_map_url_;
    bool save_requested_{false};
    bool save_succeeded_{false};
};

}  // namespace map_manager
