#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "frontier_explorer_core/frontier_goal_provider.hpp"
#include "frontier_explorer_core/types/frontier_types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "frontier_explorer_core/types/frontier_explorer_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"
#include "robot_interfaces/srv/clear_frontier_blacklist.hpp"
#include "robot_interfaces/srv/get_exploration_state.hpp"
#include "robot_interfaces/srv/get_frontier_candidates.hpp"
#include "robot_interfaces/srv/get_next_frontier_goal.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "frontier_explorer_nodes/nodes/frontier_marker_publisher.hpp"

namespace frontier_explorer
{

/// @brief: FrontierExplorerNode 是 frontier 能力 ROS wrapper，负责 service/topic/marker/state，不负责导航编排
class FrontierExplorerNode : public rclcpp::Node
{
public:
    /// @brief: 构造 frontier explorer 能力节点
    /// @param options ROS2 节点选项
    explicit FrontierExplorerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    /// @brief: 声明 ROS 参数
    void declare_params();

    /// @brief: 从 ROS 参数服务器加载参数
    void load_params();

    /// @brief: 修正参数边界并配置 frontier provider
    void apply_params();

    /// @brief: 创建订阅、服务和 publisher
    void create_interfaces();

    /// @brief: 处理用于 frontier 检测的 /map 更新
    /// @param msg OccupancyGrid 地图消息
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /// @brief: 处理用于目标安全检查的 global costmap 更新
    /// @param msg OccupancyGrid costmap 消息
    void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /// @brief: 通过 TF 查询 map frame 下的机器人位姿并更新 provider
    /// @return: true 表示机器人位姿已成功更新
    bool update_robot_pose_from_tf();

    /// @brief: 发布 frontier 可视化 marker
    /// @param visualization provider 返回的可视化快照
    void publish_markers(const FrontierGoalVisualization & visualization);

    /// @brief: 发布 frontier 目标决策调试 JSON，供 exploration_learning 采集
    /// @param result 单目标决策结果
    void publish_decision_debug(const FrontierGoalResult & result);

    /// @brief: 发布 frontier 候选列表决策调试 JSON，供 exploration_learning 采集
    /// @param result 候选列表决策结果
    void publish_decision_debug(const FrontierCandidatesResult & result);

    /// @brief: 转义 JSON 字符串内容
    /// @param value 待转义字符串
    /// @return 不包含外层引号的 JSON 安全字符串
    std::string escape_json_string(const std::string & value) const;

    /// @brief: 获取当前状态字符串
    /// @return: 当前状态字符串
    std::string state_to_string() const;

    /// @brief: 处理外部请求下一个 frontier 目标
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_get_next_frontier_goal(
            const std::shared_ptr<robot_interfaces::srv::GetNextFrontierGoal::Request> request,
            std::shared_ptr<robot_interfaces::srv::GetNextFrontierGoal::Response> response);

    /// @brief: 处理外部请求 frontier 候选目标列表
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_get_frontier_candidates(
            const std::shared_ptr<robot_interfaces::srv::GetFrontierCandidates::Request> request,
            std::shared_ptr<robot_interfaces::srv::GetFrontierCandidates::Response> response);

    /// @brief: 处理外部通知 frontier 导航失败
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_mark_frontier_failed(
            const std::shared_ptr<robot_interfaces::srv::MarkFrontierFailed::Request> request,
            std::shared_ptr<robot_interfaces::srv::MarkFrontierFailed::Response> response);

    /// @brief: 处理清空 frontier 黑名单请求
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_clear_frontier_blacklist(
            const std::shared_ptr<robot_interfaces::srv::ClearFrontierBlacklist::Request> request,
            std::shared_ptr<robot_interfaces::srv::ClearFrontierBlacklist::Response> response);

    /// @brief: 返回当前探索能力状态
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_get_exploration_state(
            const std::shared_ptr<robot_interfaces::srv::GetExplorationState::Request> request,
            std::shared_ptr<robot_interfaces::srv::GetExplorationState::Response> response);

    /// @brief: 设置能力节点状态
    /// @param new_state 新状态
    /// @param detail 状态详情
    void set_state(ExplorationState new_state, const std::string & detail = {});

    /// @brief: 获取能力节点状态
    /// @return: 当前状态
    ExplorationState get_state() const;

    /// @brief: 将状态转换为字符串
    /// @param state 待转换状态
    /// @return: 状态字符串
    std::string state_to_string(ExplorationState state) const;

    /// @brief: 获取状态详情
    /// @return: 状态详情
    std::string state_detail() const;

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    FrontierExplorerParams params_;
    FrontierGoalProvider goal_provider_;
    std::unique_ptr<FrontierMarkerPublisher> marker_publisher_;

    std::atomic<ExplorationState> state_{ExplorationState::IDLE};
    mutable std::mutex state_mutex_;
    std::string state_detail_;

    rclcpp::Service<robot_interfaces::srv::GetFrontierCandidates>::SharedPtr get_frontier_candidates_srv_;
    rclcpp::Service<robot_interfaces::srv::GetNextFrontierGoal>::SharedPtr get_next_frontier_goal_srv_;
    rclcpp::Service<robot_interfaces::srv::MarkFrontierFailed>::SharedPtr mark_frontier_failed_srv_;
    rclcpp::Service<robot_interfaces::srv::ClearFrontierBlacklist>::SharedPtr clear_frontier_blacklist_srv_;
    rclcpp::Service<robot_interfaces::srv::GetExplorationState>::SharedPtr get_exploration_state_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_debug_pub_;
};

}  // namespace frontier_explorer
