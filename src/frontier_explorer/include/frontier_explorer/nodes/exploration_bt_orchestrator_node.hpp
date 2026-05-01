#pragma once

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/exploration_bt_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace frontier_explorer
{

/// @brief: Exploration BT 编排器运行状态
enum class ExplorationBtOrchestratorState
{
    IDLE,
    RUNNING,
    COMPLETED,
    FAILED,
    CANCELLED
};

/// @brief: 基于 BehaviorTree.CPP 的探索编排节点
class ExplorationBtOrchestratorNode : public rclcpp::Node
{
public:
    /// @brief: 构造 Exploration BT 编排节点
    /// @param options ROS2 节点选项
    explicit ExplorationBtOrchestratorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    /// @brief: 注册当前进程内置的 Exploration BT 节点
    void register_bt_nodes();

    /// @brief: 处理启动探索服务请求
    /// @param request Trigger 请求
    /// @param response Trigger 响应
    void handle_start(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /// @brief: 处理停止探索服务请求
    /// @param request Trigger 请求
    /// @param response Trigger 响应
    void handle_stop(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /// @brief: 周期 tick BehaviorTree
    void tick_tree();

    /// @brief: 查询当前 BT 是否已判定探索完成
    /// @return: true 表示探索完成
    bool is_exploration_complete() const;

    /// @brief: 获取当前状态细节文本
    /// @return: 当前状态细节
    std::string current_detail() const;

    /// @brief: 发布 orchestrator 状态
    /// @param detail 状态细节文本
    void publish_state(const std::string & detail);

private:
    std::shared_ptr<ExplorationBtContext> context_;
    BT::BehaviorTreeFactory factory_;
    std::optional<BT::Tree> tree_;
    std::string bt_xml_file_;
    double tick_period_sec_{0.1};
    ExplorationBtOrchestratorState state_{ExplorationBtOrchestratorState::IDLE};

    rclcpp::Publisher<robot_interfaces::msg::ExplorationState>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace frontier_explorer
