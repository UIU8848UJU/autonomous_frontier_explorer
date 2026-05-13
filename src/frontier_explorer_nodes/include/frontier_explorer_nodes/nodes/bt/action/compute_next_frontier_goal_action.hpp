#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "frontier_explorer_nodes/nodes/exploration_bt_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/get_next_frontier_goal.hpp"

namespace frontier_explorer
{

/// @brief: 调用 FrontierExplorerNode 计算下一个 frontier goal 的 BT action 节点
class ComputeNextFrontierGoalAction : public BT::StatefulActionNode
{
public:
    /// @brief: 构造可插件化 frontier goal 计算节点，从 BT blackboard 获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    ComputeNextFrontierGoalAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造 frontier goal 计算节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    ComputeNextFrontierGoalAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 首次 tick 时发送或准备发送 frontier 请求
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 轮询 frontier 请求结果并更新 blackboard 上下文
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT halt 时清理请求中标志
    void onHalted() override;

private:
    std::shared_ptr<ExplorationBtContext> context_;
    bool request_sent_{false};
    bool response_ready_{false};
    rclcpp::Time next_request_time_;
    robot_interfaces::srv::GetNextFrontierGoal::Response::SharedPtr last_response_;
};

}  // namespace frontier_explorer
