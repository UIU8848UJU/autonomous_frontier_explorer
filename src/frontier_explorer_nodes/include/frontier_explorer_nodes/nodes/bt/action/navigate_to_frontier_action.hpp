#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "frontier_explorer_nodes/nodes/exploration_bt_context.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace frontier_explorer
{

/// @brief: 将当前 frontier goal 发送给 Nav2 NavigateToPose 的 BT action 节点，后续可替换为 NavigationNode 调用
class NavigateToFrontierAction : public BT::StatefulActionNode
{
public:
    using NavigateToPose = robot_interfaces::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /// @brief: 构造可插件化 frontier 导航节点，从 BT blackboard 获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    NavigateToFrontierAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造 frontier 导航节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    NavigateToFrontierAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 首次 tick 时发送 Nav2 NavigateToPose goal
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 轮询 Nav2 action 结果
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT halt 时取消正在执行的导航目标
    void onHalted() override;

private:
    std::shared_ptr<ExplorationBtContext> context_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    bool accepted_{false};
    bool rejected_{false};
    bool result_ready_{false};
    bool result_success_{false};
};

}  // namespace frontier_explorer
