#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "frontier_explorer_nodes/nodes/exploration_bt_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/get_frontier_candidates.hpp"

namespace frontier_explorer
{

/// @brief: 请求 FrontierExplorerNode 计算 frontier 候选列表的 BT action 节点
class ComputeFrontierCandidatesAction : public BT::StatefulActionNode
{
public:
    /// @brief: 构造可插件化 frontier 候选计算节点，从 BT blackboard 获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    ComputeFrontierCandidatesAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造 frontier 候选计算节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    ComputeFrontierCandidatesAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 首次 tick 时准备请求候选列表
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 轮询候选列表服务结果并写入共享上下文
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT halt 时清理请求中标志
    void onHalted() override;

private:
    std::shared_ptr<ExplorationBtContext> context_;
    bool request_sent_{false};
    bool response_ready_{false};
    rclcpp::Time next_request_time_;
    robot_interfaces::srv::GetFrontierCandidates::Response::SharedPtr last_response_;
};

}  // namespace frontier_explorer
