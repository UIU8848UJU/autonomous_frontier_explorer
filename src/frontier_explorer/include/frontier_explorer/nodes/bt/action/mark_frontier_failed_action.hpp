#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nodes/exploration_bt_context.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"

namespace frontier_explorer
{

/// @brief: 导航失败后通知 FrontierExplorerNode 处理失败事件的 BT action 节点，不直接管理 retry/blacklist
class MarkFrontierFailedAction : public BT::StatefulActionNode
{
public:
    /// @brief: 构造失败 frontier 标记节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    MarkFrontierFailedAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 首次 tick 时准备发送失败标记请求
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 轮询失败标记服务结果
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT halt 时清理请求中标志
    void onHalted() override;

private:
    std::shared_ptr<ExplorationBtContext> context_;
    bool request_sent_{false};
    bool response_ready_{false};
    robot_interfaces::srv::MarkFrontierFailed::Response::SharedPtr last_response_;
};

}  // namespace frontier_explorer
