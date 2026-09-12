#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "exploration_bt/exploration_bt_context.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"

namespace exploration
{

/// @brief: 导航失败后通知 FrontierStrategyNode 处理失败事件的 BT 动作节点，不直接管理重试和黑名单
class MarkFrontierFailedAction : public BT::StatefulActionNode
{
public:
    /// @brief: 构造可插件化失败前沿标记节点，从 BT 黑板获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    MarkFrontierFailedAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造失败前沿标记节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context 探索 BT 共享上下文
    MarkFrontierFailedAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 首次执行时准备发送失败标记请求
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 轮询失败标记服务结果
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT 停止时清理请求中标志
    void onHalted() override;

private:
    std::shared_ptr<ExplorationBtContext> context_;
    bool request_sent_{false};
    bool response_ready_{false};
    robot_interfaces::srv::MarkFrontierFailed::Response::SharedPtr last_response_;
};

}  // 命名空间 exploration
