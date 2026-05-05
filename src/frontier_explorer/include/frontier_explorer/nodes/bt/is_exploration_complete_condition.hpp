#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nodes/exploration_bt_context.hpp"

namespace frontier_explorer
{

/// @brief: 判断探索是否已经完成的 BT condition 节点
class IsExplorationCompleteCondition : public BT::ConditionNode
{
public:
    /// @brief: 构造可插件化探索完成判断节点，从 BT blackboard 获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    IsExplorationCompleteCondition(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造探索完成判断节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    IsExplorationCompleteCondition(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 执行完成状态判断
    /// @return: 完成返回 SUCCESS，未完成返回 FAILURE
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<ExplorationBtContext> context_;
};

}  // namespace frontier_explorer
