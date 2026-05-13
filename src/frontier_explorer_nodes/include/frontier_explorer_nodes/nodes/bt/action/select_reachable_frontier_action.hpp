#pragma once

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "frontier_explorer_nodes/nodes/exploration_bt_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/check_pose_reachability.hpp"

namespace frontier_explorer
{

/// @brief: 从 frontier 候选列表中选择第一个 NavigationNode 判定可达的目标
class SelectReachableFrontierAction : public BT::StatefulActionNode
{
public:
    /// @brief: 构造可插件化可达 frontier 选择节点，从 BT blackboard 获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    SelectReachableFrontierAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造可达 frontier 选择节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    SelectReachableFrontierAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 首次 tick 时缓存候选列表并启动可达性检查
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 逐个调用 NavigationNode reachability 服务并选择目标
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT halt 时清理请求中标志
    void onHalted() override;

private:
    /// @brief: 发送当前候选的可达性检查请求
    /// @return: true 表示请求已发送
    bool send_current_request();

private:
    std::shared_ptr<ExplorationBtContext> context_;
    std::vector<ExplorationBtContext::FrontierCandidate> candidates_;
    std::size_t current_index_{0U};
    bool request_sent_{false};
    bool response_ready_{false};
    rclcpp::Time next_request_time_;
    robot_interfaces::srv::CheckPoseReachability::Response::SharedPtr last_response_;
};

}  // namespace frontier_explorer
