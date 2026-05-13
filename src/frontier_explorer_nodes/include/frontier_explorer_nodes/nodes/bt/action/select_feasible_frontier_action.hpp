#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "frontier_explorer_nodes/nodes/exploration_bt_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/check_goal_feasibility.hpp"

namespace frontier_explorer
{

/// @brief: 从 frontier 候选列表中选择 NavigationNode 判定可执行且综合代价最优的目标
class SelectFeasibleFrontierAction : public BT::StatefulActionNode
{
public:
    /// @brief: 构造可插件化可执行 frontier 选择节点，从 BT blackboard 获取共享上下文
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    SelectFeasibleFrontierAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    /// @brief: 构造可执行 frontier 选择节点
    /// @param name BT 节点实例名
    /// @param config BT 节点配置
    /// @param context Exploration BT 共享上下文
    SelectFeasibleFrontierAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief: 声明 BT 端口；当前节点通过共享上下文通信，暂无 XML 端口
    /// @return: 空端口列表
    static BT::PortsList providedPorts()
    {
        return {};
    }

    /// @brief: 首次 tick 时缓存候选列表并启动可执行性检查
    /// @return: BT 节点状态
    BT::NodeStatus onStart() override;

    /// @brief: 逐个调用 NavigationNode feasibility 服务并选择目标
    /// @return: BT 节点状态
    BT::NodeStatus onRunning() override;

    /// @brief: BT halt 时清理请求中标志
    void onHalted() override;

private:
    /// @brief: 发送当前候选的可执行性检查请求
    /// @return: true 表示请求已发送或正在等待服务
    bool send_current_request();

    /// @brief: 判断新可执行候选是否优于当前最佳候选
    /// @param candidate 新候选
    /// @param best_candidate 当前最佳候选
    /// @return: true 表示新候选更适合导航
    bool is_better_feasible_candidate(
        const ExplorationBtContext::FrontierCandidate & candidate,
        const ExplorationBtContext::FrontierCandidate & best_candidate) const;

private:
    std::shared_ptr<ExplorationBtContext> context_;
    std::vector<ExplorationBtContext::FrontierCandidate> candidates_;
    std::vector<uint32_t> recoverable_retry_counts_;
    std::size_t current_index_{0U};
    std::optional<std::size_t> best_feasible_index_;
    bool saw_recoverable_failure_{false};
    bool request_sent_{false};
    bool response_ready_{false};
    rclcpp::Time next_request_time_;
    robot_interfaces::srv::CheckGoalFeasibility::Response::SharedPtr last_response_;
};

}  // namespace frontier_explorer
