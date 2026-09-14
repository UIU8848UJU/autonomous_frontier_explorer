#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "exploration_bt/exploration_bt_context.hpp"
#include "robot_interfaces/srv/get_frontier_candidates.hpp"

namespace exploration
{

/// @brief 在当前导航期间异步准备下一轮 frontier 候选，结束时由同步流程再次复核。
class PrefetchFrontierCandidatesAction : public BT::StatefulActionNode
{
public:
    PrefetchFrontierCandidatesAction(
        const std::string & name,
        const BT::NodeConfiguration & config);

    PrefetchFrontierCandidatesAction(
        const std::string & name,
        const BT::NodeConfiguration & config,
        const std::shared_ptr<ExplorationBtContext> & context);

    /// @brief 该节点通过共享上下文工作，不暴露 BT 端口。
    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    struct RequestState
    {
        std::mutex mutex;
        bool sent{false};
        bool ready{false};
        robot_interfaces::srv::GetFrontierCandidates::Response::SharedPtr response;
    };

    std::shared_ptr<ExplorationBtContext> context_;
    std::shared_ptr<RequestState> request_state_;
    bool response_consumed_{false};
    bool prefetch_finished_{false};
    bool prefetch_success_{false};
};

}  // 命名空间 exploration
