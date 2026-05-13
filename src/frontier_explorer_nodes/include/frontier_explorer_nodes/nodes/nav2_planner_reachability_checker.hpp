#pragma once

#include "frontier_explorer_core/reachability/frontier_reachability_checker.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "frontier_explorer_core/types/frontier_explorer_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace frontier_explorer
{

/// @brief: 基于 Nav2 ComputePathToPose 的 planner 可达性检查器，不做 footprint 落脚碰撞验证
class Nav2PlannerReachabilityChecker : public FrontierReachabilityChecker
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

    /// @brief: 构造 Nav2 planner 可达性检查器
    /// @param node ROS2 节点指针，用于创建 action client 和读取时钟
    /// @param params frontier explorer 参数快照
    Nav2PlannerReachabilityChecker(
        rclcpp::Node * node,
        const FrontierExplorerParams & params);

    /// @brief: 调用 Nav2 ComputePathToPose 检查 planner 是否能生成路径
    /// @param start map frame 下的机器人起点位姿
    /// @param goal map frame 下的候选目标位姿
    /// @return: planner 可达性检查结果
    FrontierReachabilityResult check(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

private:
    rclcpp::Node * node_{nullptr};
    rclcpp::Logger logger_;
    FrontierExplorerParams params_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr client_;
};

}  // namespace frontier_explorer
