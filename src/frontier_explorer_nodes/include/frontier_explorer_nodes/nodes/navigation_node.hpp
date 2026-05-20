#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "frontier_explorer_core/costmap/costmap_adapter.hpp"
#include "frontier_explorer_core/geometry/footprint_collision_checker.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/navigate_to_pose.hpp"
#include "robot_interfaces/srv/check_goal_feasibility.hpp"
#include "robot_interfaces/srv/check_pose_reachability.hpp"
#include "std_msgs/msg/string.hpp"

namespace frontier_explorer
{

/// @brief: NavigationNode 封装 Nav2 NavigateToPose，对探索 BT 暴露稳定导航 action 能力
class NavigationNode : public rclcpp::Node
{
public:
    using NavigateToPose = robot_interfaces::action::NavigateToPose;
    using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
    using Nav2NavigateToPose = nav2_msgs::action::NavigateToPose;
    using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2NavigateToPose>;
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    /// @brief: 构造导航能力节点
    /// @param options ROS2 节点选项
    explicit NavigationNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~NavigationNode() override;

private:
    /// @brief: 处理外部导航 goal 请求
    /// @param uuid action goal UUID
    /// @param goal 导航目标
    /// @return: goal 接收策略
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal);

    /// @brief: 处理外部取消请求
    /// @param goal_handle 外部导航 goal handle
    /// @return: cancel 接收策略
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<NavigateGoalHandle> goal_handle);

    /// @brief: goal 接收后启动执行线程
    /// @param goal_handle 外部导航 goal handle
    void handle_accepted(const std::shared_ptr<NavigateGoalHandle> goal_handle);

    /// @brief: 执行一次导航请求并桥接 Nav2 action 结果
    /// @param goal_handle 外部导航 goal handle
    void execute_navigation(const std::shared_ptr<NavigateGoalHandle> goal_handle);

    /// @brief: 处理位姿可达性检查请求
    /// @param request 可达性检查请求
    /// @param response 可达性检查响应
    void handle_check_pose_reachability(
        const std::shared_ptr<robot_interfaces::srv::CheckPoseReachability::Request> request,
        std::shared_ptr<robot_interfaces::srv::CheckPoseReachability::Response> response);

    /// @brief: 处理目标导航可执行性检查请求
    /// @param request 可执行性检查请求
    /// @param response 可执行性检查响应
    void handle_check_goal_feasibility(
        const std::shared_ptr<robot_interfaces::srv::CheckGoalFeasibility::Request> request,
        std::shared_ptr<robot_interfaces::srv::CheckGoalFeasibility::Response> response);

    /// @brief: 调用 Nav2 ComputePathToPose 并填充可达性结果
    /// @param request 可达性检查请求
    /// @param response 可达性检查响应
    /// @param path 输出规划路径；为空时只填充 response
    void compute_path_reachability(
        const robot_interfaces::srv::CheckPoseReachability::Request & request,
        robot_interfaces::srv::CheckPoseReachability::Response & response,
        nav_msgs::msg::Path * path);

    /// @brief: 更新用于 footprint 落脚碰撞检查的 costmap
    /// @param msg OccupancyGrid costmap 消息
    void footprint_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /// @brief: 检查目标位姿处 robot footprint 是否能安全落脚
    /// @param goal 待检查目标位姿
    /// @param reason 输出失败原因
    /// @param footprint_cost 输出 footprint 代价值
    /// @return: true 表示 footprint 未碰撞
    bool is_goal_footprint_valid(
        const geometry_msgs::msg::PoseStamped & goal,
        std::string & reason,
        double & footprint_cost) const;

    /// @brief: 检查规划路径是否穿越 unknown 或高代价区域
    /// @param path Nav2 planner 返回的路径
    /// @param reason 输出失败原因
    /// @param max_path_cost 输出路径上最大代价值
    /// @return: true 表示路径中心线没有穿越禁止区域
    bool is_path_costmap_safe(
        const nav_msgs::msg::Path & path,
        std::string & reason,
        double & max_path_cost) const;

    /// @brief: 发布导航结果调试 JSON，供 exploration_learning 采集
    /// @param event 事件类型，例如 feasibility_check 或 navigation_result
    /// @param success 请求是否成功
    /// @param accepted_or_feasible goal 是否被接受或可执行
    /// @param result_code 结果码
    /// @param message 结果描述
    /// @param goal 目标位姿
    /// @param path_length_m 路径长度，未知时可为 0
    void publish_navigation_debug(
        const std::string & event,
        bool success,
        bool accepted_or_feasible,
        uint16_t result_code,
        const std::string & message,
        const geometry_msgs::msg::PoseStamped & goal,
        double path_length_m) const;

    /// @brief: 转义 JSON 字符串内容
    /// @param value 待转义字符串
    /// @return 不包含外层引号的 JSON 安全字符串
    std::string escape_json_string(const std::string & value) const;

private:
    rclcpp::Logger logger_;
    CostmapAdapter footprint_costmap_;
    std::string nav2_action_name_;
    std::string compute_path_action_name_;
    std::string default_planner_id_;
    std::string footprint_costmap_topic_;
    std::chrono::milliseconds nav2_server_timeout_{500};
    std::chrono::milliseconds reachability_timeout_{500};
    bool enable_footprint_collision_check_{true};
    bool allow_unknown_footprint_{false};
    bool enable_path_safety_check_{true};
    bool allow_unknown_path_{false};
    double robot_radius_{0.1};
    double footprint_padding_{0.0};
    unsigned char footprint_cost_threshold_{253U};
    unsigned char path_cost_threshold_{253U};
    FootprintCollisionCheckerConfig footprint_collision_config_;
    std::mutex nav2_goal_mutex_;
    /// @brief: 标记当前是否已有一个外部探索导航 goal 正在执行或取消中
    bool navigation_goal_active_{false};
    Nav2GoalHandle::SharedPtr active_nav2_goal_;
    std::atomic_bool navigation_stop_requested_{false};
    std::mutex navigation_thread_mutex_;
    std::thread navigation_thread_;
    mutable std::mutex footprint_costmap_mutex_;
    rclcpp::CallbackGroup::SharedPtr navigation_callback_group_;

    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp_action::Client<Nav2NavigateToPose>::SharedPtr nav2_client_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
    rclcpp::Service<robot_interfaces::srv::CheckPoseReachability>::SharedPtr reachability_srv_;
    rclcpp::Service<robot_interfaces::srv::CheckGoalFeasibility>::SharedPtr feasibility_srv_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr footprint_costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_debug_pub_;
};

}  // namespace frontier_explorer
