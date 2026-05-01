#pragma once

// 日志宏
#define __CLASS_NAME__ "FrontierExplorerNode"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"   //目标位姿
#include "nav2_msgs/action/navigate_to_pose.hpp"    //Nav2 的导航 action
#include "nav_msgs/msg/occupancy_grid.hpp"  //地图
#include "nav_msgs/msg/odometry.hpp"    //里程计
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "core/types/frontier_types.hpp"
#include "core/costmap/costmap_adapter.hpp"
#include "core/detector/frontier_detector.hpp"
#include "nodes/frontier_explorer_params.hpp"
#include "visualization/frontier_marker_publisher.hpp"
#include "core/selector/frontier_selector.hpp"
#include "core/utils/map_utils.hpp"
// 状态节点
#include "std_srvs/srv/trigger.hpp"
#include "robot_interfaces/srv/clear_frontier_blacklist.hpp"
#include "robot_interfaces/srv/get_exploration_state.hpp"
#include "robot_interfaces/srv/get_next_frontier_goal.hpp"
#include "robot_interfaces/srv/mark_frontier_failed.hpp"
#include "robot_interfaces/msg/exploration_state.hpp"

#include "friendly_logging/logging.h"

namespace frontier_explorer
{

/// @brief: 计算下一个 frontier 目标的结果结构
struct FrontierGoalResult
{
    bool success{false};
    geometry_msgs::msg::PoseStamped goal;
    uint16_t reason_code{0};
    std::string reason_text;
    float score{0.0F};
    float distance_m{0.0F};
    float clearance_m{0.0F};
    uint32_t raw_frontier_count{0U};
    uint32_t candidate_count{0U};
    uint32_t blacklist_count{0U};
    bool exploration_complete{false};
    bool recoverable{false};
    std::optional<GridCell> goal_cell;
};

class FrontierExplorerNode : public rclcpp::Node
{
public:
    //  action起别名后面方便用
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit FrontierExplorerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

    // init
    void declare_params();
    void load_params();
    void apply_params();
    void create_interfaces();

    // 回调callback
    /// @brief: 处理用于 frontier 检测的 /map 更新
    /// @param msg OccupancyGrid 地图消息
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /// @brief: 处理用于目标安全检查的 global costmap 更新
    /// @param msg OccupancyGrid costmap 消息
    void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /// @brief: 处理里程计更新
    /// @param msg Odometry 消息
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void explore_timer_callback();

    /// @brief: 计算下一个 frontier 目标，不直接触发导航
    /// @return: frontier 目标计算结果
    FrontierGoalResult compute_next_frontier_goal();

    bool update_robot_grid_position();

    void send_navigation_goal(const GridCell & goal_cell
        ,const geometry_msgs::msg::PoseStamped & pose);

    void goal_response_callback(
            const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /// @brief 控制面部分
    void publish_state();
    std::string state_to_string() const;

    void handle_start(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handle_stop(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /// @brief: 处理外部请求下一个 frontier 目标
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_get_next_frontier_goal(
            const std::shared_ptr<robot_interfaces::srv::GetNextFrontierGoal::Request> request,
            std::shared_ptr<robot_interfaces::srv::GetNextFrontierGoal::Response> response);

    /// @brief: 处理外部通知 frontier 导航失败
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_mark_frontier_failed(
            const std::shared_ptr<robot_interfaces::srv::MarkFrontierFailed::Request> request,
            std::shared_ptr<robot_interfaces::srv::MarkFrontierFailed::Response> response);

    /// @brief: 处理清空 frontier 黑名单请求
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_clear_frontier_blacklist(
            const std::shared_ptr<robot_interfaces::srv::ClearFrontierBlacklist::Request> request,
            std::shared_ptr<robot_interfaces::srv::ClearFrontierBlacklist::Response> response);

    /// @brief: 返回当前探索能力状态
    /// @param request 服务请求
    /// @param response 服务响应
    void handle_get_exploration_state(
            const std::shared_ptr<robot_interfaces::srv::GetExplorationState::Request> request,
            std::shared_ptr<robot_interfaces::srv::GetExplorationState::Response> response);

    void set_state(ExplorationState new_state, const std::string & detail = {});
    ExplorationState get_state() const;
    std::string state_to_string(ExplorationState state) const;
    std::string state_detail() const;

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr explore_timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    CostmapAdapter map_costmap_;
    CostmapAdapter global_costmap_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    std::optional<GridCell> robot_grid_;
    std::optional<GridCell> current_goal_grid_;
    GoalHandleNavigateToPose::SharedPtr goal_handle_;

    FrontierExplorerParams params_;
    FrontierDetector detector_;
    FrontierSelector selector_;
    std::unique_ptr<FrontierMarkerPublisher> marker_publisher_;

    bool is_navigating_{false};
    bool enable_internal_navigation_loop_{false};
    
    /// @note:后续可以设计为多机控制
    std::atomic<ExplorationState> state_{ExplorationState::IDLE};
    mutable std::mutex state_mutex_;
    std::string state_detail_;

    std::optional<rclcpp::Time> last_progress_time_;
    double last_progress_distance_{0.0};
    double initial_goal_distance_{0.0};
    std::atomic<float> goal_progress_{0.0f};
    rclcpp::Time last_map_update_time_;
    std::size_t consecutive_frontier_failures_{0};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<robot_interfaces::srv::GetNextFrontierGoal>::SharedPtr get_next_frontier_goal_srv_;
    rclcpp::Service<robot_interfaces::srv::MarkFrontierFailed>::SharedPtr mark_frontier_failed_srv_;
    rclcpp::Service<robot_interfaces::srv::ClearFrontierBlacklist>::SharedPtr clear_frontier_blacklist_srv_;
    rclcpp::Service<robot_interfaces::srv::GetExplorationState>::SharedPtr get_exploration_state_srv_;
    rclcpp::Publisher<robot_interfaces::msg::ExplorationState>::SharedPtr state_pub_;
    rclcpp::Publisher<robot_interfaces::msg::ExplorationState>::SharedPtr legacy_state_pub_;
};



}  // namespace frontier_explorer
