#pragma once

#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"   //目标位姿
#include "nav2_msgs/action/navigate_to_pose.hpp"    //Nav2 的导航 action
#include "nav_msgs/msg/occupancy_grid.hpp"  //地图
#include "nav_msgs/msg/odometry.hpp"    //里程计
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace frontier_explorer
{

class FrontierExplorerNode : public rclcpp::Node
{
public:
    //  action起别名后面方便用
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit FrontierExplorerNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());   //这里是什么意思？

private:

    struct GridCell
    {
        int row;
        int col;

        bool operator==(const GridCell & other) const
        {
            return row == other.row && col == other.col;
        }
    };

    struct GridCellHash
    {
        std::size_t operator()(const GridCell & cell) const
        {
            //  手动构造简单哈希
            return (static_cast<std::size_t>(cell.row) << 32) ^
                 static_cast<std::size_t>(cell.col);
        }
    };

    // 聚类
    struct FrontierCluster
    {
        std::vector<GridCell> cells;
        GridCell centroid{};
    };

    // init
    void declare_params();
    void create_interfaces();

    // 回调callback
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void explore_timer_callback();

    bool update_robot_grid_position();
    bool is_cell_free(int row, int col) const;
    bool is_cell_unknown(int row, int col) const;
    bool is_cell_obstacle(int row, int col) const;
    bool in_bounds(int row, int col) const;




    bool is_frontier_cell_safe(const GridCell & cell) const;

    std::vector<GridCell> detect_frontier_cells() const;
    std::optional<GridCell> choose_best_frontier( 
        const std::vector<FrontierCluster> & clusters) const;

    std::vector<FrontierCluster> cluster_frontiers(
        const std::vector<GridCell> & frontier_cells) const;

    geometry_msgs::msg::PoseStamped grid_to_goal_pose(const GridCell & cell) const;

    bool is_goal_far_enough(const GridCell & cell) const;
    void send_navigation_goal(const geometry_msgs::msg::PoseStamped & pose);
    void goal_response_callback(
        const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr explore_timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    std::optional<GridCell> robot_grid_;
    /// @note 防止犯傻的停下来，先预留后面补充
    std::optional<GridCell> last_goal_grid_;    
    GoalHandleNavigateToPose::SharedPtr goal_handle_;

    bool is_navigating_{false};

    double explore_period_sec_{3.0};
    int obstacle_search_radius_cells_{2};
    int min_frontier_cluster_size_{5};
    double min_goal_distance_m_{0.5};
};



}  // namespace frontier_explorer