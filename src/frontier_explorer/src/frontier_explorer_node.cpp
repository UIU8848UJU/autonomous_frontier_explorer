#include "frontier_explorer/frontier_explorer_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_set>

namespace frontier_explorer
{

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: Node("frontier_explorer_node", options)
{
    declare_params();
    create_interfaces();

    // 秒级别
    explore_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(explore_period_sec_),
        std::bind(&FrontierExplorerNode::explore_timer_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "FrontierExplorerNode started.");
}

void FrontierExplorerNode::declare_params()
{
    this->declare_parameter<double>("explore_period_sec", 3.0);
    this->declare_parameter<int>("obstacle_search_radius_cells", 2);
    this->declare_parameter<int>("min_frontier_cluster_size", 5);
    this->declare_parameter<double>("min_goal_distance_m", 0.5);

    explore_period_sec_ = 
        this->get_parameter("explore_period_sec").as_double();
    obstacle_search_radius_cells_ = 
        this->get_parameter("obstacle_search_radius_cells").as_int();
    min_frontier_cluster_size_ = 
        this->get_parameter("min_frontier_cluster_size").as_int();
    min_goal_distance_m_ = 
        this->get_parameter("min_goal_distance_m").as_double();
}

void FrontierExplorerNode::create_interfaces()
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10),
        std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&FrontierExplorerNode::odom_callback, this, std::placeholders::_1));
    
    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

void FrontierExplorerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    if(!msg){
        RCLCPP_INFO(
        this->get_logger(),
        "First map received: width=%u, height=%u, resolution=%.3f",
        msg->info.width,
        msg->info.height,
        msg->info.resolution);
    }
    else if (
        map_msg_->info.width != msg->info.width ||
        map_msg_->info.height != msg->info.height)
    {
        RCLCPP_WARN(
        this->get_logger(),
        "Map size changed: old=(%u, %u), new=(%u, %u)",
        map_msg_->info.width,
        map_msg_->info.height,
        msg->info.width,
        msg->info.height);
    }
    
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 3000,
        "Receiving map updates..."
    );

    map_msg_ = msg;
}

void FrontierExplorerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_msg_ = msg;
}

bool FrontierExplorerNode::in_bounds(int row, int col) const
{
    if (!map_msg_) {
        return false;
    }
    return row >= 0 &&
         col >= 0 &&
         row < static_cast<int>(map_msg_->info.height) &&
         col < static_cast<int>(map_msg_->info.width);
}

bool FrontierExplorerNode::is_cell_free(int row, int col) const
{
    if (!in_bounds(row, col)) {
        return false;
    }
    
    const auto index = row * static_cast<int>(map_msg_->info.width) + col;
    return map_msg_->data[index] == 0;
}

bool FrontierExplorerNode::is_cell_unknown(int row, int col) const
{
    if (!in_bounds(row, col)) {
        return false;
    }

    const auto index = row * static_cast<int>(map_msg_->info.width) + col;
    return map_msg_->data[index] == -1;
}

bool FrontierExplorerNode::is_cell_obstacle(int row, int col) const
{
    if (!in_bounds(row, col)) {
        return true;
    }
    const auto index = row * static_cast<int>(map_msg_->info.width) + col;
    /// @note 暂定为50，因为真实的落地环境可能不是那么干净
    return map_msg_->data[index] > 50;
}

bool FrontierExplorerNode::update_robot_grid_position()
{
    if (!map_msg_ || !odom_msg_) {
        return false;
    }

    const double origin_x = map_msg_->info.origin.position.x;
    const double origin_y = map_msg_->info.origin.position.y;
    const double resolution = map_msg_->info.resolution;

    const double robot_x = odom_msg_->pose.pose.position.x;
    const double robot_y = odom_msg_->pose.pose.position.y;

    const int col = static_cast<int>((robot_x - origin_x) / resolution);
    const int row = static_cast<int>((robot_y - origin_y) / resolution);

    if (!in_bounds(row, col)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Robot grid position out of map bounds.");
        return false;
    }

    robot_grid_ = GridCell{row, col};
    return true;
}

std::vector<FrontierExplorerNode::GridCell> 
FrontierExplorerNode::detect_frontier_cells() const
{
    std::vector<GridCell> frontier_cells;
    if (!map_msg_) {
        return frontier_cells;
    }

    const int rows = static_cast<int>(map_msg_->info.height);
    const int cols = static_cast<int>(map_msg_->info.width);
    // 避免检查邻居时越界
    for (int r = 1; r < rows - 1; ++r) 
    {
        for (int c = 1; c < cols - 1; ++c) 
        {
            if (!is_cell_free(r, c)) 
            {
                continue;
            }
            bool has_unknown_neighbor = false;

            // 查看是否有未知的区域，从左下角开始找到自动找一下
            for (int dr = -1; dr <= 1 && !has_unknown_neighbor; ++dr) 
            {
                for (int dc = -1; dc <= 1; ++dc) 
                {
                    if (is_cell_unknown(r + dr, c + dc)) {
                        has_unknown_neighbor = true;
                        break;
                    }
                }
            }

        if (has_unknown_neighbor && is_frontier_cell_safe(GridCell{r, c})) 
        {
            frontier_cells.push_back(GridCell{r, c});
        }
        }
    }

    return frontier_cells;
}

bool FrontierExplorerNode::is_frontier_cell_safe(const GridCell & cell) const
{
    for (int dr = -obstacle_search_radius_cells_; 
         dr <= obstacle_search_radius_cells_; ++dr) 
    {
        for (int dc = -obstacle_search_radius_cells_;
             dc <= obstacle_search_radius_cells_; ++dc) 
            {
                if (is_cell_obstacle(cell.row + dr, cell.col + dc)) 
                {
                    return false;
                }
        }
    }
    return true;
}

std::vector<FrontierExplorerNode::FrontierCluster>
FrontierExplorerNode::cluster_frontiers(
    const std::vector<GridCell> & frontier_cells) const
{
    std::vector<FrontierCluster> clusters;
    std::unordered_set<GridCell, GridCellHash> frontier_set(
        frontier_cells.begin(), frontier_cells.end());
    std::unordered_set<GridCell, GridCellHash> visited;
    
    //对每个 frontier 做 BFS
    for (const auto & start : frontier_cells) {

        if (visited.count(start) > 0) {
            continue;
        }
            
        FrontierCluster cluster;
        std::queue<GridCell> q;
        q.push(start);
        visited.insert(start);

        // 栈递归
        while (!q.empty()) {
            auto current = q.front();
            q.pop();
            cluster.cells.push_back(current);
            
            for (int dr = -1; dr <= 1; ++dr) {

                for (int dc = -1; dc <= 1; ++dc) {

                    if (dr == 0 && dc == 0) {
                        continue;
                    }

                    GridCell next{current.row + dr, current.col + dc};
                    if (frontier_set.count(next) > 0 && visited.count(next) == 0) 
                    {
                        visited.insert(next);
                        q.push(next);
                    }
                }
            }
        }

        if (static_cast<int>(cluster.cells.size()) < min_frontier_cluster_size_) 
        {
            continue;
        }

        double sum_row = 0.0;
        double sum_col = 0.0;
        for (const auto & cell : cluster.cells) {
            sum_row += cell.row;
            sum_col += cell.col;
        }

        cluster.centroid = GridCell{
            static_cast<int>(std::round(sum_row / cluster.cells.size())),
            static_cast<int>(std::round(sum_col / cluster.cells.size()))
        };

        if (is_frontier_cell_safe(cluster.centroid)) {
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

bool FrontierExplorerNode::is_goal_far_enough(const GridCell & cell) const
{
    if (!robot_grid_ || !map_msg_) {
        return false;
    }
    
    const double resolution = map_msg_->info.resolution;
    const double dr = static_cast<double>(robot_grid_->row - cell.row);
    const double dc = static_cast<double>(robot_grid_->col - cell.col);
    const double dist = std::sqrt(dr * dr + dc * dc) * resolution;
    return dist >= min_goal_distance_m_;
}

std::optional<FrontierExplorerNode::GridCell>
FrontierExplorerNode::choose_best_frontier(
    const std::vector<FrontierCluster> & clusters) const
{
    if (!robot_grid_) {
        return std::nullopt;
    }
    
    double best_dist = std::numeric_limits<double>::max();
    std::optional<GridCell> best_cell;
    
    for (const auto & cluster : clusters) {
        const auto & c = cluster.centroid;
        
        if (!is_goal_far_enough(c)) {
            continue;
        }

        const double dr = static_cast<double>(robot_grid_->row - c.row);
        const double dc = static_cast<double>(robot_grid_->col - c.col);
        const double dist = std::sqrt(dr * dr + dc * dc);

        if (dist < best_dist) {
            best_dist = dist;
            best_cell = c;
        }
    }

    return best_cell;
}

/// @note: 暂时没有设置朝向优化，只设置了点的距离
geometry_msgs::msg::PoseStamped
FrontierExplorerNode::grid_to_goal_pose(const GridCell & cell) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    pose.pose.position.x =
        cell.col * map_msg_->info.resolution + map_msg_->info.origin.position.x;

    pose.pose.position.y =
        cell.row * map_msg_->info.resolution + map_msg_->info.origin.position.y;

    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    return pose;
}

void FrontierExplorerNode::send_navigation_goal(
    const geometry_msgs::msg::PoseStamped & pose)
{
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not ready.");
        return;
    }
    
    NavigateToPose::Goal goal;
    goal.pose = pose;
    
    RCLCPP_INFO(
        this->get_logger(),
        "Sending frontier goal: x=%.3f, y=%.3f",
        pose.pose.position.x, pose.pose.position.y);

    auto options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
    options.goal_response_callback =
        std::bind(&FrontierExplorerNode::goal_response_callback, this,
              std::placeholders::_1);
    options.result_callback =
        std::bind(&FrontierExplorerNode::result_callback, this,
              std::placeholders::_1);

    is_navigating_ = true;
    nav_client_->async_send_goal(goal, options);
}

void FrontierExplorerNode::goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Frontier goal rejected.");
        is_navigating_ = false;
        return;
    }

    goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Frontier goal accepted.");
}

void FrontierExplorerNode::result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
{
    is_navigating_ = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Frontier goal succeeded.");
        break;
        case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Frontier goal aborted.");
        break;
        case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Frontier goal canceled.");
        break;
        default:
        RCLCPP_WARN(this->get_logger(), "Frontier goal unknown result.");
        break;
    }
}

void FrontierExplorerNode::explore_timer_callback()
{
    if (is_navigating_) {
        RCLCPP_DEBUG(this->get_logger(), "Still navigating, skip exploration tick.");
        return;
    }

    if (!map_msg_) {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "No map data available.");
        return;
    }

    if (!update_robot_grid_position()) {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Robot grid position unavailable.");
        return;
    }

    const auto frontier_cells = detect_frontier_cells();
    const auto clusters = cluster_frontiers(frontier_cells);

    RCLCPP_INFO(
        this->get_logger(),
        "Detected frontier cells: %zu, clusters: %zu",
        frontier_cells.size(), clusters.size());

    const auto best_frontier = choose_best_frontier(clusters);
    if (!best_frontier.has_value()) {
        RCLCPP_WARN(
            this->get_logger(),
            "No valid frontier selected. robot_grid=(%d, %d), clusters=%zu",
            robot_grid_->row,
            robot_grid_->col,
            clusters.size());
    }
    else{
        RCLCPP_INFO(
            this->get_logger(),
            "Chosen frontier: row=%d, col=%d, robot=(%d, %d)",
            best_frontier->row,
            best_frontier->col,
            robot_grid_->row,
            robot_grid_->col);
    }

    const auto goal_pose = grid_to_goal_pose(best_frontier.value());
    send_navigation_goal(goal_pose);
}

}  // namespace frontier_explorer