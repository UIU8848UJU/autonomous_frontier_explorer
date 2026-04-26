#include "map_utils.hpp"

#include <cmath>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"

namespace frontier_explorer
{

bool in_bounds(
    const nav_msgs::msg::OccupancyGrid & map, int row, int col)
{
    return row >= 0 &&
        col >= 0 &&
        row < static_cast<int>(map.info.height) &&
        col < static_cast<int>(map.info.width);
}

bool is_cell_free(
    const nav_msgs::msg::OccupancyGrid & map, int row, int col)
{
    if (!in_bounds(map, row, col)) {
        return false;
    }

    const auto index = static_cast<std::size_t>(row) * map.info.width +
        static_cast<std::size_t>(col);

    return map.data[index] == 0;
}

bool is_cell_unknown(
    const nav_msgs::msg::OccupancyGrid & map, int row, int col)
{
    if (!in_bounds(map, row, col)) {
        return false;
    }

    const auto index = static_cast<std::size_t>(row) * map.info.width +
        static_cast<std::size_t>(col);

    return map.data[index] == -1;
}

bool is_cell_obstacle(
    const nav_msgs::msg::OccupancyGrid & map, int row, int col)
{
    if (!in_bounds(map, row, col)) {
        return true;
    }

    const auto index = static_cast<std::size_t>(row) * map.info.width +
        static_cast<std::size_t>(col);

    return map.data[index] > 50;
}

geometry_msgs::msg::PoseStamped grid_to_goal_pose(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridCell & cell,
    const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = stamp;

    pose.pose.position.x = static_cast<double>(cell.col) * map.info.resolution +
        map.info.origin.position.x;
    pose.pose.position.y = static_cast<double>(cell.row) * map.info.resolution +
        map.info.origin.position.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    return pose;
}

std::optional<GridCell> world_to_grid(
    const geometry_msgs::msg::Point & pos,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map_msg)
{
    if (!map_msg) {
        return std::nullopt;
    }

    const int col = static_cast<int>(
        (pos.x - map_msg->info.origin.position.x) / map_msg->info.resolution);
    const int row = static_cast<int>(
        (pos.y - map_msg->info.origin.position.y) / map_msg->info.resolution);

    if (!in_bounds(*map_msg, row, col)) {
        return std::nullopt;
    }

    return GridCell{row, col};
}

std::optional<geometry_msgs::msg::Point> grid_to_world(
    const GridCell & cell,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map_msg)
{
    if (!map_msg) {
        return std::nullopt;
    }

    geometry_msgs::msg::Point p;
    p.x = cell.col * map_msg->info.resolution +
        map_msg->info.origin.position.x +
        map_msg->info.resolution / 2.0;
    p.y = cell.row * map_msg->info.resolution +
        map_msg->info.origin.position.y +
        map_msg->info.resolution / 2.0;
    p.z = 0.0;
    return p;
}

double distance_in_meters(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

}  // namespace frontier_explorer
