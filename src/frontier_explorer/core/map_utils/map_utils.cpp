#include "map_utils.hpp"

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

    const auto index = static_cast<std::size_t>(row) * map.info.width 
        + static_cast<std::size_t>(col);

    return map.data[index] == 0;
}

bool is_cell_unknown(
  const nav_msgs::msg::OccupancyGrid & map, int row, int col)
{
    if (!in_bounds(map, row, col)) {
        return false;
    }

    const auto index = static_cast<std::size_t>(row) * map.info.width
         + static_cast<std::size_t>(col);

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

}  // namespace frontier_explorer