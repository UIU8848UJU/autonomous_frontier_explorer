#include <string>
#include <vector>

#include "frontier_explorer_ros/costmap/costmap_adapter.hpp"
#include "frontier_explorer_ros/geometry/footprint_collision_checker.hpp"
#include "gtest/gtest.h"
#include "navigation_core/footprint_goal_validator.hpp"
#include "navigation_core/path_safety_checker.hpp"
#include "navigation_core/path_utils.hpp"
#include "navigation_core/single_goal_gate.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_explorer
{
namespace navigation_core
{
namespace
{

nav_msgs::msg::OccupancyGrid make_grid(
    unsigned int width,
    unsigned int height,
    float resolution,
    const std::vector<int8_t> & data)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = resolution;
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.data = data;
    return grid;
}

CostmapAdapter make_costmap(const nav_msgs::msg::OccupancyGrid & grid)
{
    CostmapAdapter adapter(rclcpp::get_logger("navigation_core_test"));
    EXPECT_TRUE(adapter.updateFromOccupancyGrid(grid));
    return adapter;
}

nav_msgs::msg::Path make_path(double x0, double y0, double x1, double y1)
{
    nav_msgs::msg::Path path;
    path.poses.resize(2U);
    path.poses[0].pose.position.x = x0;
    path.poses[0].pose.position.y = y0;
    path.poses[1].pose.position.x = x1;
    path.poses[1].pose.position.y = y1;
    return path;
}

nav_msgs::msg::Path make_path_through(
    double x0, double y0,
    double x1, double y1,
    double x2, double y2)
{
    nav_msgs::msg::Path path;
    path.poses.resize(3U);
    path.poses[0].pose.position.x = x0;
    path.poses[0].pose.position.y = y0;
    path.poses[1].pose.position.x = x1;
    path.poses[1].pose.position.y = y1;
    path.poses[2].pose.position.x = x2;
    path.poses[2].pose.position.y = y2;
    return path;
}

TEST(PathUtils, ComputesSegmentLength)
{
    const auto path = make_path(0.0, 0.0, 3.0, 4.0);
    EXPECT_DOUBLE_EQ(pathLengthM(path), 5.0);
}

TEST(SingleGoalGate, SerializesAccess)
{
    SingleGoalGate gate;
    EXPECT_FALSE(gate.isActive());
    EXPECT_TRUE(gate.tryAcquire());
    EXPECT_TRUE(gate.isActive());
    EXPECT_FALSE(gate.tryAcquire());
    gate.release();
    EXPECT_FALSE(gate.isActive());
    EXPECT_TRUE(gate.tryAcquire());
    gate.release();
}

TEST(PathSafetyChecker, AcceptsFreePath)
{
    const auto grid = make_grid(5U, 5U, 1.0F, std::vector<int8_t>(25, 0));
    const auto costmap = make_costmap(grid);
    PathSafetyChecker checker(costmap);
    checker.configure(PathSafetyCheckerConfig{});

    std::string reason;
    double max_cost = 0.0;
    EXPECT_TRUE(checker.isSafe(make_path(0.5, 0.5, 4.5, 4.5), reason, max_cost));
    EXPECT_EQ(reason, "path_safe");
}

TEST(PathSafetyChecker, RejectsPathThroughObstacle)
{
    std::vector<int8_t> data(25, 0);
    data[2U * 5U + 2U] = 100;
    const auto grid = make_grid(5U, 5U, 1.0F, data);
    const auto costmap = make_costmap(grid);
    PathSafetyChecker checker(costmap);
    PathSafetyCheckerConfig config;
    config.allow_unknown = true;
    checker.configure(config);

    std::string reason;
    double max_cost = 0.0;
    EXPECT_FALSE(checker.isSafe(make_path_through(0.5, 0.5, 2.5, 2.5, 4.5, 4.5), reason, max_cost));
    EXPECT_EQ(reason, "path_crosses_high_cost");
}

TEST(FootprintGoalValidator, AcceptsFreeGoal)
{
    const auto grid = make_grid(5U, 5U, 1.0F, std::vector<int8_t>(25, 0));
    const auto costmap = make_costmap(grid);
    FootprintGoalValidator validator(costmap);
    FootprintCollisionCheckerConfig config;
    config.footprint = FootprintCollisionChecker::makeCircularFootprint(0.3, 0.0);
    validator.configure(config);

    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 2.5;
    goal.pose.position.y = 2.5;
    goal.pose.orientation.w = 1.0;

    std::string reason;
    double max_cost = 0.0;
    EXPECT_TRUE(validator.isGoalValid(goal, reason, max_cost));
}

TEST(FootprintGoalValidator, RejectsGoalOnObstacle)
{
    std::vector<int8_t> data(25, 0);
    data[2U * 5U + 2U] = 100;
    const auto grid = make_grid(5U, 5U, 1.0F, data);
    const auto costmap = make_costmap(grid);
    FootprintGoalValidator validator(costmap);
    FootprintCollisionCheckerConfig config;
    config.footprint = FootprintCollisionChecker::makeCircularFootprint(0.3, 0.0);
    validator.configure(config);

    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 2.5;
    goal.pose.position.y = 2.5;
    goal.pose.orientation.w = 1.0;

    std::string reason;
    double max_cost = 0.0;
    EXPECT_FALSE(validator.isGoalValid(goal, reason, max_cost));
    EXPECT_EQ(reason, "footprint_collision");
}

}  // namespace
}  // namespace navigation_core
}  // namespace frontier_explorer
