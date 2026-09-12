#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "navigation_core/footprint_goal_validator.hpp"
#include "navigation_core/path_safety_checker.hpp"
#include "navigation_core/path_utils.hpp"
#include "navigation_core/single_goal_gate.hpp"

namespace navigation
{
namespace navigation_core
{
namespace
{

grid_map_core::GridMap make_map(const std::vector<std::int8_t> & data)
{
    grid_map_core::GridMap map;
    map.width = 5U;
    map.height = 5U;
    map.resolution = 1.0;
    map.data = data;
    return map;
}

Path2D make_path(double x0, double y0, double x1, double y1)
{
    return Path2D{{
        robot_geometry_core::Pose2D{x0, y0, 0.0},
        robot_geometry_core::Pose2D{x1, y1, 0.0}}};
}

TEST(PathUtils, ComputesSegmentLength)
{
    EXPECT_DOUBLE_EQ(pathLengthM(make_path(0.0, 0.0, 3.0, 4.0)), 5.0);
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
}

TEST(PathSafetyChecker, AcceptsFreePath)
{
    PathSafetyChecker checker;
    checker.configure(PathSafetyCheckerConfig{});
    std::string reason;
    double max_cost = 0.0;
    EXPECT_TRUE(checker.isSafe(
        make_path(0.5, 0.5, 4.5, 4.5),
        make_map(std::vector<std::int8_t>(25U, 0)),
        reason,
        max_cost));
    EXPECT_EQ(reason, "path_safe");
}

TEST(PathSafetyChecker, RejectsPathThroughObstacle)
{
    std::vector<std::int8_t> data(25U, 0);
    data[2U * 5U + 2U] = 100;
    PathSafetyChecker checker;
    checker.configure(PathSafetyCheckerConfig{});
    std::string reason;
    double max_cost = 0.0;
    const Path2D path{{
        {0.5, 0.5, 0.0}, {2.5, 2.5, 0.0}, {4.5, 4.5, 0.0}}};
    EXPECT_FALSE(checker.isSafe(path, make_map(data), reason, max_cost));
    EXPECT_EQ(reason, "path_crosses_high_cost");
}

TEST(FootprintGoalValidator, AcceptsFreeGoal)
{
    FootprintGoalValidator validator;
    validator.configure(robot_geometry_core::FootprintCollisionConfig{
        true, false, 51, robot_geometry_core::makeCircularFootprint(0.3, 0.0)});
    std::string reason;
    double max_cost = 0.0;
    EXPECT_TRUE(validator.isGoalValid(
        {2.5, 2.5, 0.0},
        make_map(std::vector<std::int8_t>(25U, 0)),
        reason,
        max_cost));
}

TEST(FootprintGoalValidator, RejectsGoalOnObstacle)
{
    std::vector<std::int8_t> data(25U, 0);
    data[2U * 5U + 2U] = 100;
    FootprintGoalValidator validator;
    validator.configure(robot_geometry_core::FootprintCollisionConfig{
        true, false, 51, robot_geometry_core::makeCircularFootprint(0.3, 0.0)});
    std::string reason;
    double max_cost = 0.0;
    EXPECT_FALSE(validator.isGoalValid(
        {2.5, 2.5, 0.0}, make_map(data), reason, max_cost));
    EXPECT_EQ(reason, "footprint_collision");
}

}  // namespace
}  // namespace navigation_core
}  // namespace navigation
