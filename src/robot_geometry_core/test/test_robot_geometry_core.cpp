#include <vector>

#include "gtest/gtest.h"
#include "robot_geometry_core/footprint_collision_checker.hpp"

namespace
{

grid_map_core::GridMap makeMap(std::int8_t value)
{
    grid_map_core::GridMap map;
    map.width = 5U;
    map.height = 5U;
    map.resolution = 1.0;
    map.data.assign(25U, value);
    return map;
}

TEST(RobotGeometryCore, AcceptsFreeFootprint)
{
    const auto result = robot_geometry_core::checkFootprint(
        makeMap(0),
        robot_geometry_core::Pose2D{2.5, 2.5, 0.0},
        robot_geometry_core::FootprintCollisionConfig{
            true, false, 51, robot_geometry_core::makeCircularFootprint(0.3, 0.0)});
    EXPECT_TRUE(result.valid);
}

TEST(RobotGeometryCore, RejectsOccupiedFootprint)
{
    auto map = makeMap(0);
    map.data[2U * map.width + 2U] = 100;
    const auto result = robot_geometry_core::checkFootprint(
        map,
        robot_geometry_core::Pose2D{2.5, 2.5, 0.0},
        robot_geometry_core::FootprintCollisionConfig{
            true, false, 51, robot_geometry_core::makeCircularFootprint(0.3, 0.0)});
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason, "footprint_collision");
}

}  // 匿名命名空间
