#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "robot_geometry_core/footprint_collision_checker.hpp"
#include "robot_geometry_core/robot_geometry_provider.hpp"

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

TEST(RobotGeometryCore, CircularProviderIncludesSafetyPadding)
{
    const auto envelope = robot_geometry_core::makeCircularCollisionEnvelope(
        0.10, 0.02, "base_link", "gazebo_model", 7U);
    const robot_geometry_core::StaticRobotGeometryProvider provider(envelope);

    const robot_geometry_core::IRobotGeometryProvider & interface = provider;
    const auto snapshot = interface.collisionEnvelope();
    EXPECT_GE(snapshot.circumscribed_radius, 0.12);
    EXPECT_LT(snapshot.circumscribed_radius, 0.121);
    EXPECT_DOUBLE_EQ(snapshot.safety_padding, 0.02);
    EXPECT_EQ(snapshot.frame_id, "base_link");
    EXPECT_EQ(snapshot.source, "gazebo_model");
    EXPECT_EQ(snapshot.revision, 7U);
    ASSERT_EQ(snapshot.footprint.points.size(), 32U);
    constexpr double kPi = 3.14159265358979323846;
    const double edge_midpoint_angle = kPi / 32.0;
    EXPECT_TRUE(robot_geometry_core::pointInPolygon(
        0.12 * 0.999 * std::cos(edge_midpoint_angle),
        0.12 * 0.999 * std::sin(edge_midpoint_angle),
        snapshot.footprint.points));
}

TEST(RobotGeometryCore, PolygonEnvelopeComputesCircumscribedRadius)
{
    robot_geometry_core::Footprint footprint;
    footprint.points = {
        {-0.20, -0.10},
        {0.30, -0.10},
        {0.30, 0.10},
        {-0.20, 0.10}};

    const auto envelope = robot_geometry_core::makePolygonCollisionEnvelope(
        footprint, "base_footprint", "wall_calibration", 3U);
    EXPECT_NEAR(envelope.circumscribed_radius, std::sqrt(0.10), 1e-9);
    EXPECT_EQ(envelope.footprint.points.size(), 4U);
    EXPECT_EQ(envelope.source, "wall_calibration");
}

TEST(RobotGeometryCore, StaticProviderRejectsInvalidEnvelope)
{
    robot_geometry_core::RobotCollisionEnvelope invalid;
    invalid.footprint.points = {{0.0, 0.0}, {1.0, 0.0}};
    invalid.circumscribed_radius = 1.0;
    EXPECT_THROW(
        robot_geometry_core::StaticRobotGeometryProvider provider(invalid),
        std::invalid_argument);
}

TEST(RobotGeometryCore, StaticProviderRejectsUndersizedCircumscribedRadius)
{
    robot_geometry_core::RobotCollisionEnvelope invalid;
    invalid.footprint.points = {
        {-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}};
    invalid.circumscribed_radius = 0.2;
    EXPECT_THROW(
        robot_geometry_core::StaticRobotGeometryProvider provider(invalid),
        std::invalid_argument);
}

TEST(RobotGeometryCore, PolygonFactoryRejectsSelfIntersection)
{
    robot_geometry_core::Footprint crossed;
    crossed.points = {
        {-0.2, -0.1}, {0.2, 0.1}, {-0.2, 0.1}, {0.2, -0.1}};
    EXPECT_THROW(
        robot_geometry_core::makePolygonCollisionEnvelope(
            crossed, "base_link", "test", 1U),
        std::invalid_argument);
}

TEST(RobotGeometryCore, ExactPolygonCollisionUsesGoalYaw)
{
    auto map = makeMap(0);
    map.data[2U * map.width + 3U] = 100;
    robot_geometry_core::Footprint rectangle;
    rectangle.points = {
        {-1.1, -0.2},
        {1.1, -0.2},
        {1.1, 0.2},
        {-1.1, 0.2}};
    const robot_geometry_core::FootprintCollisionConfig config{
        true, false, 51, rectangle};

    const auto horizontal = robot_geometry_core::checkFootprint(
        map, robot_geometry_core::Pose2D{2.5, 2.5, 0.0}, config);
    constexpr double kHalfPi = 1.57079632679489661923;
    const auto vertical = robot_geometry_core::checkFootprint(
        map, robot_geometry_core::Pose2D{2.5, 2.5, kHalfPi}, config);

    EXPECT_FALSE(horizontal.valid);
    EXPECT_EQ(horizontal.reason, "footprint_collision");
    EXPECT_TRUE(vertical.valid);
}

}  // 匿名命名空间
