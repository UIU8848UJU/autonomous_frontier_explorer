#include <cmath>
#include <stdexcept>

#include "exploration_nodes/adapters/robot_geometry_parameter_adapter.hpp"
#include "gtest/gtest.h"

namespace
{

TEST(RobotGeometryParameterAdapter, BuildsCircularProvider)
{
    exploration::adapters::RobotGeometryParameters params;
    params.shape = "circle";
    params.radius = 0.10;
    params.padding = 0.02;
    params.frame_id = "base_link";
    params.source = "gazebo_model";
    params.revision = 4U;

    const auto provider = exploration::adapters::makeRobotGeometryProvider(params);
    const auto envelope = provider->collisionEnvelope();
    EXPECT_GE(envelope.circumscribed_radius, 0.12);
    EXPECT_LT(envelope.circumscribed_radius, 0.121);
    EXPECT_EQ(envelope.source, "gazebo_model");
    EXPECT_EQ(envelope.revision, 4U);
}

TEST(RobotGeometryParameterAdapter, BuildsCalibratedPolygonProvider)
{
    exploration::adapters::RobotGeometryParameters params;
    params.shape = "polygon";
    params.footprint_xy = {
        -0.20, -0.10,
        0.30, -0.10,
        0.30, 0.10,
        -0.20, 0.10};
    params.frame_id = "base_footprint";
    params.source = "wall_calibration";

    const auto provider = exploration::adapters::makeRobotGeometryProvider(params);
    const auto envelope = provider->collisionEnvelope();
    EXPECT_EQ(envelope.footprint.points.size(), 4U);
    EXPECT_NEAR(envelope.circumscribed_radius, std::sqrt(0.10), 1e-9);
}

TEST(RobotGeometryParameterAdapter, RejectsMalformedPolygon)
{
    exploration::adapters::RobotGeometryParameters params;
    params.shape = "polygon";
    params.footprint_xy = {0.0, 0.0, 1.0};
    EXPECT_THROW(
        exploration::adapters::makeRobotGeometryProvider(params),
        std::invalid_argument);
}

TEST(RobotGeometryParameterAdapter, RejectsAdditionalPaddingForCalibratedPolygon)
{
    exploration::adapters::RobotGeometryParameters params;
    params.shape = "polygon";
    params.padding = 0.01;
    params.footprint_xy = {
        -0.20, -0.10,
        0.30, -0.10,
        0.30, 0.10,
        -0.20, 0.10};
    EXPECT_THROW(
        exploration::adapters::makeRobotGeometryProvider(params),
        std::invalid_argument);
}

TEST(RobotGeometryParameterAdapter, LoadsLegacyRadiusAliasesFromRosParameters)
{
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    options.parameter_overrides({
        rclcpp::Parameter("legacy.robot_radius", 0.15),
        rclcpp::Parameter("legacy.footprint_padding", 0.03),
        rclcpp::Parameter("robot_geometry.source", "legacy_yaml")});
    auto node = std::make_shared<rclcpp::Node>("geometry_parameter_test", options);
    const exploration::adapters::LegacyRobotGeometryParameterNames names{
        "legacy.robot_radius", "legacy.footprint_padding"};
    exploration::adapters::declareRobotGeometryParameters(*node, names);

    const auto params = exploration::adapters::loadRobotGeometryParameters(*node, names);
    EXPECT_DOUBLE_EQ(params.radius, 0.15);
    EXPECT_DOUBLE_EQ(params.padding, 0.03);
    EXPECT_EQ(params.source, "legacy_yaml");
    context->shutdown("test complete");
}

}  // 匿名命名空间
