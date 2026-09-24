#include "exploration_nodes/adapters/robot_geometry_parameter_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace exploration
{
namespace adapters
{
namespace
{
constexpr char kShapeParameter[] = "robot_geometry.shape";
constexpr char kFootprintParameter[] = "robot_geometry.footprint";
constexpr char kFrameParameter[] = "robot_geometry.frame_id";
constexpr char kSourceParameter[] = "robot_geometry.source";
constexpr char kRevisionParameter[] = "robot_geometry.revision";

std::string normalizedShape(std::string shape)
{
    std::transform(
        shape.begin(), shape.end(), shape.begin(),
        [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return shape;
}

}  // 匿名命名空间

void declareRobotGeometryParameters(
    rclcpp::Node & node,
    const LegacyRobotGeometryParameterNames & legacy_names,
    const RobotGeometryParameters & defaults)
{
    if (legacy_names.radius.empty() || legacy_names.padding.empty()) {
        throw std::invalid_argument("legacy robot geometry parameter names must not be empty");
    }
    node.declare_parameter<std::string>(kShapeParameter, defaults.shape);
    node.declare_parameter<std::vector<double>>(kFootprintParameter, defaults.footprint_xy);
    node.declare_parameter<std::string>(kFrameParameter, defaults.frame_id);
    node.declare_parameter<std::string>(kSourceParameter, defaults.source);
    node.declare_parameter<int64_t>(
        kRevisionParameter, static_cast<int64_t>(defaults.revision));
    // 保留旧参数名，已有 YAML 不需要为了本次重构立即迁移。
    node.declare_parameter<double>(legacy_names.radius, defaults.radius);
    node.declare_parameter<double>(legacy_names.padding, defaults.padding);
}

RobotGeometryParameters loadRobotGeometryParameters(
    const rclcpp::Node & node,
    const LegacyRobotGeometryParameterNames & legacy_names)
{
    RobotGeometryParameters params;
    params.shape = node.get_parameter(kShapeParameter).as_string();
    params.footprint_xy = node.get_parameter(kFootprintParameter).as_double_array();
    params.frame_id = node.get_parameter(kFrameParameter).as_string();
    params.source = node.get_parameter(kSourceParameter).as_string();
    const auto revision = node.get_parameter(kRevisionParameter).as_int();
    if (revision < 0) {
        throw std::invalid_argument("robot geometry revision must not be negative");
    }
    params.revision = static_cast<std::uint64_t>(revision);
    params.radius = node.get_parameter(legacy_names.radius).as_double();
    params.padding = node.get_parameter(legacy_names.padding).as_double();
    return params;
}

std::shared_ptr<const robot_geometry_core::IRobotGeometryProvider>
makeRobotGeometryProvider(const RobotGeometryParameters & params)
{
    const auto shape = normalizedShape(params.shape);
    robot_geometry_core::RobotCollisionEnvelope envelope;
    if (shape == "circle") {
        if (!params.footprint_xy.empty()) {
            throw std::invalid_argument(
                "circle robot geometry must not also define polygon footprint");
        }
        envelope = robot_geometry_core::makeCircularCollisionEnvelope(
            params.radius,
            params.padding,
            params.frame_id,
            params.source,
            params.revision);
    } else if (shape == "polygon") {
        if (params.padding != 0.0) {
            throw std::invalid_argument(
                "polygon footprint must already contain safety padding");
        }
        if (params.footprint_xy.size() < 6U || params.footprint_xy.size() % 2U != 0U) {
            throw std::invalid_argument(
                "polygon footprint must contain at least three x/y pairs");
        }
        robot_geometry_core::Footprint footprint;
        footprint.points.reserve(params.footprint_xy.size() / 2U);
        for (std::size_t index = 0U; index < params.footprint_xy.size(); index += 2U) {
            footprint.points.push_back(
                {params.footprint_xy[index], params.footprint_xy[index + 1U]});
        }
        envelope = robot_geometry_core::makePolygonCollisionEnvelope(
            std::move(footprint),
            params.frame_id,
            params.source,
            params.revision);
    } else {
        throw std::invalid_argument("robot_geometry.shape must be circle or polygon");
    }
    return std::make_shared<robot_geometry_core::StaticRobotGeometryProvider>(
        std::move(envelope));
}

}  // 命名空间 adapters
}  // 命名空间 exploration
