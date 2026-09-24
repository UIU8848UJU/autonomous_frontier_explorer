#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "robot_geometry_core/footprint_collision_checker.hpp"

namespace robot_geometry_core
{

/// 机器人二维碰撞包络的不可变快照。
struct RobotCollisionEnvelope
{
    Footprint footprint;
    double circumscribed_radius{0.0};
    double safety_padding{0.0};
    std::string frame_id{"base_link"};
    std::string source{"configured"};
    std::uint64_t revision{0U};
};

namespace detail
{

inline double cross(const Point2D & a, const Point2D & b, const Point2D & c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool pointOnSegment(
    const Point2D & point,
    const Point2D & start,
    const Point2D & end,
    double epsilon)
{
    return std::abs(cross(start, end, point)) <= epsilon &&
           point.x >= std::min(start.x, end.x) - epsilon &&
           point.x <= std::max(start.x, end.x) + epsilon &&
           point.y >= std::min(start.y, end.y) - epsilon &&
           point.y <= std::max(start.y, end.y) + epsilon;
}

inline bool segmentsIntersect(
    const Point2D & first_start,
    const Point2D & first_end,
    const Point2D & second_start,
    const Point2D & second_end,
    double epsilon)
{
    const double first_side_a = cross(first_start, first_end, second_start);
    const double first_side_b = cross(first_start, first_end, second_end);
    const double second_side_a = cross(second_start, second_end, first_start);
    const double second_side_b = cross(second_start, second_end, first_end);
    if (((first_side_a > epsilon && first_side_b < -epsilon) ||
        (first_side_a < -epsilon && first_side_b > epsilon)) &&
        ((second_side_a > epsilon && second_side_b < -epsilon) ||
        (second_side_a < -epsilon && second_side_b > epsilon)))
    {
        return true;
    }
    return pointOnSegment(second_start, first_start, first_end, epsilon) ||
           pointOnSegment(second_end, first_start, first_end, epsilon) ||
           pointOnSegment(first_start, second_start, second_end, epsilon) ||
           pointOnSegment(first_end, second_start, second_end, epsilon);
}

}  // 命名空间 detail

/// 校验一个碰撞包络是否可安全交给规划与过滤模块。
inline void validateCollisionEnvelope(const RobotCollisionEnvelope & envelope)
{
    if (envelope.frame_id.empty() || envelope.source.empty()) {
        throw std::invalid_argument("robot geometry frame_id/source must not be empty");
    }
    if (!std::isfinite(envelope.circumscribed_radius) ||
        envelope.circumscribed_radius <= 0.0 ||
        !std::isfinite(envelope.safety_padding) ||
        envelope.safety_padding < 0.0)
    {
        throw std::invalid_argument("robot geometry radius/padding is invalid");
    }
    if (envelope.footprint.points.size() < 3U) {
        throw std::invalid_argument("robot geometry footprint requires at least three points");
    }

    constexpr double kEpsilon = 1e-12;
    double twice_area = 0.0;
    double max_vertex_radius = 0.0;
    const auto point_count = envelope.footprint.points.size();
    for (std::size_t index = 0U; index < point_count; ++index) {
        const auto & current = envelope.footprint.points[index];
        const auto & next = envelope.footprint.points[(index + 1U) % point_count];
        if (!std::isfinite(current.x) || !std::isfinite(current.y)) {
            throw std::invalid_argument("robot geometry footprint contains non-finite point");
        }
        twice_area += current.x * next.y - next.x * current.y;
        max_vertex_radius = std::max(max_vertex_radius, std::hypot(current.x, current.y));
        for (std::size_t other = index + 1U; other < point_count; ++other) {
            const auto & candidate = envelope.footprint.points[other];
            const double dx = current.x - candidate.x;
            const double dy = current.y - candidate.y;
            if (dx * dx + dy * dy <= kEpsilon) {
                throw std::invalid_argument("robot geometry footprint contains duplicate point");
            }
        }
    }
    if (std::abs(twice_area) <= kEpsilon) {
        throw std::invalid_argument("robot geometry footprint area must be non-zero");
    }
    if (max_vertex_radius > envelope.circumscribed_radius + kEpsilon) {
        throw std::invalid_argument("robot geometry circumscribed radius is too small");
    }

    for (std::size_t first = 0U; first < point_count; ++first) {
        const std::size_t first_next = (first + 1U) % point_count;
        for (std::size_t second = first + 1U; second < point_count; ++second) {
            const std::size_t second_next = (second + 1U) % point_count;
            if (first == second || first_next == second || second_next == first) {
                continue;
            }
            if (detail::segmentsIntersect(
                envelope.footprint.points[first],
                envelope.footprint.points[first_next],
                envelope.footprint.points[second],
                envelope.footprint.points[second_next],
                kEpsilon))
            {
                throw std::invalid_argument("robot geometry footprint must not self-intersect");
            }
        }
    }
}

/// 机器人碰撞几何来源接口；Core 不关心数据来自仿真模型、配置文件还是实机标定。
class IRobotGeometryProvider
{
public:
    virtual ~IRobotGeometryProvider() = default;

    /// 获取一个自洽的几何快照，调用方可在一次决策期间安全持有该副本。
    virtual RobotCollisionEnvelope collisionEnvelope() const = 0;
};

/// 固定几何 Provider，适用于配置文件和离线标定产物。
class StaticRobotGeometryProvider final : public IRobotGeometryProvider
{
public:
    explicit StaticRobotGeometryProvider(RobotCollisionEnvelope envelope)
    : envelope_(std::move(envelope))
    {
        robot_geometry_core::validateCollisionEnvelope(envelope_);
    }

    RobotCollisionEnvelope collisionEnvelope() const override
    {
        return envelope_;
    }

private:
    RobotCollisionEnvelope envelope_;
};

inline RobotCollisionEnvelope makeCircularCollisionEnvelope(
    double robot_radius,
    double safety_padding,
    std::string frame_id,
    std::string source,
    std::uint64_t revision)
{
    if (!std::isfinite(robot_radius) || robot_radius <= 0.0 ||
        !std::isfinite(safety_padding) || safety_padding < 0.0)
    {
        throw std::invalid_argument("circular robot geometry radius/padding is invalid");
    }
    RobotCollisionEnvelope envelope;
    envelope.footprint = makeConservativeCircularFootprint(robot_radius + safety_padding);
    for (const auto & point : envelope.footprint.points) {
        envelope.circumscribed_radius = std::max(
            envelope.circumscribed_radius, std::hypot(point.x, point.y));
    }
    envelope.safety_padding = safety_padding;
    envelope.frame_id = std::move(frame_id);
    envelope.source = std::move(source);
    envelope.revision = revision;
    validateCollisionEnvelope(envelope);
    return envelope;
}

inline RobotCollisionEnvelope makePolygonCollisionEnvelope(
    Footprint footprint,
    std::string frame_id,
    std::string source,
    std::uint64_t revision)
{
    RobotCollisionEnvelope envelope;
    envelope.footprint = std::move(footprint);
    for (const auto & point : envelope.footprint.points) {
        envelope.circumscribed_radius = std::max(
            envelope.circumscribed_radius, std::hypot(point.x, point.y));
    }
    envelope.frame_id = std::move(frame_id);
    envelope.source = std::move(source);
    envelope.revision = revision;
    validateCollisionEnvelope(envelope);
    return envelope;
}

}  // 命名空间 robot_geometry_core
