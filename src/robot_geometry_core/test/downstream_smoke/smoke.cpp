#include <cmath>

#include "robot_geometry_core/robot_geometry_provider.hpp"

int main()
{
    robot_geometry_core::Footprint footprint;
    footprint.points = {
        {-0.2, -0.1}, {0.3, -0.1}, {0.3, 0.1}, {-0.2, 0.1}};
    const robot_geometry_core::StaticRobotGeometryProvider provider(
        robot_geometry_core::makePolygonCollisionEnvelope(
            footprint, "base_link", "downstream_smoke", 1U));
    const auto envelope = provider.collisionEnvelope();
    return envelope.footprint.points.size() == 4U &&
           envelope.circumscribed_radius >= std::sqrt(0.10) ? 0 : 1;
}
