#include "navigation_core/path_utils.hpp"

#include <cmath>
#include <cstddef>

namespace frontier_explorer
{
namespace navigation_core
{

double pathLengthM(const nav_msgs::msg::Path & path)
{
    if (path.poses.size() < 2U) {
        return 0.0;
    }

    double length = 0.0;
    for (std::size_t index = 1U; index < path.poses.size(); ++index) {
        const auto & previous = path.poses[index - 1U].pose.position;
        const auto & current = path.poses[index].pose.position;
        length += std::hypot(current.x - previous.x, current.y - previous.y);
    }
    return length;
}

}  // namespace navigation_core
}  // namespace frontier_explorer
