#pragma once

#include "nav_msgs/msg/path.hpp"

namespace frontier_explorer
{
namespace navigation_core
{

/// @brief: ???????
/// @param path ?????
/// @return: ??????? m??? 2 ?????? 0
double pathLengthM(const nav_msgs::msg::Path & path);

}  // namespace navigation_core
}  // namespace frontier_explorer
