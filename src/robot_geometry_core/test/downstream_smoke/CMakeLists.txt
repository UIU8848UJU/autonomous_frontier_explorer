cmake_minimum_required(VERSION 3.16)
project(robot_geometry_core_downstream_smoke LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(robot_geometry_core CONFIG REQUIRED)

add_executable(robot_geometry_core_downstream_smoke smoke.cpp)
target_link_libraries(robot_geometry_core_downstream_smoke
  PRIVATE robot_geometry_core::robot_geometry_core
)
