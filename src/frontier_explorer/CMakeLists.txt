cmake_minimum_required(VERSION 3.8)
project(frontier_explorer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(util_package REQUIRED)

include_directories(include)

add_executable(frontier_explorer_node
  nodes/frontier_explorer/main.cpp
  nodes/frontier_explorer/frontier_explorer_node.cpp
  utils/map/map_utils.cpp
  visualization/frontier_marker_publisher.cpp
  core/costmap_adapter/costmap_adapter.cpp
  core/frontier_detector/frontier_detector.cpp
  core/frontier_selector/frontier_pruner.cpp
  core/frontier_selector/frontier_scorer.cpp
  core/frontier_selector/score_components/distance_score.cpp
  core/frontier_selector/score_components/cluster_size_score.cpp
  core/frontier_selector/score_components/clearance_score.cpp
  core/frontier_selector/score_components/retry_penalty_score.cpp
  core/frontier_selector/score_components/unknown_risk_penalty_score.cpp
  core/frontier_selector/score_components/information_gain_score.cpp
  core/frontier_selector/frontier_selector.cpp
)

target_include_directories(frontier_explorer_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/frontier_explorer
  ${CMAKE_CURRENT_SOURCE_DIR}/visualization
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  ${CMAKE_CURRENT_SOURCE_DIR}/core/costmap_adapter
  ${CMAKE_CURRENT_SOURCE_DIR}/core/frontier_detector
  ${CMAKE_CURRENT_SOURCE_DIR}/core/frontier_selector
  ${CMAKE_CURRENT_SOURCE_DIR}/core/frontier_selector/score_components
  ${CMAKE_CURRENT_SOURCE_DIR}/utils
  ${CMAKE_CURRENT_SOURCE_DIR}/utils/map
  ${CMAKE_CURRENT_SOURCE_DIR}/utils/frontier
)


ament_target_dependencies(frontier_explorer_node
    rclcpp
    rclcpp_action
    nav_msgs
    nav2_msgs
    nav2_costmap_2d
    geometry_msgs
    visualization_msgs
    std_srvs
    robot_interfaces
    tf2_geometry_msgs
    tf2
    tf2_ros
    util_package
)

install(TARGETS
    frontier_explorer_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
