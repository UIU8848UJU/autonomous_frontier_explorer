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
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(frontier_explorer_DEPS
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
    ament_index_cpp
    behaviortree_cpp_v3
)

function(configure_frontier_explorer_target target_name)
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/frontier_explorer>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/frontier_explorer>
  )

  ament_target_dependencies(${target_name}
      ${frontier_explorer_DEPS}
  )
endfunction()

add_library(frontier_explorer_core
  src/core/costmap/costmap_adapter.cpp
  src/core/detector/frontier_detector.cpp
  src/core/selector/frontier_pruner.cpp
  src/core/selector/frontier_scorer.cpp
  src/core/selector/frontier_selector.cpp
  src/core/selector/score_components/clearance_score.cpp
  src/core/selector/score_components/cluster_size_score.cpp
  src/core/selector/score_components/distance_score.cpp
  src/core/selector/score_components/information_gain_score.cpp
  src/core/selector/score_components/retry_penalty_score.cpp
  src/core/selector/score_components/unknown_risk_penalty_score.cpp
  src/core/utils/frontier_selector_utils.cpp
  src/core/utils/map_utils.cpp
  src/core/visualization/frontier_marker_publisher.cpp
)

configure_frontier_explorer_target(frontier_explorer_core)

add_executable(frontier_explorer_node
  src/nodes/frontier_explorer_main.cpp
  src/nodes/frontier_explorer_node.cpp
)

target_link_libraries(frontier_explorer_node
  frontier_explorer_core
)

configure_frontier_explorer_target(frontier_explorer_node)

add_executable(exploration_bt_orchestrator_node
  src/nodes/bt/exploration_bt_context.cpp
  src/nodes/bt/is_exploration_complete_condition.cpp
  src/nodes/bt/orchestrator/exploration_bt_orchestrator_main.cpp
  src/nodes/bt/orchestrator/exploration_bt_orchestrator_node.cpp
  src/nodes/bt/action/compute_next_frontier_goal_action.cpp
  src/nodes/bt/action/mark_frontier_failed_action.cpp
  src/nodes/bt/action/navigate_to_frontier_action.cpp
)

configure_frontier_explorer_target(exploration_bt_orchestrator_node)

install(TARGETS
    frontier_explorer_core
    frontier_explorer_node
    exploration_bt_orchestrator_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz doc behavior_trees
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_include_directories(include include/frontier_explorer)
ament_export_dependencies(${frontier_explorer_DEPS})

ament_package()
