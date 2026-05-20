cmake_minimum_required(VERSION 3.8)
project(frontier_explorer_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(util_package REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(frontier_explorer_core REQUIRED)

set(frontier_explorer_nodes_DEPS
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  nav2_costmap_2d
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  robot_interfaces
  tf2_geometry_msgs
  tf2
  tf2_ros
  util_package
  ament_index_cpp
  behaviortree_cpp_v3
  frontier_explorer_core
)

function(configure_frontier_explorer_nodes_target target_name)
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${target_name}
    ${frontier_explorer_nodes_DEPS}
  )
endfunction()

add_library(frontier_explorer_nodes_common
  src/frontier_explorer_node.cpp
  src/navigation_node.cpp
  src/nav2_planner_reachability_checker.cpp
  src/frontier_marker_publisher.cpp
)

configure_frontier_explorer_nodes_target(frontier_explorer_nodes_common)

add_executable(frontier_explorer_node
  src/frontier_explorer_main.cpp
)

target_link_libraries(frontier_explorer_node
  frontier_explorer_nodes_common
)

configure_frontier_explorer_nodes_target(frontier_explorer_node)

add_executable(navigation_node
  src/navigation_main.cpp
)

target_link_libraries(navigation_node
  frontier_explorer_nodes_common
)

configure_frontier_explorer_nodes_target(navigation_node)

add_library(frontier_explorer_bt_nodes SHARED
  src/bt/exploration_bt_context.cpp
  src/bt/is_exploration_complete_condition.cpp
  src/bt/action/compute_frontier_candidates_action.cpp
  src/bt/action/compute_next_frontier_goal_action.cpp
  src/bt/action/mark_frontier_failed_action.cpp
  src/bt/action/navigate_to_frontier_action.cpp
  src/bt/action/select_feasible_frontier_action.cpp
  src/bt/action/select_reachable_frontier_action.cpp
  src/bt/plugins/exploration_bt_plugins.cpp
)

target_compile_definitions(frontier_explorer_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
configure_frontier_explorer_nodes_target(frontier_explorer_bt_nodes)

add_executable(exploration_bt_orchestrator_node
  src/bt/orchestrator/exploration_bt_orchestrator_main.cpp
  src/bt/orchestrator/exploration_bt_orchestrator_node.cpp
  src/bt/exploration_bt_context.cpp
)

configure_frontier_explorer_nodes_target(exploration_bt_orchestrator_node)

install(TARGETS
  frontier_explorer_nodes_common
  frontier_explorer_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  frontier_explorer_node
  navigation_node
  exploration_bt_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz doc behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${frontier_explorer_nodes_DEPS})

ament_package()
