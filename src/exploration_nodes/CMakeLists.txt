cmake_minimum_required(VERSION 3.8)
project(exploration_nodes)

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
find_package(frontier_strategy_ros REQUIRED)
find_package(navigation_core REQUIRED)
find_package(robot_geometry_core REQUIRED)
find_package(grid_map_ros REQUIRED)

set(frontier_strategy_node_DEPS
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  robot_interfaces
  tf2
  tf2_ros
  grid_map_ros
  frontier_strategy_ros
)

set(navigation_node_DEPS
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  nav2_costmap_2d
  geometry_msgs
  std_msgs
  robot_interfaces
  grid_map_ros
)

function(configure_exploration_nodes_target target_name)
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${target_name}
    ${ARGN}
  )
endfunction()

add_library(robot_geometry_parameter_adapter_lib
  src/adapters/robot_geometry_parameter_adapter.cpp
)
configure_exploration_nodes_target(
  robot_geometry_parameter_adapter_lib
  rclcpp
)
target_link_libraries(robot_geometry_parameter_adapter_lib
  robot_geometry_core::robot_geometry_core
)

add_library(frontier_strategy_node_lib
  src/frontier_strategy_node.cpp
  src/nav2_planner_reachability_checker.cpp
  src/frontier_marker_publisher.cpp
)

configure_exploration_nodes_target(
  frontier_strategy_node_lib
  ${frontier_strategy_node_DEPS}
)
target_link_libraries(frontier_strategy_node_lib
  robot_geometry_parameter_adapter_lib
  robot_geometry_core::robot_geometry_core
)

add_executable(frontier_strategy_node
  src/frontier_strategy_main.cpp
)

target_link_libraries(frontier_strategy_node
  frontier_strategy_node_lib
)

configure_exploration_nodes_target(
  frontier_strategy_node
  ${frontier_strategy_node_DEPS}
)

add_library(navigation_node_lib
  src/navigation_node.cpp
  src/adapters/navigation_converters.cpp
)

configure_exploration_nodes_target(
  navigation_node_lib
  ${navigation_node_DEPS}
)
target_link_libraries(navigation_node_lib
  robot_geometry_parameter_adapter_lib
  navigation_core::navigation_core
  robot_geometry_core::robot_geometry_core
  grid_map_ros::grid_map_ros
)

add_executable(navigation_node
  src/navigation_main.cpp
)

target_link_libraries(navigation_node
  navigation_node_lib
)

configure_exploration_nodes_target(
  navigation_node
  ${navigation_node_DEPS}
)

install(TARGETS
  robot_geometry_parameter_adapter_lib
  frontier_strategy_node_lib
  navigation_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  frontier_strategy_node
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config doc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  ${frontier_strategy_node_DEPS}
  ${navigation_node_DEPS}
  robot_geometry_core
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_navigation_converters
    test/test_navigation_converters.cpp
  )
  target_link_libraries(test_navigation_converters
    navigation_node_lib
  )
  configure_exploration_nodes_target(
    test_navigation_converters
    nav2_costmap_2d
  )

  ament_add_gtest(test_robot_geometry_parameter_adapter
    test/test_robot_geometry_parameter_adapter.cpp
  )
  target_link_libraries(test_robot_geometry_parameter_adapter
    robot_geometry_parameter_adapter_lib
  )
  configure_exploration_nodes_target(
    test_robot_geometry_parameter_adapter
    rclcpp
  )
endif()

ament_package()
