cmake_minimum_required(VERSION 3.8)
project(frontier_strategy_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(frontier_strategy_core CONFIG REQUIRED)
find_package(exploration_core CONFIG REQUIRED)
find_package(robot_geometry_core CONFIG REQUIRED)
find_package(grid_map_ros CONFIG REQUIRED)
find_package(robot_interfaces REQUIRED)

set(frontier_strategy_ros_DEPS
  rclcpp
  nav_msgs
  nav2_costmap_2d
  geometry_msgs
  exploration_core
  robot_geometry_core
  grid_map_ros
  robot_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/adaptation/frontier_map_parameter_adapter.cpp
  src/frontier_goal_provider.cpp
  src/utils/map_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  frontier_strategy_core::frontier_strategy_core
  robot_geometry_core::robot_geometry_core
  grid_map_ros::grid_map_ros
)
ament_target_dependencies(${PROJECT_NAME}
  ${frontier_strategy_ros_DEPS}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${frontier_strategy_ros_DEPS} frontier_strategy_core exploration_core robot_geometry_core grid_map_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_ros
    test/test_frontier_ros.cpp
  )
  target_link_libraries(test_frontier_ros
    ${PROJECT_NAME}
    grid_map_ros::grid_map_ros
  )
  ament_target_dependencies(test_frontier_ros
    ${frontier_strategy_ros_DEPS}
  )
endif()

ament_package()
