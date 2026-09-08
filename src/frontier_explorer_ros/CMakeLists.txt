cmake_minimum_required(VERSION 3.8)
project(frontier_explorer_ros)

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
find_package(frontier_explorer_core CONFIG REQUIRED)

set(frontier_explorer_ros_DEPS
  rclcpp
  nav_msgs
  nav2_costmap_2d
  geometry_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/frontier_goal_provider.cpp
  src/costmap/costmap_adapter.cpp
  src/selector/frontier_selector.cpp
  src/utils/map_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  frontier_explorer_core::frontier_explorer_core
)
ament_target_dependencies(${PROJECT_NAME}
  ${frontier_explorer_ros_DEPS}
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
ament_export_dependencies(${frontier_explorer_ros_DEPS} frontier_explorer_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_ros
    test/test_frontier_ros.cpp
  )
  target_link_libraries(test_frontier_ros
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_frontier_ros
    ${frontier_explorer_ros_DEPS}
  )
endif()

ament_package()
