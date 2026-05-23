cmake_minimum_required(VERSION 3.8)
project(frontier_explorer_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(frontier_explorer_core_DEPS
  rclcpp
  nav_msgs
  nav2_costmap_2d
  geometry_msgs
  nav2_msgs
  tf2
)

add_library(${PROJECT_NAME}
  src/frontier_goal_provider.cpp
  src/costmap/costmap_adapter.cpp
  src/detector/frontier_detector.cpp
  src/selector/frontier_pruner.cpp
  src/selector/frontier_scorer.cpp
  src/selector/frontier_selector.cpp
  src/selector/score_components/clearance_score.cpp
  src/selector/score_components/cluster_size_score.cpp
  src/selector/score_components/distance_score.cpp
  src/selector/score_components/information_gain_score.cpp
  src/selector/score_components/retry_penalty_score.cpp
  src/selector/score_components/unknown_risk_penalty_score.cpp
  src/utils/frontier_selector_utils.cpp
  src/utils/map_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${frontier_explorer_core_DEPS}
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
ament_export_dependencies(${frontier_explorer_core_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_algorithms
    test/test_frontier_algorithms.cpp
  )
  target_link_libraries(test_frontier_algorithms
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_frontier_algorithms
    ${frontier_explorer_core_DEPS}
  )
endif()

ament_package()
