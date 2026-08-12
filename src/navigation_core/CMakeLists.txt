cmake_minimum_required(VERSION 3.8)
project(navigation_core)

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
find_package(frontier_explorer_core REQUIRED)

set(navigation_core_DEPS
  rclcpp
  nav_msgs
  nav2_costmap_2d
  geometry_msgs
  frontier_explorer_core
)

add_library(${PROJECT_NAME}
  src/path_safety_checker.cpp
  src/footprint_goal_validator.cpp
  src/single_goal_gate.cpp
  src/path_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${navigation_core_DEPS}
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
ament_export_dependencies(${navigation_core_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_navigation_core
    test/test_navigation_core.cpp
  )
  target_link_libraries(test_navigation_core
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_navigation_core
    ${navigation_core_DEPS}
  )
endif()

ament_package()
