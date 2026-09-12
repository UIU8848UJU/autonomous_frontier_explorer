cmake_minimum_required(VERSION 3.16)
project(navigation_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(grid_map_core CONFIG REQUIRED)
find_package(robot_geometry_core CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/path_safety_checker.cpp
  src/footprint_goal_validator.cpp
  src/single_goal_gate.cpp
  src/path_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  grid_map_core::grid_map_core
  robot_geometry_core::robot_geometry_core
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
ament_export_dependencies(grid_map_core robot_geometry_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_core test/test_navigation_core.cpp)
  target_link_libraries(test_navigation_core ${PROJECT_NAME})
  target_link_libraries(test_navigation_core
    grid_map_core::grid_map_core
    robot_geometry_core::robot_geometry_core)
endif()

ament_package()
