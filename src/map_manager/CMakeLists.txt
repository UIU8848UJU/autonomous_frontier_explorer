cmake_minimum_required(VERSION 3.8)
project(map_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(map_manager_node
  nodes/map_manager/main.cpp
  nodes/map_manager/map_manager_node.cpp
  core/map_storage/map_storage.cpp
)

target_include_directories(map_manager_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes/map_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/core/map_storage
)

ament_target_dependencies(map_manager_node
  rclcpp
  nav_msgs
  nav2_msgs
  robot_interfaces
)

install(TARGETS
  map_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
