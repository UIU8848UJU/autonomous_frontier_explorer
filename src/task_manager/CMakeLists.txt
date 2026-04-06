cmake_minimum_required(VERSION 3.8)
project(task_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(util_package REQUIRED)


add_executable(task_manager_node
  nodes/task_manager/main.cpp
  nodes/task_manager/task_manager_node.cpp
  core/task_flow/task_flow.cpp
)

ament_target_dependencies(task_manager_node
  rclcpp
  robot_interfaces
  std_srvs
  util_package
)

target_include_directories(task_manager_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/src/util_package/include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  task_manager_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
