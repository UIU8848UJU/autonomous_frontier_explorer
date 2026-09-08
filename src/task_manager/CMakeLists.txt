cmake_minimum_required(VERSION 3.8)
project(task_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(util_package REQUIRED)

add_library(task_manager_core SHARED
  core/task_flow/task_flow.cpp
)

target_include_directories(task_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
  $<INSTALL_INTERFACE:include>
)

add_executable(task_manager_node
  nodes/task_manager/main.cpp
  nodes/task_manager/task_manager_node.cpp
  nodes/task_manager/adapters/ros_message_converter.cpp
)

target_link_libraries(task_manager_node
  task_manager_core
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
    ${CMAKE_SOURCE_DIR}/src/util_package/include
)

install(TARGETS
  task_manager_core
  EXPORT export_task_manager_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY core/
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
)

install(TARGETS
  task_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_task_manager_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_manager_core
    test/test_task_manager_core.cpp
  )
  target_link_libraries(test_task_manager_core
    task_manager_core
  )

  ament_add_gtest(test_task_manager_ros_converter
    test/test_ros_message_converter.cpp
    nodes/task_manager/adapters/ros_message_converter.cpp
  )
  target_include_directories(test_task_manager_ros_converter
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/nodes/task_manager
  )
  target_link_libraries(test_task_manager_ros_converter
    task_manager_core
  )
  ament_target_dependencies(test_task_manager_ros_converter
    robot_interfaces
  )

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
