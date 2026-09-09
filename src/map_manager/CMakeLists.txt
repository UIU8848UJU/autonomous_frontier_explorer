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

add_library(map_manager_core SHARED
  core/map_manager_core.cpp
)

target_include_directories(map_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(map_lifecycle_node
  nodes/map_lifecycle/main.cpp
  nodes/map_lifecycle/map_lifecycle_node.cpp
  nodes/map_lifecycle/adapters/nav2_map_saver.cpp
  nodes/map_lifecycle/adapters/ros_message_converter.cpp
)

target_include_directories(map_lifecycle_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes/map_lifecycle
)

target_link_libraries(map_lifecycle_node
  map_manager_core
)

ament_target_dependencies(map_lifecycle_node
  rclcpp
  nav_msgs
  nav2_msgs
  robot_interfaces
)

install(TARGETS
  map_manager_core
  EXPORT export_map_manager_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  map_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_map_manager_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_map_manager_core
    test/test_map_manager_core.cpp
  )
  target_link_libraries(test_map_manager_core
    map_manager_core
  )

  ament_add_gtest(test_map_lifecycle_node
    test/test_map_lifecycle_node.cpp
    nodes/map_lifecycle/map_lifecycle_node.cpp
    nodes/map_lifecycle/adapters/nav2_map_saver.cpp
    nodes/map_lifecycle/adapters/ros_message_converter.cpp
  )
  target_include_directories(test_map_lifecycle_node
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/nodes/map_lifecycle
  )
  target_link_libraries(test_map_lifecycle_node
    map_manager_core
  )
  ament_target_dependencies(test_map_lifecycle_node
    rclcpp
    nav_msgs
    nav2_msgs
    robot_interfaces
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
