cmake_minimum_required(VERSION 3.8)
project(frontier_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(frontier_explorer_node
  src/main.cpp
  src/frontier_explorer_node.cpp
)

ament_target_dependencies(frontier_explorer_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
)

install(TARGETS
  frontier_explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()