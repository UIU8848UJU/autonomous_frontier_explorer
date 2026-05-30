cmake_minimum_required(VERSION 3.8)
project(chassis_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(chassis_bridge_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)

add_library(${PROJECT_NAME}_core
  src/chassis_protocol.cpp
  src/odom_estimator.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  ${chassis_bridge_DEPS}
)

add_executable(fake_chassis_node
  src/fake_chassis_node.cpp
)
target_link_libraries(fake_chassis_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(fake_chassis_node
  ${chassis_bridge_DEPS}
)

add_executable(chassis_bridge_node
  src/chassis_bridge_node.cpp
)
ament_target_dependencies(chassis_bridge_node
  rclcpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  fake_chassis_node
  chassis_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${chassis_bridge_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_chassis_protocol
    test/test_chassis_protocol.cpp
  )
  target_link_libraries(test_chassis_protocol
    ${PROJECT_NAME}_core
  )
  ament_target_dependencies(test_chassis_protocol
    ${chassis_bridge_DEPS}
  )

  ament_add_gtest(test_fake_chassis_math
    test/test_fake_chassis_math.cpp
  )
  target_link_libraries(test_fake_chassis_math
    ${PROJECT_NAME}_core
  )
  ament_target_dependencies(test_fake_chassis_math
    ${chassis_bridge_DEPS}
  )
endif()

ament_package()
