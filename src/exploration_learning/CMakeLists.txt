cmake_minimum_required(VERSION 3.8)
project(exploration_learning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interfaces REQUIRED)

set(exploration_learning_DEPS
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  robot_interfaces
)

add_library(${PROJECT_NAME}
  src/data_collection/episode_recorder.cpp
  src/reward/reward_calculator.cpp
  src/collector/json_utils.cpp
  src/collector/event_buffer.cpp
  src/collector/data_record_plugin_factory.cpp
  src/collector/dataset_writer.cpp
  src/plugins/frontier_decision_plugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${exploration_learning_DEPS}
)

add_executable(dataset_recorder_node
  src/dataset_recorder_main.cpp
  src/collector/dataset_recorder_node.cpp
)

target_link_libraries(dataset_recorder_node
  ${PROJECT_NAME}
)

ament_target_dependencies(dataset_recorder_node
  ${exploration_learning_DEPS}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  dataset_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch doc
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${exploration_learning_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_event_buffer
    test/test_event_buffer.cpp
  )
  target_link_libraries(test_event_buffer
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_event_buffer
    ${exploration_learning_DEPS}
  )

  ament_add_gtest(test_dataset_writer
    test/test_dataset_writer.cpp
  )
  target_link_libraries(test_dataset_writer
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_dataset_writer
    ${exploration_learning_DEPS}
  )

  ament_add_gtest(test_frontier_decision_plugin
    test/test_frontier_decision_plugin.cpp
  )
  target_link_libraries(test_frontier_decision_plugin
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_frontier_decision_plugin
    ${exploration_learning_DEPS}
  )
endif()

ament_package()
