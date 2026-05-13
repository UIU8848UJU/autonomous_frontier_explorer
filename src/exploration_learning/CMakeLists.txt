cmake_minimum_required(VERSION 3.8)
project(exploration_learning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

set(exploration_learning_DEPS
  rclcpp
  robot_interfaces
)

add_library(${PROJECT_NAME}
  src/episode_recorder.cpp
  src/reward_calculator.cpp
  src/dataset_writer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${exploration_learning_DEPS}
)

add_executable(rl_data_collector_node
  src/rl_data_collector_main.cpp
  src/rl_data_collector_node.cpp
)

target_link_libraries(rl_data_collector_node
  ${PROJECT_NAME}
)

ament_target_dependencies(rl_data_collector_node
  ${exploration_learning_DEPS}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rl_data_collector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${exploration_learning_DEPS})

ament_package()
