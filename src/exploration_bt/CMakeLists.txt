cmake_minimum_required(VERSION 3.8)
project(exploration_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

set(exploration_bt_DEPS
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  std_srvs
)

function(configure_exploration_bt_target target_name)
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${target_name}
    ${exploration_bt_DEPS}
  )
endfunction()

# BT 插件由编排器动态加载。插件只负责组合逻辑，
# 通过 ROS 接口契约调用各项能力。
add_library(exploration_bt_nodes SHARED
  src/bt/exploration_bt_context.cpp
  src/bt/is_exploration_complete_condition.cpp
  src/bt/action/compute_frontier_candidates_action.cpp
  src/bt/action/mark_frontier_failed_action.cpp
  src/bt/action/navigate_to_frontier_action.cpp
  src/bt/action/prefetch_frontier_candidates_action.cpp
  src/bt/action/select_feasible_frontier_action.cpp
  src/bt/plugins/exploration_bt_plugins.cpp
)

target_compile_definitions(exploration_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
configure_exploration_bt_target(exploration_bt_nodes)

add_executable(exploration_bt_orchestrator_node
  src/bt/orchestrator/exploration_bt_orchestrator_main.cpp
  src/bt/orchestrator/exploration_bt_orchestrator_node.cpp
  src/bt/exploration_bt_context.cpp
)

configure_exploration_bt_target(exploration_bt_orchestrator_node)

install(TARGETS
  exploration_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  exploration_bt_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config behavior_trees doc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${exploration_bt_DEPS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_feasibility_batch
    test/test_feasibility_batch.cpp
  )
  target_include_directories(test_feasibility_batch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_feasibility_cache_key
    test/test_feasibility_cache_key.cpp
  )
  target_include_directories(test_feasibility_cache_key PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_feasibility_cache_key
    geometry_msgs
  )
endif()

ament_package()
