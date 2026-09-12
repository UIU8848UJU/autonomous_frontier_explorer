cmake_minimum_required(VERSION 3.16)
project(exploration_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(grid_map_core CONFIG REQUIRED)

# 行为层只定义通用探索协议，不把 Frontier、ROS 或 Nav2 引入接口。
add_library(exploration_core INTERFACE)
add_library(exploration_core::exploration_core ALIAS exploration_core)
target_include_directories(exploration_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(exploration_core INTERFACE grid_map_core::grid_map_core)

install(TARGETS exploration_core
  EXPORT exploration_coreTargets
)
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT exploration_coreTargets
  FILE exploration_coreTargets.cmake
  NAMESPACE exploration_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/exploration_core
)

configure_package_config_file(
  cmake/exploration_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/exploration_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/exploration_core
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/exploration_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/exploration_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/exploration_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/exploration_core
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(test_exploration_core test/test_exploration_core.cpp)
  target_link_libraries(test_exploration_core PRIVATE exploration_core GTest::gtest_main)
  gtest_discover_tests(test_exploration_core)

  add_test(
    NAME exploration_core_no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DCORE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/no_ros_dependencies.cmake
  )
endif()
