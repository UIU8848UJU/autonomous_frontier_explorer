cmake_minimum_required(VERSION 3.16)
project(grid_map_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# 地图数据结构只依赖 C++ 标准库，供 Frontier、探索行为和后续导航核心复用。
add_library(grid_map_core INTERFACE)
add_library(grid_map_core::grid_map_core ALIAS grid_map_core)
target_include_directories(grid_map_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS grid_map_core
  EXPORT grid_map_coreTargets
)
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT grid_map_coreTargets
  FILE grid_map_coreTargets.cmake
  NAMESPACE grid_map_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/grid_map_core
)

configure_package_config_file(
  cmake/grid_map_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/grid_map_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/grid_map_core
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/grid_map_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/grid_map_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/grid_map_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/grid_map_core
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(test_grid_map_core test/test_grid_map_core.cpp)
  target_link_libraries(test_grid_map_core PRIVATE grid_map_core GTest::gtest_main)
  gtest_discover_tests(test_grid_map_core)

  add_test(
    NAME grid_map_core_no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DCORE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/no_ros_dependencies.cmake
  )
endif()
