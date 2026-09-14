cmake_minimum_required(VERSION 3.16)
project(navigation_core VERSION 0.1.0 LANGUAGES CXX)

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
find_package(robot_geometry_core CONFIG REQUIRED)

add_library(navigation_core SHARED
  src/path_safety_checker.cpp
  src/footprint_goal_validator.cpp
  src/single_goal_gate.cpp
  src/path_utils.cpp
)
add_library(navigation_core::navigation_core ALIAS navigation_core)

target_include_directories(navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(navigation_core PUBLIC
  grid_map_core::grid_map_core
  robot_geometry_core::robot_geometry_core
)

install(TARGETS navigation_core
  EXPORT navigation_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT navigation_coreTargets
  FILE navigation_coreTargets.cmake
  NAMESPACE navigation_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/navigation_core
)

configure_package_config_file(
  cmake/navigation_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/navigation_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/navigation_core
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/navigation_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/navigation_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/navigation_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/navigation_core
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  add_executable(test_navigation_core test/test_navigation_core.cpp)
  target_link_libraries(test_navigation_core PRIVATE
    navigation_core
    GTest::gtest_main
  )
  gtest_discover_tests(test_navigation_core)

  add_test(
    NAME navigation_core_no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DCORE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/no_ros_dependencies.cmake
  )
endif()
