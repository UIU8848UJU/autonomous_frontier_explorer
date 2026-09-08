cmake_minimum_required(VERSION 3.16)
project(frontier_explorer_core VERSION 0.0.5 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(frontier_explorer_detection_core SHARED
  src/detector/frontier_detector.cpp
)
target_include_directories(frontier_explorer_detection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(frontier_explorer_scoring_core SHARED
  src/selector/frontier_candidate_ranker.cpp
  src/selector/frontier_scorer.cpp
  src/selector/score_components/clearance_score.cpp
  src/selector/score_components/cluster_size_score.cpp
  src/selector/score_components/distance_score.cpp
  src/selector/score_components/information_gain_score.cpp
  src/selector/score_components/retry_penalty_score.cpp
  src/selector/score_components/unknown_risk_penalty_score.cpp
)
target_include_directories(frontier_explorer_scoring_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(frontier_explorer_selection_core SHARED
  src/selector/frontier_pruner.cpp
  src/selector/frontier_selection_policy.cpp
)
target_include_directories(frontier_explorer_selection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(frontier_explorer_selection_core PUBLIC
  frontier_explorer_scoring_core
)

add_library(frontier_explorer_core INTERFACE)
target_include_directories(frontier_explorer_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(frontier_explorer_core INTERFACE
  frontier_explorer_detection_core
  frontier_explorer_scoring_core
  frontier_explorer_selection_core
)

set(exported_targets
  frontier_explorer_core
  frontier_explorer_detection_core
  frontier_explorer_scoring_core
  frontier_explorer_selection_core
)

install(TARGETS ${exported_targets}
  EXPORT frontier_explorer_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT frontier_explorer_coreTargets
  FILE frontier_explorer_coreTargets.cmake
  NAMESPACE frontier_explorer_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/frontier_explorer_core
)

configure_package_config_file(
  cmake/frontier_explorer_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/frontier_explorer_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/frontier_explorer_core
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/frontier_explorer_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/frontier_explorer_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/frontier_explorer_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/frontier_explorer_core
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  function(add_frontier_core_test target_name source_file linked_target)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE ${linked_target} GTest::gtest_main)
    gtest_discover_tests(${target_name})
  endfunction()

  add_frontier_core_test(
    test_frontier_detector_core
    test/test_frontier_detector_core.cpp
    frontier_explorer_detection_core
  )
  add_frontier_core_test(
    test_frontier_scoring_core
    test/test_frontier_scoring_core.cpp
    frontier_explorer_scoring_core
  )
  add_frontier_core_test(
    test_frontier_selection_core
    test/test_frontier_selection_core.cpp
    frontier_explorer_selection_core
  )

  add_test(
    NAME frontier_explorer_core_no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DCORE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/no_ros_dependencies.cmake
  )
endif()
