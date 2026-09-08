if(NOT DEFINED CORE_SOURCE_DIR)
  message(FATAL_ERROR "CORE_SOURCE_DIR is required")
endif()

set(scan_roots
  "${CORE_SOURCE_DIR}/CMakeLists.txt"
  "${CORE_SOURCE_DIR}/package.xml"
  "${CORE_SOURCE_DIR}/include"
  "${CORE_SOURCE_DIR}/src"
)

set(source_files)
foreach(scan_root IN LISTS scan_roots)
  if(IS_DIRECTORY "${scan_root}")
    file(GLOB_RECURSE discovered_files
      "${scan_root}/*.cpp"
      "${scan_root}/*.hpp"
      "${scan_root}/*.h"
    )
    list(APPEND source_files ${discovered_files})
  elseif(EXISTS "${scan_root}")
    list(APPEND source_files "${scan_root}")
  endif()
endforeach()

set(forbidden_dependencies
  "ament_cmake"
  "rclcpp"
  "nav_msgs"
  "nav2_"
  "geometry_msgs"
  "tf2"
)

set(violations)
foreach(source_file IN LISTS source_files)
  file(READ "${source_file}" source_content)
  foreach(forbidden_dependency IN LISTS forbidden_dependencies)
    string(FIND "${source_content}" "${forbidden_dependency}" match_position)
    if(NOT match_position EQUAL -1)
      file(RELATIVE_PATH relative_file "${CORE_SOURCE_DIR}" "${source_file}")
      list(APPEND violations "${relative_file}: ${forbidden_dependency}")
    endif()
  endforeach()
endforeach()

if(violations)
  list(REMOVE_DUPLICATES violations)
  list(JOIN violations "\n  " violation_text)
  message(FATAL_ERROR
    "frontier_explorer_core must be ROS/Nav2 independent:\n  ${violation_text}")
endif()

message(STATUS "frontier_explorer_core ROS/Nav2 dependency boundary is clean")
