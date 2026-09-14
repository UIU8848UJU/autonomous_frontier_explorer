set(_scan_files
  "${CORE_SOURCE_DIR}/CMakeLists.txt"
  "${CORE_SOURCE_DIR}/package.xml"
)
file(GLOB_RECURSE _headers
  "${CORE_SOURCE_DIR}/include/*"
  "${CORE_SOURCE_DIR}/src/*"
)
list(APPEND _scan_files ${_headers})

set(_forbidden_patterns
  "ament_cmake"
  "rclcpp"
  "nav_msgs"
  "nav2_"
  "geometry_msgs"
  "tf2"
)

foreach(_file IN LISTS _scan_files)
  file(READ "${_file}" _content)
  foreach(_pattern IN LISTS _forbidden_patterns)
    string(FIND "${_content}" "${_pattern}" _position)
    if(NOT _position EQUAL -1)
      message(FATAL_ERROR "navigation_core 包含 ROS 依赖 '${_pattern}': ${_file}")
    endif()
  endforeach()
endforeach()
