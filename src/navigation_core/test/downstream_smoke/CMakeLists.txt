cmake_minimum_required(VERSION 3.16)
project(navigation_core_downstream_smoke LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(navigation_core CONFIG REQUIRED)

add_executable(navigation_core_downstream_smoke smoke.cpp)
target_link_libraries(navigation_core_downstream_smoke
  PRIVATE navigation_core::navigation_core
)
