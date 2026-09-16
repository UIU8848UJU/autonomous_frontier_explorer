cmake_minimum_required(VERSION 3.8)
project(autonomousr_explorer_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/readiness_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
