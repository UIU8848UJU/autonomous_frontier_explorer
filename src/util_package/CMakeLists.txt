cmake_minimum_required(VERSION 3.8)
project(util_package)
cmake_policy(SET CMP0009 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/
        DESTINATION include/)

include_directories(include)

ament_export_include_directories(include)  # 新增这一行
ament_package()