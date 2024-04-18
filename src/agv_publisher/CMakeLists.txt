cmake_minimum_required(VERSION 3.5)
project(agv_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(agv_publisher src/agv_publisher.cpp)
ament_target_dependencies(agv_publisher rclcpp std_msgs geometry_msgs)

install(TARGETS
  agv_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()