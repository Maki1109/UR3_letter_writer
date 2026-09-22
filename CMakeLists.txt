cmake_minimum_required(VERSION 3.8)
project(ur3_letter_writer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(include)

add_executable(letter_writer_node
  src/letter_writer_node.cpp
  src/letters.cpp
)

ament_target_dependencies(letter_writer_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)

install(TARGETS
  letter_writer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
