cmake_minimum_required(VERSION 3.5)
project(moveit_planning_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interfaces REQUIRED)

add_executable(moveit_server_node src/moveit_server.cpp)

ament_target_dependencies(moveit_server_node
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
  interfaces
)

install(TARGETS moveit_server_node DESTINATION lib/${PROJECT_NAME})

ament_package()
