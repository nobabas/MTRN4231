cmake_minimum_required(VERSION 3.8)
project(brain)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(brain src/brain.cpp)
ament_target_dependencies(brain rclcpp)

install(TARGETS brain
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()