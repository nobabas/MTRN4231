cmake_minimum_required(VERSION 3.8)
project(brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)          # <--- Added rclcpp explicitly
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
# These are optional depending on if you use them in your .cpp files, 
# but good to keep if you had them before:
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 2. INCLUDE DIRECTORIES
# Tells CMake to look in the 'include' folder for your .hpp files
include_directories(include)

# 3. BUILD EXECUTABLE
# List EVERY .cpp file here. If you miss one, you get "undefined reference" errors.
add_executable(brain
  src/brain.cpp
  src/routines/soil_routine.cpp
  src/routines/topography_routine.cpp
  src/routines/vertical_profile_routine.cpp
  src/routines/heat_map_routine.cpp
)

# 4. TARGET INCLUDE DIRECTORIES
# Explicitly links the include folder to this specific executable
target_include_directories(brain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 5. LINK DEPENDENCIES
ament_target_dependencies(brain
  rclcpp
  std_msgs
  std_srvs
  interfaces
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# 6. INSTALLATION RULES
install(TARGETS brain
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# If you still use dummy_inputs.py for testing
install(PROGRAMS
  test_nodes/dummy_inputs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()