cmake_minimum_required(VERSION 3.8)
project(px4ctrl_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add main executable with SE3 controller
add_executable(px4ctrl_ros2_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/se3con.cpp
  src/input.cpp
  src/KF_filter.cpp
  src/timestamp_manager.cpp

)

# Test executables - moved to test folder
add_executable(data_monitor_node
  test/data_monitor_node.cpp
  src/input.cpp
  src/timestamp_manager.cpp
)

# KF Filter test executable
add_executable(test_kf_filter_node
  src/test_kf_filter.cpp
  src/KF_filter.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
  src/timestamp_manager.cpp
)

# KF Filter Publisher executable
add_executable(kf_filter_publisher_node
  src/kf_filter_publisher.cpp
  src/KF_filter.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
  src/timestamp_manager.cpp
)

# Simple KF Filter Publisher executable
add_executable(simple_kf_publisher_node
  src/kf_filter_publisher_simple.cpp
  src/input.cpp
  src/timestamp_manager.cpp
)



# hover_example_node disabled - not needed for compilation

# Specify dependencies for main node
ament_target_dependencies(px4ctrl_ros2_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

ament_target_dependencies(data_monitor_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

ament_target_dependencies(test_kf_filter_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_msgs
  tf2_msgs
  Eigen3
)

ament_target_dependencies(kf_filter_publisher_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_msgs
  tf2_msgs
  std_msgs
  Eigen3
)

ament_target_dependencies(simple_kf_publisher_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_msgs
  tf2_msgs
  std_msgs
  Eigen3
)

# hover_example_node dependencies removed

# Install executables
install(TARGETS 
  px4ctrl_ros2_node
  data_monitor_node
  test_kf_filter_node
  kf_filter_publisher_node
  simple_kf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/uav_tf_publisher.py
  scripts/test_kf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()