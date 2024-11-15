# Minimum CMake version and project definition
cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(hello_mtc_with_perception)

# Build settings
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

# Define required packages (alphabetical order)
set(REQUIRED_PACKAGES
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  #moveit_task_constructor_core
  #moveit_task_constructor_msgs
  mycobot_interfaces
  PCL
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  shape_msgs
  tf2_eigen
  tf2_geometry_msgs
)

# Find required packages
foreach(package ${REQUIRED_PACKAGES})
  find_package(${package} REQUIRED)
endforeach()


install(DIRECTORY 
  config include/ launch rviz scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export and package
ament_package()


