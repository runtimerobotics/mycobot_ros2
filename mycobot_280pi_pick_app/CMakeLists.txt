cmake_minimum_required(VERSION 3.5)
project(mycobot_280pi_pick_app)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(mycobot_280pi_pick_app src/mycobot_280pi_pick_app.cpp)
ament_target_dependencies(mycobot_280pi_pick_app rclcpp moveit_ros_planning_interface sensor_msgs cv_bridge OpenCV)

install(TARGETS mycobot_280pi_pick_app DESTINATION lib/${PROJECT_NAME})

ament_package()
