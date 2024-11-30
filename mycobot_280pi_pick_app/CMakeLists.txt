cmake_minimum_required(VERSION 3.5)
project(mycobot_280pi_pick_app)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(mycobot_280pi_pick_app src/mycobot_280pi_pick_app.cpp)
ament_target_dependencies(mycobot_280pi_pick_app rclcpp moveit_ros_planning_interface)

install(TARGETS mycobot_280pi_pick_app DESTINATION lib/${PROJECT_NAME})

ament_package()
