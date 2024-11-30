#include <memory>

#include <rclcpp/rclcpp.hpp>                                          //Provides ROS2 node functionality
#include <moveit/move_group_interface/move_group_interface.h>         //Controls and plans robot arm motion
#include <moveit/planning_scene_interface/planning_scene_interface.h> //Allows interaction with the planning scene (environment), allows adding and removing collision objects
#include <moveit_msgs/msg/robot_trajectory.hpp>                       //For storing and executing planned trajectories
#include <moveit_msgs/msg/constraints.hpp>                            //Allows you to define and apply constraints like position, orientation, and joint limits in the MoveIt motion planning pipeline
#include <moveit_msgs/msg/object_color.hpp>                           // For coloring the object
#include <moveit_msgs/msg/planning_scene.hpp>                         // For applying object colors to the planning scene

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <vector>
#include <signal.h>
#include <cmath>
#include <iostream>
#include <string>
#include <filesystem>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


using namespace std::chrono_literals;

class MoveIt_Task
{
public:
  // Constructor
  MoveIt_Task(const std::shared_ptr<rclcpp::Node> &node, moveit::planning_interface::MoveGroupInterface &move_group, moveit::planning_interface::PlanningSceneInterface &planning_scene) : node_(node), move_group_(move_group), planning_scene_(planning_scene)
  {


   

  }
private:
  std::shared_ptr<rclcpp::Node> node_;                         // Store the node pointer
  moveit::planning_interface::MoveGroupInterface &move_group_; // Store the reference
  moveit::planning_interface::PlanningSceneInterface planning_scene_;



};
int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);                                          // Initialize ROS 2
  auto node = rclcpp::Node::make_shared("pick_app"); // Create a ROS2 node called 'moveit2_cartesian_motion'

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");  // Create a MoveGroupInterface object to control the robot arm with the 'tmr_arm' group
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // Create a PlanningSceneInterface for managing collision objects in the environment

  rclcpp::executors::MultiThreadedExecutor executor; // Start a multi-threaded ROS2 executor to handle callbacks
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto move_obj = MoveIt_Task(node, move_group, planning_scene_interface); // Instantiate the MoveIt_Task 

  return 0;

}