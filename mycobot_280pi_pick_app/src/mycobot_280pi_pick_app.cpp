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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>


#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>


#include <vector>
#include <signal.h>
#include <cmath>
#include <iostream>
#include <string>
#include <filesystem>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


using namespace std::chrono_literals;

class MoveIt_Task
{
public:
  // Constructor
  MoveIt_Task(const std::shared_ptr<rclcpp::Node> &node, 
              moveit::planning_interface::MoveGroupInterface &move_group_arm, 
              moveit::planning_interface::MoveGroupInterface &move_group_gripper,
              moveit::planning_interface::PlanningSceneInterface &planning_scene)
              : node_(node), move_group_arm_(move_group_arm), 
              move_group_gripper_(move_group_gripper), 
              planning_scene_(planning_scene),
              transform_stamped_()  // Initialize with default transform

  {

        node_->declare_parameter<bool>("sim", true);
        centroid_ = {640/2, 480/2};


        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);


        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



    // Set the maximum velocity scaling factor (optional)
    move_group_arm_.setMaxVelocityScalingFactor(0.75/2);
    move_group_arm_.setMaxAccelerationScalingFactor(0.9/2);

///////////////////////////////////////////////////////////////////

   //move_group_arm_.startStateMonitor(5.0);

///////////////////////////////////////////////////////////////////

    // Set planner ID
    // move_group_.setPlannerId("RRTstarkConfigDefault");  // Switch to RRT* planner
    move_group_arm_.setPlannerId("RRTConnectkConfigDefault");  // Switch to RRTConnect planner
    // move_group_.setPlannerId("APSConfigDefault");

    move_group_arm_.setPlanningPipelineId("ompl");     // Set planning pipeline
    move_group_arm_.setGoalPositionTolerance(1);    // 1 mm tolerance
    move_group_arm_.setGoalOrientationTolerance(1); // Tolerance for orientation (0.01 radians = 0.57 degress)

    // Set planning time and planning attempts
    move_group_arm_.setPlanningTime(5.0);     // 10 seconds max planning time
    move_group_arm_.setNumPlanningAttempts(10); // Try planning 5 times

    //Homing first
    move_home("home");
    //Goto
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
    //move_gripper("open");
    //std::vector<double> joint_goal_degrees_pose1 = {-0.2443,-1.0821,0.4189,-0.9250,0.1222,-0.2793};
    //move_abs_joints(joint_goal_degrees_pose1);
    //rclcpp::sleep_for(std::chrono::milliseconds(2000));

    node_->get_parameter("sim", sim);

    // Image publisher for the contour image
    contour_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/contour_image", 10);


    if(sim)
    {

    RCLCPP_WARN(node_->get_logger(), "Simulation setting loaded 1..");

    // Image and Point Cloud subscribers
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/camera_head/color/image_raw", 10, std::bind(&MoveIt_Task::image_callback, this, std::placeholders::_1));

    RCLCPP_WARN(node_->get_logger(), "Simulation setting loaded 2..");

    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera_head/depth/color/points", 10, std::bind(&MoveIt_Task::pointcloud_callback, this, std::placeholders::_1));
    }
    else{

    RCLCPP_WARN(node_->get_logger(), "Real robot setting loaded..");

    // Image and Point Cloud subscribers
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&MoveIt_Task::image_callback, this, std::placeholders::_1));

    pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10, std::bind(&MoveIt_Task::pointcloud_callback, this, std::placeholders::_1));


    }


  }
  // Destructor
  ~MoveIt_Task()
  {
    RCLCPP_WARN(node_->get_logger(), "Closing application...");
  }


    geometry_msgs::msg::TransformStamped get_transform(
        const std::string &target_frame, 
        const std::string &source_frame)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // Lookup transform from source_frame to target_frame
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get transform: %s", ex.what());
        }
        return transform_stamped;
    }


    void update_gripper_pose()
    {
        try
        {
            transform_stamped_ = get_transform("g_base", "gripper_base");
            RCLCPP_INFO(node_->get_logger(),
                        "Translation of gripper: x=%.3f, y=%.3f, z=%.3f, x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                        transform_stamped_.transform.translation.x,
                        transform_stamped_.transform.translation.y,
                        transform_stamped_.transform.translation.z,
                        transform_stamped_.transform.rotation.x,
                        transform_stamped_.transform.rotation.y,
                        transform_stamped_.transform.rotation.z,
                        transform_stamped_.transform.rotation.w);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get transform: %s", ex.what());
        }
    }



    // Callback for image topic
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat hsv_img, mask1, mask2, combined_mask;


    //RCLCPP_WARN(node_->get_logger(), "Image callabck 1..");

        // Convert image to HSV format
        cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

        // Detect lower and upper ranges of red color
        cv::inRange(hsv_img, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv_img, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);

        combined_mask = mask1 | mask2;

        // Find contours in the mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    //RCLCPP_WARN(node_->get_logger(), "Image callabck 2..");


        if (!contours.empty()) {
            // Find the largest contour based on area
            auto largest_contour = *std::max_element(contours.begin(), contours.end(), [](auto &a, auto &b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

            // Calculate the centroid of the largest contour
            cv::Moments m = cv::moments(largest_contour);
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);

            // Draw a circle at the centroid
            cv::circle(img, cv::Point(cx, cy), 15, cv::Scalar(0, 255, 0), -1);

            // Store centroid for 3D point extraction
            centroid_ = {cx, cy};
        }


      //  RCLCPP_WARN(node_->get_logger(), "Image callabck 3..");

        // Republish the contour image
        auto contour_msg = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
        contour_image_pub_->publish(*contour_msg);

      //  RCLCPP_WARN(node_->get_logger(), "Image callabck 4..");

        // Display the image locally (optional)
        //cv::imshow("Red Object Detection", img);
        //cv::waitKey(1);
    }



    // Function to publish TF
    void publish_tf_object(double x, double y, double z) {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = node_->get_clock()->now();
        if(sim)
        {
          transformStamped.header.frame_id = "camera_head_depth_frame";   // Parent frame
        }
        else
        {
          transformStamped.header.frame_id = "camera_depth_frame";   // Parent frame

        }
        transformStamped.child_frame_id = "detected_object"; // Child frame

        if(sim)
        {
          transformStamped.transform.translation.x = x;
          transformStamped.transform.translation.y = y;
          transformStamped.transform.translation.z = z;
        }
        else
        {
          transformStamped.transform.translation.x = z;
          transformStamped.transform.translation.y = y;
          transformStamped.transform.translation.z = x;

        }

        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0; // Identity quaternion (no rotation)

        tf_broadcaster_->sendTransform(transformStamped);

        RCLCPP_INFO(node_->get_logger(), "Published TF of Detect Object: [%f, %f, %f]", x, y, z);

        publish_goal_tf_frame("detected_object");

        //It will update the current gripper pose w r t to the base
        update_gripper_pose();
    }



    // Function to publish TF
    void publish_tf_goal(double x, double y, double z) {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = node_->get_clock()->now();
        transformStamped.header.frame_id = "g_base";   // Parent frame
        transformStamped.child_frame_id = "goal_pose"; // Child frame

        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;


        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0; // Identity quaternion (no rotation)

        tf_broadcaster_->sendTransform(transformStamped);

        RCLCPP_INFO(node_->get_logger(), "Published TF to the Goal Pose: [%f, %f, %f]", x, y, z);

    }



    // Callback for point cloud topic
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {


        RCLCPP_WARN(node_->get_logger(), "Point Cloud Callback");

        if (centroid_.has_value()) {
            int point_index = (centroid_->y * msg->width + centroid_->x);

            // Access the data in the point cloud
            float* data_ptr = reinterpret_cast<float*>(&msg->data[point_index * msg->point_step]);


            if (data_ptr != nullptr) {
              
            RCLCPP_WARN(node_->get_logger(), " Data pointer is valid");

            try{

            geometry_msgs::msg::Point pt;

            if (std::isnan(data_ptr[0]) || std::isinf(data_ptr[0]) || std::isnan(data_ptr[1]) || std::isinf(data_ptr[1]) || std::isnan(data_ptr[2]) || std::isinf(data_ptr[2])) {

                RCLCPP_WARN(node_->get_logger(), "Unable to fnd points in point cloud");

            }
            else
            {
              pt.x = data_ptr[0];
              pt.y = data_ptr[1];
              pt.z = data_ptr[2];

              RCLCPP_INFO(node_->get_logger(), "3D Point: x=%f, y=%f, z=%f", pt.x, pt.y, pt.z);


              // Publish the TF
              publish_tf_object( pt.x, pt.y, pt.z);
            }


            }

            catch(int e)
            {
              RCLCPP_INFO(node_->get_logger(), "Exception in getting 3D coordinates");
            }

           RCLCPP_WARN(node_->get_logger(), "PCL callabck 3..");

          }
          else
          {
              RCLCPP_ERROR(node_->get_logger(), "Null pointer detected for point data.");

          }

        }
    }



    void publish_goal_tf_frame(const std::string &target_frame) {
        try {
            // Lookup the transform from the base frame to the target frame
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("g_base", target_frame, tf2::TimePointZero);



            // Convert to PoseStamped for MoveIt
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "g_base";
            target_pose.header.stamp = node_->get_clock()->now();

            target_pose.pose.position.x = transform_stamped.transform.translation.x;
            target_pose.pose.position.y = transform_stamped.transform.translation.y;
            target_pose.pose.position.z = 0.12;
            //target_pose.pose.position.z = transform_stamped.transform.translation.z;

            //Publish TF of goal
            publish_tf_goal(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
        }
    }
        




//Move a particular joint
bool move_joint(int index, double joint_value)
{
    std::vector<double> joint_group_positions;
    move_group_arm_.getCurrentState()->copyJointGroupPositions("arm", joint_group_positions);
    joint_group_positions[index] = joint_value;
    move_group_arm_.setJointValueTarget(joint_group_positions);
    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        moveit::core::MoveItErrorCode exec_status = move_group_arm_.execute(plan); // Execute the Cartesian motion
        std::cout << "Execution Status Code: " << exec_status.val << std::endl;
        RCLCPP_INFO(node_->get_logger(), "Successfully moved the joint!  [%d]   [%f]",index,joint_value);
        switch (exec_status.val) 
        {
          case moveit::core::MoveItErrorCode::SUCCESS:
            std::cout << "Execution Status: SUCCESS" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::FAILURE:
            std::cout << "Execution Status: FAILURE" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::PLANNING_FAILED:
            std::cout << "Execution Status: PLANNING_FAILED" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
            std::cout << "Execution Status: INVALID_MOTION_PLAN" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            std::cout << "Execution Status: MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::CONTROL_FAILED:
            std::cout << "Execution Status: CONTROL_FAILED" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
            std::cout << "Execution Status: UNABLE_TO_AQUIRE_SENSOR_DATA" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::TIMED_OUT:
            std::cout << "Execution Status: TIMED_OUT" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::PREEMPTED:
            std::cout << "Execution Status: PREEMPTED" << std::endl;
            break;
          default:
            std::cout << "Execution Status: UNKNOWN ERROR CODE (" << exec_status.val << ")" << std::endl;
        }
      return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to plan for the joint movement.[%d]   [%f]",index,joint_value);
        return false;
    }
}



  // Function to move the robot to a predefined "home" position using a named target
  bool move_home(std::string home_pose)
  {

    std::string named_target = home_pose; // Store the provided named target for the robot's home position

    bool target_exists = move_group_arm_.setNamedTarget(named_target); // Set the named target for the robot's arm (move_group uses named targets predefined in the robot's configuration)

    if (!target_exists)
    {
      RCLCPP_ERROR(node_->get_logger(), "Named target '%s' not found!", named_target.c_str()); // Log an error message if the target doesn't exist
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // Plan the motion to the named target (MoveIt creates a trajectory to move to the target)

    // If planning was successful, execute the planned motion
    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning to named target '%s' successful!", named_target.c_str());

      move_group_arm_.execute(my_plan); // Execute the planned motion to the home position
      //current_plan = my_plan;

      RCLCPP_INFO(node_->get_logger(), "Motion to '%s' executed.", named_target.c_str());

      //set_lifting_height(0.0);

      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning to named target '%s' failed!", named_target.c_str());
      return false;
    }
    
  }

////////////////////////////////////////////////////


  // Function to move the robot to a predefined "home" position using a named target
  bool move_gripper(std::string home_pose)
  {

    std::string named_target = home_pose; // Store the provided named target for the robot's home position

    bool target_exists = move_group_gripper_.setNamedTarget(named_target); // Set the named target for the robot's arm (move_group uses named targets predefined in the robot's configuration)

    if (!target_exists)
    {
      RCLCPP_ERROR(node_->get_logger(), "Named target '%s' not found!", named_target.c_str()); // Log an error message if the target doesn't exist
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_gripper_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); // Plan the motion to the named target (MoveIt creates a trajectory to move to the target)

    // If planning was successful, execute the planned motion
    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning to named target '%s' successful!", named_target.c_str());

      move_group_arm_.execute(my_plan); // Execute the planned motion to the home position
      //current_plan = my_plan;

      RCLCPP_INFO(node_->get_logger(), "Motion to '%s' executed.", named_target.c_str());

      //set_lifting_height(0.0);

      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning to named target '%s' failed!", named_target.c_str());
      return false;
    }
    
  }


//////////////////////////////////////////////////////////////////


  // Function to move the robot's end-effector in Cartesian space relative to its current position
  bool move_relative_cartesian(std::string axis, double value)
  {
    
    geometry_msgs::msg::PoseStamped current_pose = move_group_arm_.getCurrentPose(); // Get the current pose (position and orientation) of the end-effector

    move_group_arm_.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;          // Define waypoints for the Cartesian motion (a series of poses the robot will follow)
    geometry_msgs::msg::Pose target_pose = current_pose.pose; // Initialize the target pose with the current pose

    std::string x_axis = "x"; // Define strings to represent the axes of movement
    std::string y_axis = "y";
    std::string z_axis = "z";

    // Check which axis to move along and update the corresponding position in the target pose
    if (axis == x_axis)
    {
      target_pose.position.x = target_pose.position.x + value; // Move along the x-axis by adding the value to the current x position
      waypoints.push_back(target_pose);                        // Add the updated pose to the waypoints
    }
    else if (axis == y_axis)
    {
      target_pose.position.y = target_pose.position.y + value;
      waypoints.push_back(target_pose);
    }
    else if (axis == z_axis)
    {
      target_pose.position.z = target_pose.position.z + value;
      waypoints.push_back(target_pose);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "No axis found");
      return false;
    }

    // Plan the Cartesian path using the waypoints
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;      // End-effector step size in meters
    const double jump_threshold = 0.0; // Disable jump threshold (prevents sudden movements)

    int try_count = 0;

    // Execute the trajectory if at least 90% of the path was computed successfully
    while (try_count<10)
    {


      double fraction = move_group_arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(node_->get_logger(), "Cartesian path (%.2f%% achieved)", fraction * 100.0); // Print the result of Cartesian path planning (the percentage of the Cartesian path that was successfully computed)

      if (fraction > 0.7)
      {
        // Create a MoveGroupInterface plan and assign the computed trajectory to it
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = trajectory;

        moveit::core::MoveItErrorCode exec_status = move_group_arm_.execute(cartesian_plan); // Execute the Cartesian motion
        std::cout << "Execution Status Code: " << exec_status.val << std::endl;
        switch (exec_status.val) 
        {
          case moveit::core::MoveItErrorCode::SUCCESS:
            std::cout << "Execution Status: SUCCESS" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::FAILURE:
            std::cout << "Execution Status: FAILURE" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::PLANNING_FAILED:
            std::cout << "Execution Status: PLANNING_FAILED" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
            std::cout << "Execution Status: INVALID_MOTION_PLAN" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            std::cout << "Execution Status: MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::CONTROL_FAILED:
            std::cout << "Execution Status: CONTROL_FAILED" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
            std::cout << "Execution Status: UNABLE_TO_AQUIRE_SENSOR_DATA" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::TIMED_OUT:
            std::cout << "Execution Status: TIMED_OUT" << std::endl;
            break;
          case moveit::core::MoveItErrorCode::PREEMPTED:
            std::cout << "Execution Status: PREEMPTED" << std::endl;
            break;
          default:
            std::cout << "Execution Status: UNKNOWN ERROR CODE (" << exec_status.val << ")" << std::endl;
        }



        //current_plan = cartesian_plan;

        RCLCPP_INFO(node_->get_logger(), "Cartesian motion executed.");
        //Saving trajectory
        //save_trajectory(file_name);

        ++try_count;
        return true;
      }
      else if(try_count < 10)
      { 
        ++try_count;
        RCLCPP_WARN(node_->get_logger(), "Failed to compute a valid Cartesian path. Retrying...");
        continue;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to compute a valid Cartesian path after several attempts. Stopping...");
        //stopRobot();        // Stop the robot
        rclcpp::shutdown(); // Shut down ROS safely
        exit(0);            // Exit the application
        return false;
      }
 
    
    }

    exit(0);            // Exit the application
    return false; // Ensure function always returns a boolean value
  }



///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool is_frame_available(const std::string &target_frame, const std::string &source_frame)
    {
        try
        {
            // Check for transform availability with a timeout of 100ms
            tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(100));
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(node_->get_logger(), "Frame not available: %s", ex.what());
            return false;
        }
    }



  // Function to move the robot's end-effector in Cartesian space relative to its current position
  bool move_abs_cartesian(double x, double y, double z)
  {

    //rclcpp::sleep_for(std::chrono::seconds(2));

    //geometry_msgs::msg::PoseStamped current_pose = move_group_arm_.getCurrentPose(); // Get the current pose (position and orientation) of the end-effector

    std::vector<geometry_msgs::msg::Pose> waypoints;          // Define waypoints for the Cartesian motion (a series of poses the robot will follow)
    geometry_msgs::msg::Pose target_pose ; // Initialize the target pose with the current pose

    target_pose.position.x =  x; // Update the target pose with the absolute values provided for x, y, and z coordinates
    target_pose.position.y =  y;
    target_pose.position.z =  z;
    waypoints.push_back(target_pose); // Add the updated target pose to the waypoints


    // Plan the Cartesian path using the waypoints
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.1;      // End-effector step size in meters
    const double jump_threshold = 0.0; // Disable jump threshold (prevents sudden movements)

    int try_count = 0;

    // Execute the trajectory if at least 90% of the path was computed successfully
    //while (try_count<20)
    //{


      //if(mode == "save")
      //{

      double fraction = move_group_arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(node_->get_logger(), "Cartesian path (%.2f%% achieved)", fraction * 100.0); // Print the result of Cartesian path planning (the percentage of the Cartesian path that was successfully computed)

      if (fraction > 0.3)
      {
        // Create a MoveGroupInterface plan and assign the computed trajectory to it
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = trajectory;

        move_group_arm_.execute(cartesian_plan); // Execute the Cartesian motion
        //current_plan = cartesian_plan;

        RCLCPP_INFO(node_->get_logger(), "Cartesian motion executed.");
        //Saving trajectory
        //save_trajectory(file_name);

        //++try_count;
        return true;
      }
      /*
      else if(try_count < 10)
      { 
        ++try_count;
        RCLCPP_WARN(node_->get_logger(), "Failed to compute a valid Cartesian path. Retrying...");
        continue;
      }
      */
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to compute a valid Cartesian path after several attempts. Stopping...");
        //stopRobot();        // Stop the robot
        //rclcpp::shutdown(); // Shut down ROS safely
        //exit(0);            // Exit the application
        return false;
      }
    
     //}
   
    //}


    exit(0);            // Exit the application
    return false; // Ensure function always returns a boolean value
  }

//////////////////////////////////////////////////////////////////////////


 // Function to move the robot's end-effector to an absolute position with orientation constraints
  bool move_abs(std::vector<double> position, std::string target_orientation= "N")
  {
    geometry_msgs::msg::Pose target_pose; // Define a target pose (position and orientation) for the robot

    double x = position[0];
    double y = position[1];
    double z = position[2];

    auto start_time_total = node_->now();

    int current_try = 1;
    while(current_try < max_planning_tries)
    {

      auto start_time_last = node_->now();

    if (target_orientation == "N")
    {
      target_pose.orientation.x = 0.0;  
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 1.0;
    }
    else if (target_orientation == "E")
    {
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 1.0 / std::sqrt(2);
      target_pose.orientation.w = 1.0 / std::sqrt(2);
    }
    else if (target_orientation == "S")
    {
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 1.0;
      target_pose.orientation.w = 0.0;
    }
    else if (target_orientation == "W")
    {
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = -1.0 / std::sqrt(2);
      target_pose.orientation.w = 1.0 / std::sqrt(2);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "No orientation found");
      return false;
    }
    

    target_pose.position.x = x; // Set the target position with the provided x, y, and z coordinates
    target_pose.position.y = y;
    target_pose.position.z = z;

    move_group_arm_.setPoseTarget(target_pose); // Apply the target pose (both position and orientation)
    // move_group_arm_.setRandomTarget();

    /////////////////////////////////////////////////////////////////////////////////////


      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      moveit::core::MoveItErrorCode plan_result = move_group_arm_.plan(my_plan);

      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        // If planning was successful, execute the plan
        RCLCPP_INFO(node_->get_logger(), "Planning successful on try %d, executing the plan...", current_try);
        
        auto end_time = node_->now();

        RCLCPP_INFO(node_->get_logger(), "Constrained absolute path computed after %d tries in %f seconds (last planning time: %fs)", current_try, (end_time - start_time_total).seconds(), (end_time - start_time_last).seconds());
        move_group_arm_.execute(my_plan);
        //current_plan = my_plan;
        //save_trajectory(file_name);

        move_group_arm_.clearPathConstraints(); // Clear the path constraints after the motion is completed
        return true;                     // Return true if the motion was successfully planned and executed

      }
      else
      {
        std::string error_message = getErrorMessageForMoveItCode(plan_result);
        switch (plan_result.val) 
        {
          case moveit::core::MoveItErrorCode::PLANNING_FAILED:
              error_message = "Planning failed.";
              break;
          case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
              error_message = "Invalid motion plan.";
              break;
          case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
              error_message = "Invalid robot state.";
              break;
          case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
              error_message = "No IK solution could be found.";
              break;
          case moveit::core::MoveItErrorCode::TIMED_OUT:
              error_message = "Planning timed out.";
              break;
          case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
              error_message = "Goal is in collision.";
              break;
          case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
              error_message = "Start state is in collision.";
              break;
          case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
              error_message = "Start state violates path constraints.";
              break;
          default:
              error_message = "Unknown error code: " + std::to_string(plan_result.val);
              break;
        }
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error: %s", error_message.c_str());
        ++current_try;
        continue;
        //stopRobot();        // Stop the robot
        rclcpp::shutdown(); // Shut down ROS safely
        exit(0);            // Exit the application
      }
    //}


    }

    RCLCPP_ERROR(node_->get_logger(), "Shutting down...");
    throw std::runtime_error("Failed to plan and execute the motion");
  }

  ///////////////////////////////////////////

  std::string getErrorMessageForMoveItCode(const moveit::core::MoveItErrorCode& error_code) 
  {
    switch (error_code.val) 
    {
      case moveit::core::MoveItErrorCode::PLANNING_FAILED:
        return "Planning failed.";
      case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
        return "Invalid motion plan.";
      case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
        return "Invalid robot state.";
      case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
        return "No IK solution could be found.";
      case moveit::core::MoveItErrorCode::TIMED_OUT:
        return "Planning timed out.";
      case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
        return "Goal is in collision.";
      case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
        return "Start state is in collision.";
      case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "Start state violates path constraints.";
      default:
        return "Unknown error code: " + std::to_string(error_code.val);
    }
  }


///////////////////////////////////////////////////////////////////
  bool move_abs_joints(const std::vector<double>& joint_goal)
  {
    int current_try = 1;
    while(current_try < max_planning_tries)
    {
      // Set the joint goal
      move_group_arm_.setJointValueTarget(joint_goal);
      // Plan the motion
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group_arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
          RCLCPP_INFO(node_->get_logger(), "Planning successful! Executing the plan...");
          move_group_arm_.execute(plan);
          return success;
      }
      else
      {
          ++current_try;
          RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
          continue;
      }
    }
    RCLCPP_ERROR(node_->get_logger(), "Shutting down...");
    throw std::runtime_error("Failed to plan and execute the motion");
  }




//////////////////////////////////////////////////////////////////////
private:
  std::shared_ptr<rclcpp::Node> node_;                         // Store the node pointer
  moveit::planning_interface::MoveGroupInterface &move_group_arm_; // Store the reference
  moveit::planning_interface::MoveGroupInterface &move_group_gripper_; // Store the reference
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  int max_planning_tries = 100;

  std::optional<cv::Point> centroid_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr contour_image_pub_; // Publisher for contour image

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // TF broadcaster
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_stamped_;  // Stores the transform

  //Run on gazebo or real robot
  bool sim;


};
int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);                                          // Initialize ROS 2
  auto node = rclcpp::Node::make_shared("pick_app"); // Create a ROS2 node called 'moveit2_cartesian_motion'

  moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");  // Create a MoveGroupInterface object to control the robot arm with the 'tmr_arm' group
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");  // Create a MoveGroupInterface object to control the robot arm with the 'tmr_arm' group

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // Create a PlanningSceneInterface for managing collision objects in the environment

  rclcpp::executors::MultiThreadedExecutor executor; // Start a multi-threaded ROS2 executor to handle callbacks
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto move_obj = MoveIt_Task(node, move_group_arm, move_group_gripper, planning_scene_interface); // Instantiate the MoveIt_Task 

  while (rclcpp::ok()) {


    rclcpp::sleep_for(std::chrono::milliseconds(2000));

/*
    move_obj.move_home("home");
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    move_obj.move_gripper("open");
    std::vector<double> joint_goal_degrees_pose1 = {-0.139626, -0.837758, -0.942478, 0.209439, -0.017453, -0.139626};
    move_obj.move_abs_joints(joint_goal_degrees_pose1);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    move_obj.move_gripper("open");
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    move_obj.move_gripper("close");
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    std::vector<double> joint_goal_degrees_pose2 = {0.523599,-0.715584,-1.186823,0.314159,0.017453,0.523599};
    move_obj.move_abs_joints(joint_goal_degrees_pose2);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    move_obj.move_gripper("open");
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
*/

  }

  rclcpp::shutdown();


  return 0;

}