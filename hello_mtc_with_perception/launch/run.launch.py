# Author: Addison Sears-Collins
# Date: September 4, 2024
# Description: ROS 2 launch file for MoveIt Task Constructor pick and place with perception node.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
 

def generate_launch_description():
    # Constants for paths to different files and folders
    package_name_gazebo = 'mycobot_gazebo'
    package_name_moveit_config = 'mycobot_moveit_config_manual_setup'
    package_name_mtc = 'hello_mtc_with_perception'

    # Set the path to different files and folders
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
    pkg_share_mtc = FindPackageShare(package=package_name_mtc).find(package_name_mtc)

    # Paths for various configuration files
    urdf_file_path = 'urdf/mycobot_280.urdf.xacro'
    srdf_file_path = 'config/mycobot_280.srdf'
    moveit_controllers_file_path = 'config/moveit_controllers.yaml'
    joint_limits_file_path = 'config/joint_limits.yaml'
    kinematics_file_path = 'config/kinematics.yaml'
    pilz_cartesian_limits_file_path = 'config/pilz_cartesian_limits.yaml'
    initial_positions_file_path = 'config/initial_positions.yaml'
    mtc_node_params_file_path = 'config/mtc_node_params.yaml'

    # Set the full paths
    urdf_model_path = os.path.join(pkg_share_mtc, urdf_file_path)
    srdf_model_path = os.path.join(pkg_share_moveit_config, srdf_file_path)
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, moveit_controllers_file_path)
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, joint_limits_file_path)
    kinematics_file_path = os.path.join(pkg_share_moveit_config, kinematics_file_path)
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, pilz_cartesian_limits_file_path)
    initial_positions_file_path = os.path.join(pkg_share_moveit_config, initial_positions_file_path)
    mtc_node_params_file_path = os.path.join(pkg_share_mtc, mtc_node_params_file_path)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    exe = LaunchConfiguration('exe')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_exe_cmd = DeclareLaunchArgument(
        name="exe",
        default_value="mtc_node",
        description="Which node to run",
        choices=["mtc_node"])
  
    # Load the robot configuration
    # Typically, you would also have this line in here: .robot_description(file_path=urdf_model_path)
    # Another launch file is launching the robot description.
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name=package_name_moveit_config)
        .robot_description_semantic(file_path=srdf_model_path)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )
    
    node = Node(
        package=package_name_mtc,
        executable=exe,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.pilz_cartesian_limits,
            moveit_config.planning_pipelines,
            {'use_sim_time': use_sim_time},
            mtc_node_params_file_path,
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_exe_cmd)

    # Add any actions
    ld.add_action(node)

    return ld
    
