from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mycobot_moveit", package_name="mycobot_moveit_config_manual_setup")
        .robot_description(file_path="config/mycobot_280.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .sensors_3d(
            file_path=None
        )     
        .to_moveit_configs()
    )



    # Generate the demo launch with static_tf included
    demo_launch = generate_demo_launch(moveit_config)

    return demo_launch


#    moveit_config = MoveItConfigsBuilder("opaul", package_name="opaul_moveit_config").to_moveit_configs()
#    return generate_demo_launch(moveit_config)


#    moveit_config = MoveItConfigsBuilder("opaul", package_name="opaul_moveit_config").to_moveit_configs()
#    return generate_demo_launch(moveit_config)