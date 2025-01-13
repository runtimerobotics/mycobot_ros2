import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Declare arguments
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false', description='Simulation mode flag'
    )

    manual_arg = DeclareLaunchArgument(
        'manual', default_value='false', description='Manual mode flag'
    )

    # Get the sim and manual values as LaunchConfigurations
    sim = LaunchConfiguration('sim')
    manual = LaunchConfiguration('manual')

    # Simulation bringup launch file
    sim_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mycobot_280pi_gazebo'),
                'launch',
                'mycobot_280pi_bringup_gazebo_sim_vision.launch.py'
            )
        ),
        launch_arguments={'sim': sim, 'manual': manual}.items(),
        condition=IfCondition(sim)
    )

    # Real-world bringup launch file
    real_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mycobot_280pi_gazebo'),
                'launch',
                'mycobot_280pi_bringup_gazebo_real_vision.launch.py'
            )
        ),
        launch_arguments={'sim': sim, 'manual': manual}.items(),
        condition=UnlessCondition(sim)  # Runs if sim is not 'true'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'depth_module.depth_profile': '640x480x15',
            'pointcloud.enable': 'true',
            'align_depth': 'true',
            'pointcloud.ordered_pc': 'true'
        }.items(),
        condition=UnlessCondition(sim)  # Runs only if sim is false
    )



    # Launch the MoveIt config demo after 7 seconds
    demo_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mycobot_280pi_moveit_config'),
                        'launch',
                        'demo.launch.py'
                    )
                ),
                launch_arguments={'sim': sim, 'manual': manual}.items()
            )
        ]
    )

    # Start the pick app node after an additional 15 seconds (22 seconds total)
    pick_app_node = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='mycobot_280pi_pick_app',
                executable='mycobot_280pi_pick_app',
                output='screen',
                parameters=[{
                    'sim': sim,
                    'manual': manual
                }]
            )
        ]
    )


    return LaunchDescription([
        sim_arg,
        manual_arg,
        sim_bringup_launch,
        real_bringup_launch,
        demo_launch,
        #pick_app_node,
        realsense_launch  # Add this line

    ])
