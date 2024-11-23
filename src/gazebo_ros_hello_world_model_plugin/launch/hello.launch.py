import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the `empty_world.launch.py` from the `gazebo_ros` package
    print("Hello!")
    gazebo_ros_path = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_path = FindPackageShare(package='gazebo_ros_hello_world_model_plugin').find('gazebo_ros_hello_world_model_plugin')
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_path,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                pkg_path,
                'worlds',
                'hello.world'
            )
            # Add more arguments if needed
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(empty_world_launch)
    return ld
