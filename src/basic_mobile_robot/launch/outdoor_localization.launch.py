# outdoor_localization.launch.py
# This is meant to be launched after the main indoor launch file is already running.
# It only starts the navsat_transform_node for outdoor (GPS-based) localization.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths and config
    pkg_share = FindPackageShare('basic_mobile_robot').find('basic_mobile_robot')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # NavSat Transform Node
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')
        ]
    )

      # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    
    ld.add_action(navsat_transform_node)

    return ld
