import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pkg_share = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    
    # Launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # Declare launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', 
        default_value=default_rviz_config_path, 
        description='Full path to the RViz config file')
    
    # Start RViz
    start_rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen')
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_rviz_cmd)
    
    return ld
