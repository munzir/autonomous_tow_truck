import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot')
    default_model_path = os.path.join(pkg_share, 'models/basic_mobile_bot_v2.urdf')
    static_map_path = os.path.join(pkg_share, 'maps', 'map_name.yaml')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params_hardware.yaml')
    imu_launch_path = os.path.join(pkg_share, 'launch', 'imu.launch.py')
    odometry_launch_path = os.path.join(pkg_share, 'launch', 'odometry.launch.py')
    lidar_launch_path = os.path.join(
        FindPackageShare('sllidar_ros2').find('sllidar_ros2'),
        'launch',
        'sllidar_a3_launch.py'
    )
    
    # Launch configuration variables
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl = LaunchConfiguration('amcl')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    
    # Declare launch arguments
    declare_amcl_cmd = DeclareLaunchArgument(
        name='amcl', default_value='False', description='Use AMCL-based localization')
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', default_value='true', description='Automatically startup the nav2 stack')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map', default_value=static_map_path, description='Full path to map file to load')
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file', default_value=nav2_params_path, description='Full path to ROS2 parameters file')
    
    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': Command(['xacro ', default_model_path])}],
        arguments=[default_model_path])
    
    # Include the IMU launch file
    start_imu_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include the LiDAR launch file only if AMCL is enabled
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(amcl),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include odometry launch file
    start_odometry_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Launch Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments = {'namespace': namespace,
                            'use_namespace': use_namespace,
                            'slam': slam,
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'params_file': params_file,
                            'autostart': autostart}.items())
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_amcl_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # Add actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_imu_cmd)
    ld.add_action(start_lidar_cmd)
    ld.add_action(start_odometry_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    
    return ld
