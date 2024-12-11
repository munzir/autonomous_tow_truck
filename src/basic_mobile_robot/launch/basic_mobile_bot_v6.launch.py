import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot')
    default_model_path = os.path.join(pkg_share, 'models/basic_mobile_bot_v2.urdf')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    robot_name_in_urdf = 'basic_mobile_bot'
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    static_map_path = os.path.join(pkg_share, 'maps', 'map_name.yaml')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params_hardware.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    imu_launch_path = os.path.join(pkg_share, 'launch', 'imu.launch.py')  # Added IMU launch path
    # odometry_launch_path = os.path.join(pkg_share, 'launch', 'odometry.launch.py')  # Path to pose launch file
    # lidar_launch_path = os.path.join(pkg_share, 'launch', 'lasers.launch.py')  # Added LiDAR launch path
    # Launch configuration variables
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam = LaunchConfiguration('slam')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments  
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')
        
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use')
        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')
        
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',  # Set this to True if you want to use simulation time
        description='Use simulation (Gazebo) clock if true')


    # Specify the actions

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
                    {'use_sim_time': use_sim_time}])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': Command(['xacro ', model])}])

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])    

    #  Include the IMU launch file
    start_imu_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the LiDAR launch file
    lidar_launch_path = os.path.join(
        FindPackageShare('sllidar_ros2').find('sllidar_ros2'),
        'launch',
        'sllidar_a3_launch.py'
    )

    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # #Include odometry 
    # start_odometry_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(odometry_launch_path),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )


    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd) 

    # Add any actions
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_imu_cmd) #imu
    ld.add_action(start_lidar_cmd) #lidar
    ld.add_action(start_ros2_navigation_cmd)
    # ld.add_action(start_odometry_cmd)
    return ld
