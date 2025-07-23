import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
  static_map_path = os.path.join(pkg_share, 'maps', 'map_name.yaml')
  nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params_hardware.yaml')
  nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
  behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'sadaf_navigate_through_poses_w_replanning_and_recovery.xml')

  imu_launch_path = os.path.join(pkg_share, 'launch', 'imu.launch.py')
  odometry_launch_path = os.path.join(pkg_share, 'launch', 'odometry.launch.py')

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
  amcl = LaunchConfiguration('amcl')
  ekf = LaunchConfiguration('ekf')
  joystick = LaunchConfiguration('joystick')

  # Set nav2_launch_dir based on AMCL condition
  default_launch_dir = os.path.join(pkg_share, 'launch')
  nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
  nav2_launch_dir = os.path.join(nav2_dir, 'launch')

  # Declare the launch arguments
  declare_amcl_cmd = DeclareLaunchArgument(
    name='amcl',
    default_value='True',
    description='Use AMCL-based localization'
  )

  declare_joystick_cmd = DeclareLaunchArgument(
    name='joystick',
    default_value='False',
    description='Use joystick'
  )

  declare_ekf_cmd = DeclareLaunchArgument(
    name='ekf',
    default_value='False',
    description='Use ekf-based localization'
  )

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
    default_value='False',
    description='Use simulation (Gazebo) clock if true')

  # Start robot localization using an Extended Kalman filter
  # start_robot_localization_cmd = Node(
  #   condition = IfCondition(ekf),
  #   package='robot_localization',
  #   executable='ekf_node',
  #   name='ekf_filter_node',
  #   output='screen',
  #   parameters=[robot_localization_file_path, 
  #   {'use_sim_time': use_sim_time}])

  # Start robot state publisher
  start_robot_state_publisher_cmd = Node(
      condition=IfCondition(use_robot_state_pub),
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace=namespace,
      parameters=[{'use_sim_time': use_sim_time, 
                   'robot_description': Command(['xacro ', model])}],
                   arguments=[default_model_path])

  # joynode
  start_joy_node = Node(
    condition=IfCondition(joystick),
    package='joy',
    executable='joy_node',
    output='screen'
  )

  # Start the YOLO detection node
  start_detection_node = Node(
    package="my_detection_package",
    executable="object_detection_node",  # This matches setup.py entry point
    output="screen"
  )
# 
  # Start the bridge node to convert Detection2DArray to PointCloud2
  # start_detection_bridge_node = Node(
  #   package="my_detection_package",
  #   executable="detection2darray_to_pointcloud2",
  #   name="detection_bridge_node",
  #   output="screen"
  # )
  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  # Include the IMU launch file
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

  # Include the LiDAR launch file only if AMCL is enabled
  start_lidar_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(lidar_launch_path),
    condition=IfCondition(amcl),  # This ensures LiDAR only starts if AMCL is True
    launch_arguments={'use_sim_time': use_sim_time}.items()
  )


  # Include odometry
  start_odometry_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(odometry_launch_path),
      launch_arguments={'joystick': joystick}.items()
  )

  # Launch the ROS 2 Navigation Stack
  start_ros2_navigation_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(default_launch_dir, 'bringup_launch.py')),
      launch_arguments = {
          'namespace': namespace,
          'use_namespace': use_namespace,
          'slam': slam,
          'map': map_yaml_file,
          'use_sim_time': use_sim_time,
          'params_file': params_file,
          'default_bt_xml_filename': default_bt_xml_filename,
          'autostart': autostart
      }.items(),
      condition=UnlessCondition(LaunchConfiguration('amcl'))  # Condition to not launch if amcl is true
  )

  start_ros2_navigation_cmd_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart
        }.items(),
        condition=IfCondition(LaunchConfiguration('amcl'))  # Run this when 'amcl' is true
    )

  # Static transform from map to odom when AMCL is OFF
  static_tf_map_to_odom_cmd = Node(
      condition=UnlessCondition(amcl),
      package='tf2_ros',
      executable='static_transform_publisher',
      name='static_map_to_odom',
      arguments=['-2.0', '1.0', '0.0', '0.0', '0.0', '0.977902', '0.209059', 'map', 'odom'],
      output='screen'
  )


  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_amcl_cmd) #param to control amcl
  ld.add_action(declare_ekf_cmd) #param to control ekf
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
  ld.add_action(declare_joystick_cmd)
  # Add actions
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(start_imu_cmd)
  ld.add_action(start_lidar_cmd)
  ld.add_action(start_ros2_navigation_cmd) #amcl off, using the nav2 bringup - modified
  ld.add_action(start_ros2_navigation_cmd_amcl) #amcl on, using the nav2 available directory
  ld.add_action(start_odometry_cmd) #single odometry launch
  ld.add_action(start_joy_node)
  ld.add_action(static_tf_map_to_odom_cmd)
  ld.add_action(start_detection_node)
  # ld.add_action(start_detection_bridge_node)
  # ld.add_action(start_robot_localization_cmd) 
  return ld