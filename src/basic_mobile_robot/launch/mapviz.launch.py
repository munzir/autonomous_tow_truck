from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('basic_mobile_robot')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mapviz_config',
                default_value=PathJoinSubstitution([current_pkg, 'config', 'gps_wpf_demo.mvc']),
                description='Mapviz config file'
            ),
            Node(
                package='mapviz',
                executable='mapviz',
                name='mapviz',
                output='screen',
                parameters=[
                    {'config': LaunchConfiguration('mapviz_config')}
                ],
            ),
            Node(
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                output="screen",
                remappings=[("fix", "/gps/fix")]
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_base_link_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
            )
        ]
    )
