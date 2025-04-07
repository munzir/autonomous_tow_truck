from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.345', '0', '0.28', '0', '0', '0', 'base_link', 'camera_link']
    )

    return LaunchDescription([realsense_node, static_tf])