from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Configure RealSense with optimized settings
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        parameters=[{
                'enable_depth': True,
                'enable_color': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_gyro': False,
                'enable_accel': False,
                'align_depth.enable': False,  # Keep disabled for now
                'pointcloud.enable': False  # Keep disabled for now
        }],
        arguments=['--log-level', 'WARN'],
        output='screen'  # Recommended for debugging
    )

    # Static TF from camera to base
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.345', '0', '0.28', '-1.5708', '1.5708', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    return LaunchDescription([realsense_node, static_tf])
