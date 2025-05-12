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
            'pointcloud.stream_filter': 2,  # For better point density
            'depth_module.profile': '640x480x30',  # Optimal resolution
            # 'frameset': 'camera_depth_frame',  # Critical for TF consistency
            # Frame IDs must match your TF tree
            'base_frame_id': 'camera_link',
            'depth_frame_id': 'camera_depth__optical_frame',
            'infra_frame_id': 'camera_infra_frame',
            'color_frame_id': 'camera_color_frame',
            # Optional performance tuning
            'depth_module.enable_auto_exposure': True,
            'pointcloud.ordered_pc': False
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.345', '0', '0.28', '0', '0', '0', 'base_link', 'camera_link']
    )
    optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_depth_optical_frame']
    )

    return LaunchDescription([realsense_node, static_tf, optical_tf])