from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RealSense D435 camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_rgb': True,
                'publish_tf': True,
                'pointcloud.enable': False
            }]
        ),

        # AprilTag detection node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[{
                'image_transport': 'raw',
                'camera_frame': 'camera_color_optical_frame',
                'publish_tag_tf': True,
                'tag_family': 'tag36h11',
                'size': 0.05,  # Tag size in meters (adjust accordingly)
                'use_sim_time': False
            }],
            remappings=[
                ('/image_rect', '/camera/color/image_raw'),
                ('/camera_info', '/camera/color/camera_info')
            ]
        ),

        # Static TF broadcaster from YAML file (tag positions)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_broadcaster',
            parameters=['/home/zain/Desktop/april_tag_ws/src/april_tag_setup/config/tag_transforms.yaml'],
            output='screen'
        )
    ])