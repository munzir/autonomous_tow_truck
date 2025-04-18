from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():
    # RealSense node with RGB + Depth
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
            'align_depth.enable': False,
            'pointcloud.enable': False
        }],
        arguments=['--log-level', 'WARN'],
        output='screen'
    )

    # # Static TF: base_link → camera_link (with 90° CW about X and 90° CCW about Z)
    # static_tf_base_to_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=[
    #         '0.345', '0', '0.28',                # Translation: x y z
    #         str(-math.pi/2), '0', str(math.pi/2),  # Rotation: roll pitch yaw (in radians)
    #         'base_link', 'camera_link'          # Parent → Child
    #     ],
    #     output='screen'
    # )

    # # Static TF: camera_link → camera_link_optical (standard camera optical frame)
    # static_tf_camera_to_optical = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=[
    #         '0', '0', '0',
    #         str(-math.pi/2), '0', str(-math.pi/2),
    #         'camera_link', 'camera_link_optical'
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        realsense_node,
        # static_tf_base_to_camera,
        # static_tf_camera_to_optical
    ])
