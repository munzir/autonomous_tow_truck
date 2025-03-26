from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Change if using simulation
                'publish_filtered_gps': True,
                'broadcast_cartesian_transform': True,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0
            }],
            remappings=[
                ('/odometry/gps', '/odometry/gps'),  # Ensure proper remap
                ('/gps/fix', '/gps/fix'),
                ('/imu', '/imu')
            ]
        )
    ])

