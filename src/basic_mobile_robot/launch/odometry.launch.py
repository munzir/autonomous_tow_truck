from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='joyard',
            output='screen',
            parameters=[{
                'joystick': LaunchConfiguration('joystick'),
            }]
        ),
        Node(
            package='py_pubsub',
            executable='freqnangle',
            output='screen'
        ),
        Node(
            package='py_pubsub',
            executable='poser',
            output='screen'
        ),
        Node(
            package='py_pubsub',
            executable='listener',
            output='screen'
        )
    ])
