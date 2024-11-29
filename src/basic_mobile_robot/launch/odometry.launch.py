from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    py_pubsub_share = FindPackageShare('py_pubsub').find('py_pubsub')

    if not py_pubsub_share:
        raise ValueError("Package 'pubsub_driver' not found in the workspace!")

    # Define the nodes to launch
    freqnangle_node = Node(
        package='py_pubsub',
        executable='freqnangle',
        name='freqnangle',
        output='screen'
    )

    poser_node = Node(
        package='py_pubsub',
        executable='poser',
        name='poser',
        output='screen'
    )

    listener_node = Node(
        package='py_pubsub',
        executable='listener',
        name='listener',
        output='screen'
    )

    # Add nodes to the launch description
    ld = LaunchDescription()
    ld.add_action(freqnangle_node)
    ld.add_action(poser_node)
    ld.add_action(listener_node)

    return ld
