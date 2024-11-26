from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the share directory for the `umx_driver` package
    umx_driver_share = FindPackageShare('umx_driver').find('umx_driver')

    if not umx_driver_share:
        raise ValueError("Package 'umx_driver' not found in the workspace!")

    return LaunchDescription([
        # Arguments to configure the IMU driver
        DeclareLaunchArgument(
            name='serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the UM7 sensor'
        ),
        DeclareLaunchArgument(
            name='baudrate',
            default_value='115200',
            description='Baudrate for the UM7 communication'
        ),
        DeclareLaunchArgument(
            name='frame_id',
            default_value='imu_link',
            description='Frame ID for the UM7 IMU data'
        ),

        # Node to launch the UM7 driver
        Node(
            package='umx_driver',  
            executable='um7_driver',  
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
            }]
        )
    ])
