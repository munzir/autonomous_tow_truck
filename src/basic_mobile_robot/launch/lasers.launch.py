# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='frame_id',
            default_value='laser',
            description='Frame ID for the RPLIDAR'
        ),
        DeclareLaunchArgument(
            name='serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the RPLIDAR'
        ),
        DeclareLaunchArgument(
            name='serial_baudrate',
            default_value='115200',
            description='Baudrate for the RPLIDAR communication'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2').find('sllidar_ros2'),
                    'launch',
                    'sllidar_a3_launch.py'  
                ])
            ),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate')
            }.items(),
        ),
    ])
