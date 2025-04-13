#!/bin/bash

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/autonomous_tow_truck/install/setup.bash
source ~/autonomous_tow_truck/install/local_setup.bash


# Launch the main file
ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py

