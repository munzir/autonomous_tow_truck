#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/autonomous_tow_truck/install/setup.bash
source ~/autonomous_tow_truck/install/local_setup.bash

# Launch the ROS 2 script
ros2 launch basic_mobile_robot outdoor_localization.launch.py