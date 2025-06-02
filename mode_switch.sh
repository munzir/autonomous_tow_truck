#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/autonomous_tow_truck/install/setup.bash
source ~/autonomous_tow_truck/install/local_setup.bash

# Get mode from CLI
MODE=$1

# Run mode publisher with the mode as argument
ros2 run basic_mobile_robot mode_switch.py "$MODE"
