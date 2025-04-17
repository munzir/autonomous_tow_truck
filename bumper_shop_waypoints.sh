#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/autonomous_tow_truck/install/setup.bash
source ~/autonomous_tow_truck/install/local_setup.bash

# Kill old path nodes
pkill -f nav_through_poses.py

CSV_FILE="/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints_from_AS.csv"

ros2 run waypoint_publisher waypoint_publisher_node --ros-args -p csv_filename:=${CSV_FILE} &
sleep 1

ros2 run basic_mobile_robot nav_through_poses.py --ros-args -p csv_filename:=${CSV_FILE}