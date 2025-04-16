#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source ~/autonomous_tow_truck/install/setup.bash
source ~/autonomous_tow_truck/install/local_setup.bash

# Define the CSV file path for Bumper Shop (from Assembly Shop)
CSV_FILE="/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints_from_AS.csv"

# Run waypoint publisher node with the given CSV
ros2 run waypoint_publisher waypoint_publisher_node --ros-args -p csv_filename:=${CSV_FILE} &

# Run the nav_through_poses script with the same CSV
ros2 run basic_mobile_robot nav_through_poses.py --ros-args -p csv_filename:=${CSV_FILE}
