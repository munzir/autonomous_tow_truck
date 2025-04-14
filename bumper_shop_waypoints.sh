#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source ~/autonomous_tow_truck/install/setup.bash
source ~/autonomous_tow_truck/install/local_setup.bash

# Run waypoint publisher node with Bumper Shop CSV
ros2 run waypoint_publisher waypoint_publisher_node --ros-args -p csv_filename:=/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints_from_AS.csv &

# Run the nav_through_poses script
ros2 run basic_mobile_robot nav_through_poses.py

