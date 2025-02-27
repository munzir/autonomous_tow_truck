#!/bin/bash

WAYPOINT_S1a="225.80519104003906, -11.303391456604004,0.7286353867072394, 0.6849017982440926"
WAYPOINT_S1b="225.82989501953125, -1.825505018234253,0.7080276184846394, 0.7061847431536382"
#WAYPOINT_FIXED="41.125274658203125,21.119226455688477,0.165393089159818,-0.986227725253236"
WAYPOINT_S2="58.43534469604492,5.011921405792236,0.704090605550395, 0.7101101458053377"
WAYOINT_U="14.5093994140625,6.217558860778809,0.7052711969471163,-0.7089376127395005"
WAYPOINT_S3="58.107887268066406, 4.509756565093994,0.7219366933243214,-0.6919591106647449"
WAYPOINT_S4a="108.11625671386719,-1.5209693908691406,0.7167262270619986,-0.6973546554239617"
WAYPOINT_S4b="108.37663269042969,-11.022077560424805,0.7055722799214855,-0.7086379596143555"
OUTPUT_FILE="$HOME/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv"

# Clear the output file before appending
> "$OUTPUT_FILE"

# Append output from Python scripts and echo commands to the output file
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight1a.txt >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight1b.txt >> "$OUTPUT_FILE"
echo "$WAYPOINT_S1a" >> "$OUTPUT_FILE"
echo "$WAYPOINT_S1b" >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight2a.txt >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight2b.txt >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight2c.txt >> "$OUTPUT_FILE"
#echo "$WAYPOINT_FIXED" >> "$OUTPUT_FILE"
echo "$WAYPOINT_S2" >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight3.txt >> "$OUTPUT_FILE"
echo "$WAYPOINT_U" >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight4.txt >> "$OUTPUT_FILE"
echo "$WAYPOINT_S3" >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight5.txt >> "$OUTPUT_FILE"
echo "$WAYPOINT_S4a" >> "$OUTPUT_FILE"
echo "$WAYPOINT_S4b" >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight6a.txt >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight6b.txt >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py initial_pose_straight6c.txt >> "$OUTPUT_FILE"
echo "$WAYPOINT_S5" >> "$OUTPUT_FILE"
echo "Waypoints have been generated and saved to $OUTPUT_FILE."

