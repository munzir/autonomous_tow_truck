#!/bin/bash

WP_RAMP="296.50634765625, 87.04972076416016, 0.0197379409416775, 0.999805187867808"
WP_DIAG="60.32508850097656,  102.2496566772461, 0.34310736399037384,  0.9392961922501215"
WP_U1="12.751440048217773, 107.90078735351562,  0.32420940157530426, -0.9459853402300603"
WP_U2="12.61766242980957,  108.1629638671875, 0.7152279231204859, -0.6988912776596631"
WP_U3="12.451135635375977, 107.06465911865234, 0.9343952424402237, -0.3562380256304422"
WP_U4="12.981222152709961, 106.63021850585938, 0.9999993351995934,  0.0011530829853807236"

OUTPUT_FILE="$HOME/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv"

# Clear the output file before appending
> "$OUTPUT_FILE"

# Append output from Python scripts and echo commands to the output file
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP1.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP2.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP3.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP4.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP5.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP6.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP7.TXT >> "$OUTPUT_FILE"
echo "$wP_RAMP" >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP8.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP9.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP10.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP11.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP12.TXT >> "$OUTPUT_FILE"
python3 /root/autonomous_tow_truck/src/basic_mobile_robot/generate_waypoints/compute_waypoints.py IP13.TXT >> "$OUTPUT_FILE"
echo "$wP_DIAG" >> "$OUTPUT_FILE"
echo "$wP_U1" >> "$OUTPUT_FILE"

echo "Waypoints have been generated and saved to $OUTPUT_FILE."
