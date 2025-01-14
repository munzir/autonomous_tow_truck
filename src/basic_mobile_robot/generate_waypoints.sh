#!/bin/bash

# Hardcoded waypoints 
WAYPOINT_S1a="188.95095825195312,73.09672546386719,0.4627738816895823,0.8864763586390539"
WAYPOINT2_S1b="184.30267333984375,81.26746368408203,0.5170994204068351,0.8559253410285941"
WAYPOINT_FIXED="41.125274658203125,21.119226455688477,0.165393089159818,-0.986227725253236"
WAYPOINT_S2="31.453933715820312,21.137266159057617,0.5826882902241809,0.8126957342299889"


# Output CSV file to our waypoints location (can change it according to our requirements)
OUTPUT_FILE="$HOME/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv"

echo "x,y,w,z" > "$OUTPUT_FILE" 

#straight 1
python3 compute_waypoints intial_pose_straight1 >> "$OUTPUT_FILE"
#S1 a and b
echo "$WAYPOINT_S1a" >> "$OUTPUT_FILE"
echo "$WAYPOINT_S1b" >> "$OUTPUT_FILE"
#all subparts of straight 2
python3 compute_waypoints intial_pose_straight2i >> "$OUTPUT_FILE"
python3 compute_waypoints intial_pose_straight2a >> "$OUTPUT_FILE"
python3 compute_waypoints intial_pose_straight2b >> "$OUTPUT_FILE"
echo "$WAYPOINT_FIXED" >> "$OUTPUT_FILE"
#S2
echo "$WAYPOINT_S2" >> "$OUTPUT_FILE"
#straight 3
python3 compute_waypoints intial_pose_straight3  >> "$OUTPUT_FILE"


echo "Waypoints have been generated and saved to $OUTPUT_FILE."

