#!/usr/bin/env python3

import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
import csv

from robot_navigator import BasicNavigator, NavigationResult
 
'''
Navigates a robot through goal poses.
'''
def main():
 
    # Start the ROS 2 Python Client Library
    rclpy.init()
    
    
    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()
    temp_node = rclpy.create_node('nav_through_poses_param_node')
    csv_filename = temp_node.declare_parameter('csv_filename', '/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv').get_parameter_value().string_value
    temp_node.destroy_node()
    file_path = csv_filename
    goal_poses = []
    try:
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            all_rows = list(reader)
            
            # First row is initial pose
            if len(all_rows[0]) == 4:
                x, y, w, z = map(float, all_rows[0])
                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = x
                initial_pose.pose.position.y = y
                initial_pose.pose.position.z = 0.0
                initial_pose.pose.orientation.x = 0.0
                initial_pose.pose.orientation.y = 0.0
                initial_pose.pose.orientation.z = z
                initial_pose.pose.orientation.w = w
                navigator.setInitialPose(initial_pose)
                # navigator.waitUntilNav2Active()

                # # ✅ Give AMCL some time to localize
                # print("Waiting for localization...")
                # navigator.waitForInitialPose(timeout=Duration(seconds=5))
                navigator.lifecycleStartup()
                navigator.waitUntilNav2Active() 
    except Exception as e:
        print(f'Failed to load goal_poses: {e}')


    exit(0)

if __name__ == '__main__':
  main()
    