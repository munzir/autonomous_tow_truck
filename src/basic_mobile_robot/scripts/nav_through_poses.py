#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com
 
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
  csv_filename = temp_node.declare_parameter('csv_filename', '/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/all_waypoints.csv').get_parameter_value().string_value
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
          
          # Remaining rows are goals
          for row in all_rows[1:]:
              if len(row) == 4:
                  x, y, w, z = map(float, row)
                  goal_pose = PoseStamped()
                  goal_pose.header.frame_id = 'map'
                  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                  goal_pose.pose.position.x = x
                  goal_pose.pose.position.y = y
                  goal_pose.pose.position.z = 0.0
                  goal_pose.pose.orientation.x = 0.0
                  goal_pose.pose.orientation.y = 0.0
                  goal_pose.pose.orientation.z = z
                  goal_pose.pose.orientation.w = w
                  goal_poses.append(goal_pose)
                  print(goal_pose)
      print(f'Loaded {len(goal_poses)} goal_poses from {file_path}')
  except Exception as e:
      print(f'Failed to load goal_poses: {e}')

  # sanity check a valid path exists
  # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
 
  # Go through the goal poses
  navigator.goThroughPoses(goal_poses[:])
 
  i = 0
  # Keep doing stuff as long as the robot is moving towards the goal poses
  while not navigator.isNavComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')
 
      # Some navigation timeout to demo cancellation
      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1000000.0):
        navigator.cancelNav()
 
      # Some navigation request change to demo preemption
      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500000.0):
        goal_pose_alt = PoseStamped()
        goal_pose_alt.header.frame_id = 'map'
        goal_pose_alt.header.stamp = navigator.get_clock().now().to_msg()
        # 291.1080766192939,86.93031060800634,0,1
        goal_pose_alt.pose.position.x = -6.5
        goal_pose_alt.pose.position.y = -4.2
        goal_pose_alt.pose.position.z = 0.0
        goal_pose_alt.pose.orientation.x = 0.0
        goal_pose_alt.pose.orientation.y = 0.0  
        goal_pose_alt.pose.orientation.z = 0.0
        goal_pose_alt.pose.orientation.w = 1.0
        navigator.goThroughPoses([goal_pose_alt])
 
  # Do something depending on the return code
  result = navigator.getResult()
  # if result == TaskResult.SUCCEEDED:
  #   print('Goal succeeded!')
  # elif result == TaskResult.CANCELED:
  #   print('Goal was canceled!')
  # elif result == TaskResult.FAILED:
  #   print('Goal failed!')
  # else:
  #   print('Goal has an invalid return status!')
  
  
  if result == 0:  # SUCCEEDED
    print('Goal succeeded!')
    navigator.last_reached_index += 1
  elif result == 1:  # CANCELED
    print('Goal was canceled!')
  elif result == 2:  # FAILED
    print('Goal failed!')
    # Add here which was the last waypoint executed and what is next
  else:
    print('Goal has an invalid return status!')
    if navigator.last_reached_index >= 0 and navigator.last_reached_index < len(goal_poses):
        last_pose = goal_poses[navigator.last_reached_index]
        print(f"Last successfully reached waypoint index: {navigator.last_reached_index}")
        print(f"Coordinates: x={last_pose.pose.position.x:.2f}, y={last_pose.pose.position.y:.2f}")
        if navigator.last_reached_index + 1 < len(goal_poses):
            next_pose = goal_poses[navigator.last_reached_index + 1]
            print(f"Next intended waypoint index: {navigator.last_reached_index + 1}")
            print(f"Coordinates: x={next_pose.pose.position.x:.2f}, y={next_pose.pose.position.y:.2f}")

 
  # Close the ROS 2 Navigation Stack
  # navigator.lifecycleShutdown()
 
  exit(0)
 
if __name__ == '__main__':
  main()