#!/bin/bash
ros2 run tf2_ros static_transform_publisher -2.0 1.0 0.0 0.0 0.0 0.977902 0.209059 map odom
ros2 launch basic_mobile_robot basic_mobile_bot_v6.launch.py
