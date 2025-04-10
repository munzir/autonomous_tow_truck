#!/bin/bash

ros2 run py_pubsub joyard &       # Runs your custom node
ros2 run joy joy_node &           # Runs the standard ROS2 joy node

wait  # Keeps script running while both nodes are active
