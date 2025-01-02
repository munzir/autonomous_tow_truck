#!/bin/bash
colcon build
source install/local_setup.bash
source install/setup.bash
ros2 run py_pubsub joyard
ros2 run joy joy_node
ros2 run py_pubsub poser
ros2 run py_pubsub listener
ros2 run py_pubsub freqnangle
