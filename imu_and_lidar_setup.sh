#!/bin/bash
cd ~/imu_ws
colcon build
source install/setup.bash
cd ~/lidar_ws
colcon build
source install/setup.bash
