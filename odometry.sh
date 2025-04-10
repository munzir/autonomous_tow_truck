#!/bin/bash

ros2 run py_pubsub poser &      # Run poser in background
ros2 run py_pubsub listener &   # Run listener in background
ros2 run py_pubsub freqnangle & # Run freqnangle in background

wait  # Wait for all to finish (keeps script alive while nodes run)

