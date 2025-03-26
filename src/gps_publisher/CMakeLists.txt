cmake_minimum_required(VERSION 3.8)
project(gps_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(
  PROGRAMS
  gps_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (if any)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
