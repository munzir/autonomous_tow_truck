cmake_minimum_required(VERSION 3.5)
project(object_detection_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Python support

# Install Python nodes
install(
  PROGRAMS
  object_detection_pkg/yolo_detection_node.py
  object_detection_pkg/obstacle_processing_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, and other resources
install(
  DIRECTORY config launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
