cmake_minimum_required(VERSION 3.8)
project(yolo_costmap_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
  ${nav2_costmap_2d_INCLUDE_DIRS}
)

# Create the library
add_library(yolo_layer
  src/yolo_layer.cpp
  include/yolo_costmap_plugin/yolo_layer.hpp
)

# Link libraries
target_link_libraries(yolo_layer
  ${nav2_costmap_2d_LIBRARIES}
  pluginlib::pluginlib
)

# Install targets
install(TARGETS yolo_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(yolo_layer)

ament_package()
