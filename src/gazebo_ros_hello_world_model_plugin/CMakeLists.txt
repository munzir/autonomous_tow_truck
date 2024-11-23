cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_hello_world_model_plugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add the plugin library
add_library(${PROJECT_NAME} SHARED src/gazebo_ros_hello_world_model_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  gazebo_ros
)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install other resources
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Export the package
ament_package()

