cmake_minimum_required(VERSION 3.8)
project(load_precomputed_path)

# Add compile options for GCC/Clang if needed
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}:/root/autonomous_tow_truck/install")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add the following link_directories command if the issue persists.
link_directories(${YAML_CPP_LIBRARY_DIRS})

# Add library for the custom LoadPrecomputedPath node
add_library(${PROJECT_NAME}_bt_nodes SHARED src/load_precomputed_path.cpp)

# Link dependencies to the library
target_compile_options(${PROJECT_NAME}_bt_nodes PRIVATE -fPIC)
target_link_libraries(${PROJECT_NAME}_bt_nodes
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_bt_nodes
  rclcpp
  nav_msgs
  geometry_msgs
  behaviortree_cpp_v3
  yaml-cpp
)

# Add executable for the load_precomputed_path_node
add_executable(load_precomputed_path_node src/load_precomputed_path.cpp)

# Link dependencies and your custom library to the executable
target_link_libraries(load_precomputed_path_node
  ${PROJECT_NAME}_bt_nodes
  rclcpp
  nav_msgs
  geometry_msgs
  behaviortree_cpp_v3
  yaml-cpp
)

ament_target_dependencies(load_precomputed_path_node
  rclcpp
  nav_msgs
  geometry_msgs
  behaviortree_cpp_v3
  yaml-cpp
)

# Install the library and executable
install(TARGETS load_precomputed_path_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_bt_nodes
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Export environment hooks to set LD_LIBRARY_PATH
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

# Install include directories if needed
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# If building tests, include the following section
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

