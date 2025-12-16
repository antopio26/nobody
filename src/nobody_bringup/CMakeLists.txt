cmake_minimum_required(VERSION 3.8)
project(nobody_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_py REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# PCL is often found via system, but pcl_conversions/pcl_ros should handle it. 
# Explicitly finding PCL might be needed if not transitively included.
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
find_package(PCL REQUIRED)

find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(nobody_bringup_plugins plugins/toggle_layer.cpp)
ament_target_dependencies(nobody_bringup_plugins
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  rcl_interfaces
  pluginlib
)


install(TARGETS nobody_bringup_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install Python executables
install(PROGRAMS
  scripts/base_footprint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
