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
find_package(PCL REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# C++ Node
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
ament_target_dependencies(pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

# Install Python executables
install(PROGRAMS
  scripts/pointcloud_filter_node.py
  scripts/base_footprint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables
install(TARGETS
  pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
