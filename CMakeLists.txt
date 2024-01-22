cmake_minimum_required(VERSION 3.5)
project(assignment3)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
scripts/a3_referee.py
scripts/waypoint_follower.py
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
models
rviz
worlds
maps
DESTINATION share/${PROJECT_NAME}/)



ament_package()
