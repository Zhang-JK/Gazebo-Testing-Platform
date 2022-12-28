cmake_minimum_required(VERSION 3.0.2)
project(move_grid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_grid
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/move_grid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
