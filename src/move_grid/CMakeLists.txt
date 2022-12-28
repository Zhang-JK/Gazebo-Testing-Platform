cmake_minimum_required(VERSION 3.0.2)
project(move_grid)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  GridCmd.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES move_grid
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs message_runtime 
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/move_grid
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
