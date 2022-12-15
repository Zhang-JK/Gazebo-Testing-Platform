cmake_minimum_required(VERSION 3.0.2)
project(simulation_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voice_recognition
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/odom2tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
