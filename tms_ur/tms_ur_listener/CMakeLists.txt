cmake_minimum_required(VERSION 2.8.3)
project(tms_ur_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES tms_ur_listener
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
