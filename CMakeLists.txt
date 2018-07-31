cmake_minimum_required(VERSION 2.8.3)
project(tms_nw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#add_message_files(
#  FILES
#)

#add_service_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES tms_nw
   CATKIN_DEPENDS roscpp rospy std_msgs message_generation
#  DEPENDS system_lib
)
