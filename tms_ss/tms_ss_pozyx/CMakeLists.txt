cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_pozyx)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tms_msg_db
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tms_ss_pozyx
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tms_ss_pozyx src/main.cpp)

add_dependencies(tms_ss_pozyx tms_msg_db_generate_messages_cpp)

target_link_libraries(tms_ss_pozyx
	${catkin_LIBRARIES}
	)
