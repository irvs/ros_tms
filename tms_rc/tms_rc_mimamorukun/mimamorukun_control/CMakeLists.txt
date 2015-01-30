cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_mimamorukun_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kobuki_virtual_control
  CATKIN_DEPENDS geometry_msgs roscpp tms_msg_db tms_msg_rc
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(mimamorukun_manual_control src/mimamorukun_manual_control.cpp src/ClientSocket.cpp src/Socket.cpp)
 add_executable(mimamorukun_automatic_control src/mimamorukun_automatic_control.cpp src/ClientSocket.cpp src/Socket.cpp)

 add_dependencies(mimamorukun_manual_control DEPENDS tms_msg_rc_gencpp tms_msg_db_gencpp)
 add_dependencies(mimamorukun_automatic_control DEPENDS tms_msg_rc_gencpp tms_msg_db_gencpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(mimamorukun_manual_control
   ${catkin_LIBRARIES}
 )
 target_link_libraries(mimamorukun_automatic_control
   ${catkin_LIBRARIES}
 )
