cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_mimamorukun_virtual_control)

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
 add_executable(mimamorukun_virtual_control src/mimamorukun_virtual_control.cpp)

 add_dependencies(mimamorukun_virtual_control DEPENDS tms_msg_rc_gencpp tms_msg_db_gencpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(mimamorukun_virtual_control
   ${catkin_LIBRARIES}
 )
