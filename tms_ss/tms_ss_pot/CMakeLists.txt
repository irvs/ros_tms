cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_pot)

###################################
## packege  find   configuration ##
###################################

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  sensor_msgs
  tms_msg_ss
  tms_msg_db
  message_generation
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES tms_ss_pot
   CATKIN_DEPENDS message_runtime roscpp std_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pot_ctrl src/pot.cpp)
add_executable(lsr_ctrl src/LaserTracker.cpp)

add_dependencies(pot_ctrl tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(lsr_ctrl tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)


target_link_libraries(pot_ctrl   ${catkin_LIBRARIES})
target_link_libraries(lsr_ctrl   ${catkin_LIBRARIES})
