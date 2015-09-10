cmake_minimum_required(VERSION 2.8.3)
project(tms_ts_subtask)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tms_msg_db
  tms_msg_ts
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tms_ts_subtask
  CATKIN_DEPENDS roscpp std_msgs tms_msg_db tms_msg_ts
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
