cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_tracker)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  visualization_msgs
  cv_bridge
)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES tms_ss_tracker
#  DEPENDS system_lib eigen opencv
   CATKIN_DEPENDS roscpp message_runtime tms_msg_ss
)

include_directories(
  include
  /usr/local/include/opencv2
  /usr/local/include/opencv
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
)

add_library(tms_ss_tracker
   src/target.cpp
   src/laser.cpp
   src/multiple_particle_filter.cpp
   src/particle_filter.cpp
 )

add_executable(tracker src/tracker.cpp src/target.cpp src/laser.cpp src/multiple_particle_filter.cpp src/particle_filter.cpp)

add_dependencies(tracker DEPENDS tms_msg_ss_gencpp)

target_link_libraries(tracker ${catkin_LIBRARIES})

