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
  visualization_msgs
  cv_bridge
  )

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tms_ss_pot
    CATKIN_DEPENDS message_runtime roscpp std_msgs
    DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  /usr/local/include/opencv2
  /usr/local/include/opencv
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(tms_ss_pot
    include/target.cpp
    include/laser.cpp
    include/particle_filter.cpp
    include/multiple_particle_filter.cpp
 )

add_executable(urg_tracker src/urg_tracker.cpp include/target.cpp include/laser.cpp include/particle_filter.cpp include/multiple_particle_filter.cpp)
add_executable(urg_single_tracker src/urg_single_tracker.cpp include/target.cpp include/laser.cpp include/particle_filter.cpp include/multiple_particle_filter.cpp)
add_executable(change_nan src/change_nan.cpp)
add_executable(visualize src/tracker_visualization.cpp)
add_executable(laser_visualize src/laser_visualization.cpp)
add_executable(laser_visualize_multi src/laser_visualization_multi.cpp)

add_dependencies(urg_tracker tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(urg_single_tracker tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(change_nan tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(visualize tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(laser_visualize tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)
add_dependencies(laser_visualize_multi tms_ss_pot_gencpp tms_msg_ss_gencpp tms_msg_db_gencpp tms_msg_rp_gencpp)

target_link_libraries(urg_tracker ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(urg_single_tracker ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(change_nan ${catkin_LIBRARIES})
target_link_libraries(visualize ${catkin_LIBRARIES})
target_link_libraries(laser_visualize ${catkin_LIBRARIES})
target_link_libraries(laser_visualize_multi ${catkin_LIBRARIES})
