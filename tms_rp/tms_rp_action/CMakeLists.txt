###############################
## cmake and project setting ##
###############################

cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_rp_action
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime pcl_ros
  DEPENDS eigen
)


#######################################
## choreonoid specific configuration ##
#######################################

set(ChoreonoidPath ~/choreonoid)
set(graspPluginPath ${ChoreonoidPath}/extplugin/graspPlugin)
set(rostmsPluginPath ${ChoreonoidPath}/extplugin/rostmsPlugin)
set(prmPath ${ChoreonoidPath}/extplugin/graspPlugin/PRM)
set(ChoreonoidLibPath ${ChoreonoidPath}/lib/choreonoid-1.4)

set(libname CnoidRosTmsTmsRpPlugin)


###############################
## Qt specific configuration ##
###############################

find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)
QT4_ADD_RESOURCES(RCS src/tms_rp_icons.qrc)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ChoreonoidPath}
  ${ChoreonoidPath}/include
  ${graspPluginPath}
  ${prmPath}
  /usr/include/eigen3
)

link_directories(
  ${ChoreonoidLibPath}
)

add_library(
  ${libname}
  SHARED
  ${RCS}
  src/tms_rp_plugin.cpp
  src/tms_rp_bar.cpp
  src/tms_rp_controller.cpp
  src/tms_rp_rp.cpp
  src/tms_rp_static_map.cpp
  src/tms_rp_pp.cpp
)

add_dependencies(${libname} tms_msg_rp_gencpp tms_msg_db_gencpp tms_msg_rs_gencpp tms_msg_ss_gencpp)

target_link_libraries(
  ${libname}
  CnoidBodyPlugin
  CnoidPoseSeqPlugin
  CnoidGraspPlugin
  CnoidPRMPlugin
  uuid
  dl
  pthread
  omniORB4
  omnithread
  omniDynamic4
  ${catkin_LIBRARIES}
)

set_target_properties(
  ${libname}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY 
  ${ChoreonoidLibPath}
)

add_definitions(-DCNOID_BACKWARD_COMPATIBILITY)
