cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_rostms_plugin)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnoid_planner_interface
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)

#######################################
## choreonoid specific configuration ##
#######################################
set(choreonoidPath    ~/choreonoid)
set(choreonoidLibPath ${choreonoidPath}/lib/choreonoid-1.4)
set(rostmsPluginPath  ~/catkin_ws/src/ros_tms/tms_rp/tms_rp_rostms_plugin/src)

set(graspPluginPath ${choreonoidPath}/extplugin/graspPlugin)
set(prmPath         ${choreonoidPath}/extplugin/graspPlugin/PRM)

set(cnoidPlannerROSInterfacePath  ${rostmsPluginPath}/libCnoidPlannerROSInterface)
set(voronoiPlannerPath            ${rostmsPluginPath}/libVoronoiPathPlanner)
set(giveObjPlannerPath            ${rostmsPluginPath}/libGiveObjToHumanPlanner)
set(wagonPlannerPath              ${rostmsPluginPath}/libWagonPlanner)

set(libCnoidPlannerROSInterface CnoidRosTmsPlannerROSInterfacePlugin)
set(libVoronoiPathPlanner       CnoidRosTmsVoronoiPathPlannerPlugin)
set(libGiveObjToHumanPlanner    CnoidRosTmsGiveObjToHumanPlannerPlugin)
set(libWagonPlanner             CnoidRosTmsWagonPlannerPlugin)

###############################
## Qt specific configuration ##
###############################
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${choreonoidPath}
  ${choreonoidPath}/include
  ${graspPluginPath}
  ${prmPath}
  ${rostmsPluginPath}
  ${cnoidPlannerROSInterfacePath}
  ${voronoiPlannerPath}
  ${giveObjPlannerPath}
  ${wagonPlannerPath}
  /usr/include/eigen3
)

link_directories(
  ${choreonoidLibPath}
)

# add_library(
#   ${libCnoidPlannerROSInterface}
#   SHARED
#   src/libCnoidPlannerROSInterface/cnoid_planner_interface_plugin.cpp
#   src/libCnoidPlannerROSInterface/cnoid_planner_interface_bar.cpp
#   src/libCnoidPlannerROSInterface/cnoid_planner_controller.cpp
# )

#add_library(
#  ${libVoronoiPathPlanner}
#  SHARED
#  src/libVoronoiPathPlanner/VoronoiPathPlannerPlugin.cpp
#  src/libVoronoiPathPlanner/VoronoiPathPlanner.cpp
#  src/libVoronoiPathPlanner/VoronoiPathPlannerBar.cpp
#)

#add_library(
#  ${libGiveObjToHumanPlanner}
#  SHARED
#  src/libGiveObjToHumanPlanner/HumanPlugin.cpp
#  src/libGiveObjToHumanPlanner/HumanPlanner.cpp
#  src/libGiveObjToHumanPlanner/GiveObjToHumanPlannerBar.cpp
#  src/libGiveObjToHumanPlanner/GiveObjToHumanPlanner.cpp
#)

# add_library(
#   ${libWagonPlanner}
#   SHARED
#   src/libWagonPlanner/WagonPlugin.cpp
#   src/libWagonPlanner/WagonPlanner.cpp
#   src/libWagonPlanner/WagonPushPlanner.cpp
#   src/libWagonPlanner/WagonBar.cpp
#   src/libWagonPlanner/WagonTrajectoryBar.cpp
#   src/libWagonPlanner/WagonTrajectoryPlanner.cpp
# )

#add_dependencies(${libCnoidPlannerROSInterface} tms_msg_db_gencpp tms_msg_rp_gencpp)
#add_dependencies(${libVoronoiPathPlanner}       tms_msg_db_gencpp tms_msg_rp_gencpp)
#add_dependencies(${libGiveObjToHumanPlanner}    tms_msg_db_gencpp tms_msg_rp_gencpp)
#add_dependencies(${libWagonPlanner}             tms_msg_db_gencpp tms_msg_rp_gencpp)

# target_link_libraries(
#   ${libCnoidPlannerROSInterface}
#   CnoidBodyPlugin
#   CnoidPoseSeqPlugin
#   CnoidGraspPlugin
#   CnoidPRMPlugin
#   #CnoidWagonPlannerPlugin
#   #CnoidVoronoiPathPlannerPlugin
#   #CnoidGiveObjToHumanPlannerPlugin
#   uuid
#   dl
#   pthread
#   omniORB4
#   omnithread
#   omniDynamic4
#   ${catkin_LIBRARIES}
# )

#target_link_libraries(
#  ${libVoronoiPathPlanner}
#  CnoidBodyPlugin
#  CnoidPoseSeqPlugin
#  CnoidGraspPlugin
#  CnoidPRMPlugin
#  uuid
#  dl
#  pthread
#  omniORB4
#  omnithread
#  omniDynamic4
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(
#  ${libGiveObjToHumanPlanner}
#  CnoidBodyPlugin
#  CnoidPoseSeqPlugin
#  CnoidGraspPlugin
#  CnoidPRMPlugin
#  uuid
#  dl
#  pthread
#  omniORB4
#  omnithread
#  omniDynamic4
#  ${catkin_LIBRARIES}
#)

# target_link_libraries(
#   ${libWagonPlanner}
#   CnoidBodyPlugin
#   CnoidPoseSeqPlugin
#   CnoidGraspPlugin
#   CnoidPRMPlugin
#   uuid
#   dl
#   pthread
#   omniORB4
#   omnithread
#   omniDynamic4
#   ${catkin_LIBRARIES}
# )

# set_target_properties(
#   ${libCnoidPlannerROSInterface}
#   PROPERTIES LIBRARY_OUTPUT_DIRECTORY 
#   ${choreonoidLibPath}
# )

#set_target_properties(
#  ${libVoronoiPathPlanner}
#  PROPERTIES LIBRARY_OUTPUT_DIRECTORY
#  ${choreonoidLibPath}
#)

#set_target_properties(
#  ${libGiveObjToHumanPlanner}
#  PROPERTIES LIBRARY_OUTPUT_DIRECTORY
#  ${choreonoidLibPath}
#)

# set_target_properties(
#   ${libWagonPlanner}
#   PROPERTIES LIBRARY_OUTPUT_DIRECTORY 
#   ${choreonoidLibPath}
# )

add_definitions(-DCNOID_BACKWARD_COMPATIBILITY)

