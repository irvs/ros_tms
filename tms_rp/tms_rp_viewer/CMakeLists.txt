## cmake and project setting
cmake_minimum_required(VERSION 2.8.3)
project(tms_rp_viewer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  sensor_msgs
  pcl_ros
)

## catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tms_rp_viewer
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime pcl_ros
  DEPENDS eigen
)

## choreonoid specific configuration 
set(ChoreonoidPath ~/choreonoid)
set(graspPluginPath ${ChoreonoidPath}/extplugin/graspPlugin)
set(prmPath ${ChoreonoidPath}/extplugin/graspPlugin/PRM)
set(ChoreonoidLibPath ${ChoreonoidPath}/lib/choreonoid-1.4)
set(rpActionPluginPath ~/catkin_ws/src/ros_tms/tms_rp/tms_rp_action)

set(libname CnoidRosTmsRpViewerPlugin)

## Qt specific configuration 
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_OPENGL_INCLUDE_DIR})
add_definitions(-DQT_NO_KEYWORDS)
QT4_ADD_RESOURCES(RCS src/tms_rp_viewer_icons.qrc)

## Build option
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ChoreonoidPath}
  ${ChoreonoidPath}/include
  ${graspPluginPath}
  ${rpActionPluginPath}/include
  ${prmPath}
  #${Eigen_INCLUDE_DIRS}
  /usr/include/eigen3
)

link_directories(
  ${ChoreonoidLibPath}
)

add_library(
  ${libname}
  SHARED
  #${RCS}
  ${rpActionPluginPath}/src/tms_rp_controller.cpp
  src/tms_rp_viewer_bar.cpp
  src/tms_rp_viewer_plugin.cpp
)

add_dependencies(${libname} tms_msg_db_gencpp tms_msg_ss_gencpp tms_msg_rp_gencpp)

target_link_libraries(
  ${libname}
  CnoidBodyPlugin
  CnoidPoseSeqPlugin
  CnoidGraspPlugin
  CnoidPRMPlugin
  ${catkin_LIBRARIES}
)

set_target_properties(
  ${libname}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY 
  ${ChoreonoidLibPath}
)

add_definitions(-DCNOID_BACKWARD_COMPATIBILITY)
