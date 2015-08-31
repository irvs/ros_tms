cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_kinect_v2)

## If you will use choreonoid, Switch to ON
set(USING_Choreonoid_AND_Grasp_Plugin OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  visualization_msgs
  message_generation
  cmake_modules
)
find_package(Eigen REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
    Skeleton.msg
    SkeletonArray.msg
    CameraPosture.msg
    SkeletonStreamWrapper.msg
    pose_setting.msg
)

add_service_files(
  FILES
    ConvertToJointAngles.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    message_runtime
    geometry_msgs
    std_msgs
    visualization_msgs
    cmake_modules
)

if (${USING_Choreonoid_AND_Grasp_Plugin})
  #######################################
  ## Choreonoid specific configuration ##
  #######################################
  set(Choreonoid_ROOT ~/choreonoid)
  set(GraspPlugin_PATH ${Choreonoid_ROOT}/extplugin/graspPlugin)
  set(PRM_PATH ${GraspPlugin_PATH}/PRM)
  set(Choreonoid_LIB_DIR ${Choreonoid_ROOT}/lib/choreonoid-1.4)
  set(cnoid_plugin CnoidPoseSetPlugin)
  ###############################
  ## Qt specific configuration ##
  ###############################
  find_package(Qt4 REQUIRED)
  set(QT_USE_QTOPENGL TRUE)
  include(${QT_USE_FILE})
  add_definitions(-DQT_NO_KEYWORDS)
endif (${USING_Choreonoid_AND_Grasp_Plugin})

###################
## Build         ##
###################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Choreonoid_ROOT}
  ${Choreonoid_ROOT}/include
  ${QT_INCLUDE_DIR}
  ${QT_OPENGL_INCLUDE_DIR}
  ${GraspPlugin_PATH}
  ${PRM_PATH}
  ./calc_joint_angles
  )
link_directories(
  ${Choreonoid_LIB_DIR}
  )

add_executable(skeleton_viewer skeleton_viewer/main.cpp)
target_link_libraries(skeleton_viewer ${catkin_LIBRARIES})
add_dependencies(skeleton_viewer tms_ss_kinect_v2_generate_messages_cpp)

add_executable(calc_joint_angles calc_joint_angles/main.cpp)
target_link_libraries(calc_joint_angles ${catkin_LIBRARIES})
add_dependencies(calc_joint_angles tms_ss_kinect_v2_generate_messages_cpp)

add_executable(skeleton_integrator skeleton_integrator/main.cpp)
target_link_libraries(skeleton_integrator ${catkin_LIBRARIES})
add_dependencies(skeleton_integrator tms_ss_kinect_v2_generate_messages_cpp)

if (${USING_Choreonoid_AND_Grasp_Plugin})
  add_library(
    ${cnoid_plugin}
    SHARED
    calc_joint_angles/skeleton_pose_set_plugin.cpp
    calc_joint_angles/skeleton_pose_set_bar.cpp
    )
  target_link_libraries(
    ${cnoid_plugin}
    CnoidBodyPlugin
    CnoidPoseSeqPlugin
    CnoidGraspPlugin
    CnoidPRMPlugin
    ${catkin_LIBRARIES}
    )
  add_dependencies(${cnoid_plugin} tms_ss_kinect_v2_generate_messages_cpp)
  set_target_properties(
    ${cnoid_plugin}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    ${Choreonoid_LIB_DIR}
    )
  add_definitions(-DCNOID_10_11_12_13_14)
  add_definitions(-DCNOID_BACKWARD_COMPATIBILITY)
endif (${USING_Choreonoid_AND_Grasp_Plugin})
