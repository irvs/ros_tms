cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_kinect_v2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  skeleton_description
)
find_package(Eigen REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    cmake_modules
)

###################
## Build         ##
###################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_executable(skeleton_viewer skeleton_viewer/main.cpp)
target_link_libraries(skeleton_viewer ${catkin_LIBRARIES})
add_dependencies(skeleton_viewer tms_msg_ss_generate_messages_cpp)

add_executable(skeleton_integrator skeleton_integrator/main.cpp)
target_link_libraries(skeleton_integrator ${catkin_LIBRARIES})
add_dependencies(skeleton_integrator tms_msg_ss_generate_messages_cpp)
