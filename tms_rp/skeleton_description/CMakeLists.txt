cmake_minimum_required(VERSION 2.8.3)
project(skeleton_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  robot_state_publisher
)
find_package(Eigen REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    calc_joint_angles
  CATKIN_DEPENDS
    roscpp
    rospy
    cmake_modules
    #tms_msg_ss
)

###################
## Build         ##
###################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ./calc_joint_angles
  )

add_executable(skeleton_state_publisher skeleton_state_publisher/main.cpp)
target_link_libraries(skeleton_state_publisher ${catkin_LIBRARIES})
add_dependencies(skeleton_state_publisher tms_ss_kinect_v2_generate_messages_cpp)

add_executable(kinect_state_publisher kinect_state_publisher/main.cpp)
target_link_libraries(kinect_state_publisher ${catkin_LIBRARIES})
add_dependencies(kinect_state_publisher tms_ss_kinect_v2_generate_messages_cpp)

add_executable(box_state_publisher box_state_publisher/main.cpp)
target_link_libraries(box_state_publisher ${catkin_LIBRARIES})
add_dependencies(box_state_publisher tms_ss_kinect_v2_generate_messages_cpp)
