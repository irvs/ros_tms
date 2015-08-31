cmake_minimum_required(VERSION 2.8.3)
project(mongodb_log)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED rospy roslib rosgraph rostopic tf sensor_msgs mongodb_store)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread filesystem)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mongodb_log
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(mongodb_log
#   src/${PROJECT_NAME}/mongodb_log.cpp
# )

#link_libraries(mongoclient)
add_executable(mongodb_log_tf src/mongodb_log_tf.cpp)
add_executable(mongodb_log_pcl src/mongodb_log_pcl.cpp)
add_executable(mongodb_log_cimg src/mongodb_log_cimg.cpp)

# rosbuild_add_executable(rosmongodb_log_trimesh src/rosmongodb_log_trimesh.cpp)
target_link_libraries(mongodb_log_tf
  ${catkin_LIBRARIES}
  mongoclient
  ssl
  crypto
  ${Boost_LIBRARIES}
)
target_link_libraries(mongodb_log_pcl
  ${catkin_LIBRARIES}
  mongoclient
  ssl
  crypto
  ${Boost_LIBRARIES}
)
target_link_libraries(mongodb_log_cimg
  ${catkin_LIBRARIES}
  mongoclient
  ssl
  crypto
  ${Boost_LIBRARIES}
)

#rosbuild_link_boost(mongodb_log_cimg system)
#rosbuild_link_boost(mongodb_log_pcl system)
#rosbuild_link_boost(mongodb_log_tf system)
#rosbuild_link_boost(mongodb_log_trimesh system)


## Declare a cpp executable
# add_executable(mongodb_log_node src/mongodb_log_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mongodb_log_node mongodb_log_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(mongodb_log_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/mongodb_log.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark executables and/or libraries for installation
install(TARGETS mongodb_log_tf mongodb_log_cimg mongodb_log_pcl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mongodb_log.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
