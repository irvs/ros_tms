cmake_minimum_required(VERSION 2.8.3)
project(mongodb_store)

# include our own cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS roscpp message_generation rospy std_msgs std_srvs mongodb_store_msgs)

find_package(MongoClient REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################


## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetParam.srv
  SetParam.srv
  MongoFind.srv
  MongoUpdate.srv
  MongoInsert.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES message_store 
 CATKIN_DEPENDS mongodb_store_msgs
 DEPENDS MongoClient
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MongoClient_INCLUDE_DIR} 
)

link_directories(${catkin_LINK_DIRS})
link_directories(${MongoClient_LINK_DIRS})

add_library(message_store src/message_store.cpp )

add_executable(example_mongodb_store_cpp_client src/example_mongodb_store_cpp_client.cpp)
add_executable(example_multi_event_log src/example_multi_event_log.cpp)
# add_executable(pc_test src/point_cloud_test.cpp)


add_dependencies(example_mongodb_store_cpp_client mongodb_store_msgs_generate_messages_cpp )
add_dependencies(message_store mongodb_store_msgs_generate_messages_cpp )

target_link_libraries(message_store
  ${MongoClient_LIBRARIES}
  ${catkin_LIBRARIES}
)



# Specify libraries to link a library or executable target against
target_link_libraries(example_mongodb_store_cpp_client
  message_store
  ${MongoClient_LIBRARIES}  
  ${catkin_LIBRARIES}  
)

target_link_libraries(example_multi_event_log
  message_store
  ${MongoClient_LIBRARIES}  
  ${catkin_LIBRARIES}  
)

target_link_libraries(example_multi_event_log
  message_store
  ${MongoClient_LIBRARIES}  
  ${catkin_LIBRARIES}  
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/mongodb_store/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS example_mongodb_store_cpp_client example_multi_event_log message_store #pc_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)

  add_rostest(tests/message_store.test)

  add_executable(message_store_cpp_test tests/message_store_cpp_test.cpp)

  target_link_libraries(message_store_cpp_test
    message_store 
  ${MongoClient_LIBRARIES} ${OPENSSL_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    gtest
  ) 

  add_rostest(tests/message_store_cpp_client.test)

endif()
