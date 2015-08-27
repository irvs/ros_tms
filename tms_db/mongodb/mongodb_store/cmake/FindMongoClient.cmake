# - Find MongoClient; NOTE: this is specific to warehouse_ros!
# 
# And copied to here from https://raw.githubusercontent.com/ros-planning/warehouse_ros/master/cmake/FindMongoDB.cmake -- thanks :)
#
# Find the MongoClient includes and client library
# This module defines
#  MongoClient_INCLUDE_DIR, where to find mongo/client/dbclient.h
#  MongoClient_LIBRARIES, the libraries needed to use MongoClient.
#  MongoClient_FOUND, If false, do not try to use MongoClient.
#  MongoClient_EXPOSE_MACROS, If true, mongo_ros should use '#define MONGO_EXPOSE_MACROS'



set(MongoClient_EXPOSE_MACROS "NO")

set(MongoClient_PossibleIncludePaths
  /usr/include/
  /usr/local/include/
  /usr/include/mongo/
  /usr/local/include/mongo/
  /opt/mongo/include/
  $ENV{ProgramFiles}/Mongo/*/include
  $ENV{SystemDrive}/Mongo/*/include
  )
find_path(MongoClient_INCLUDE_DIR mongo/client/dbclient.h
  ${MongoClient_PossibleIncludePaths})

if(MongoClient_INCLUDE_DIR)
  find_path(MongoClient_dbclientinterface_Path mongo/client/dbclientinterface.h
    ${MongoClient_PossibleIncludePaths})
  if (MongoClient_dbclientinterface_Path)
    set(MongoClient_EXPOSE_MACROS "YES")
  endif()
endif()

if(WIN32)
  find_library(MongoClient_LIBRARIES NAMES mongoclient
    PATHS
    $ENV{ProgramFiles}/Mongo/*/lib
    $ENV{SystemDrive}/Mongo/*/lib
    )
else(WIN32)
  find_library(MongoClient_LIBRARIES NAMES mongoclient
    PATHS
    /usr/lib
    /usr/lib64
    /usr/lib/mongo
    /usr/lib64/mongo
    /usr/local/lib
    /usr/local/lib64
    /usr/local/lib/mongo
    /usr/local/lib64/mongo
    /opt/mongo/lib
    /opt/mongo/lib64
    )
endif(WIN32)

# if we got the mongo stuff, add in the other things we want
if(MongoClient_LIBRARIES)
  find_package(Boost REQUIRED COMPONENTS system thread program_options filesystem)
  find_package(OpenSSL REQUIRED)
  set(MongoClient_LIBRARIES ${MongoClient_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_LIBRARIES})  
endif(MongoClient_LIBRARIES)  

if(MongoClient_INCLUDE_DIR AND MongoClient_LIBRARIES)
  set(MongoClient_FOUND TRUE)
  message(STATUS "Found MongoClient: ${MongoClient_INCLUDE_DIR}, ${MongoClient_LIBRARIES}")
  message(STATUS "MongoClient using new interface: ${MongoClient_EXPOSE_MACROS}")
else(MongoClient_INCLUDE_DIR AND MongoClient_LIBRARIES)
  set(MongoClient_FOUND FALSE)
  if (MongoClient_FIND_REQUIRED)
    message(FATAL_ERROR "MongoClient not found.")
  else (MongoClient_FIND_REQUIRED)
    message(STATUS "MongoClient not found.")
  endif (MongoClient_FIND_REQUIRED)
endif(MongoClient_INCLUDE_DIR AND MongoClient_LIBRARIES)

mark_as_advanced(MongoClient_INCLUDE_DIR MongoClient_LIBRARIES MongoClient_EXPOSE_MACROS)
