cmake_minimum_required(VERSION 2.8.3)
project(tms_db_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tms_msg_db
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES tms_db_manager
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tms_msg_db
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tms_db_writer src/tms_db_writer.cpp)
add_dependencies(tms_db_writer tms_msg_db_generate_messages_cpp)
target_link_libraries(tms_db_writer ${catkin_LIBRARIES} mysqlclient)

add_executable(tms_db_reader src/tms_db_reader.cpp)
add_dependencies(tms_db_reader tms_msg_db_generate_messages_cpp)
target_link_libraries(tms_db_reader ${catkin_LIBRARIES} mysqlclient)

add_executable(tms_db_manager src/tms_db_manager.cpp)
add_dependencies(tms_db_manager tms_msg_db_generate_messages_cpp)
target_link_libraries(tms_db_manager ${catkin_LIBRARIES} mysqlclient)

add_executable(tms_db_publisher src/tms_db_publisher.cpp)
add_dependencies(tms_db_publisher tms_msg_db_generate_messages_cpp)
target_link_libraries(tms_db_publisher ${catkin_LIBRARIES} mysqlclient)
