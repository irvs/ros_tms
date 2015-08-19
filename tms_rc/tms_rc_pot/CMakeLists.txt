cmake_minimum_required(VERSION 2.8.3)
project(tms_rc_pot)

find_package(catkin REQUIRED COMPONENTS
  tms_msg_rc
  roscpp
  rospy
  tf
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_test
#  CATKIN_DEPENDS roscpp rospy tf turtle_sim
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_test_node src/tf_test.cpp)
add_dependencies(tf_test_node tf_test_generate_messages_cpp)
target_link_libraries(tf_test_node ${catkin_LIBRARIES})

add_executable(virtual_robot src/virtual_pot_robot.cpp)
add_dependencies(virtual_robot tf_test_generate_messages_cpp tms_msg_rc_gencpp)
target_link_libraries(virtual_robot ${catkin_LIBRARIES})
