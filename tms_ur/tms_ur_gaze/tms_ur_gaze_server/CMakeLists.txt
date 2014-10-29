cmake_minimum_required(VERSION 2.8.3)
project(tms_ur_gaze_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  result.msg
  dictation_data.msg
)

add_service_files(
  FILES
  object_list.srv
  recognized_text.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
# INCLUDE_DIRS include
  LIBRARIES tms_ur_gaze_server
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_selector_test src/object_selector.cpp)
add_dependencies(object_selector_test tms_ur_gaze_server_generate_messages_cpp)
target_link_libraries(object_selector_test
  ${catkin_LIBRARIES}
)

add_executable(request_generator_test src/request_generator.cpp)
add_dependencies(request_generator_test tms_ur_gaze_server_generate_messages_cpp)
target_link_libraries(request_generator_test
  ${catkin_LIBRARIES}
)
