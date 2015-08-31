cmake_minimum_required(VERSION 2.8.3)
project(mongodb_store_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib actionlib_msgs)

add_message_files(
  FILES
  StringList.msg
  StringPair.msg
  StringPairList.msg
  SerialisedMessage.msg
)

add_service_files(
  FILES
  MongoInsertMsg.srv
  MongoUpdateMsg.srv
  MongoQueryMsg.srv
  MongoDeleteMsg.srv
)

add_action_files(
  FILES
  MoveEntries.action
)


generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
 CATKIN_DEPENDS message_generation
)

