cmake_minimum_required(VERSION 2.8.3)
project(mujoco_ros_msgs)

set(MSG_DEPS
  std_msgs
  geometry_msgs
  object_recognition_msgs
)

find_package(catkin REQUIRED COMPONENTS roslib genmsg message_generation ${MSG_DEPS})

add_message_files(
  FILES
  ModelStates.msg
)

add_service_files(
  FILES
  SpawnObjects.srv
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roslib message_runtime ${MSG_DEPS}
  INCLUDE_DIRS
  LIBRARIES
)