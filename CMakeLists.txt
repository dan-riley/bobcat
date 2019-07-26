cmake_minimum_required(VERSION 2.8.3)
project(marble_multi_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  CommsCheck.msg
  CommsCheckArray.msg
  Beacon.msg
  BeaconArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
