cmake_minimum_required(VERSION 2.8.3)
project(marble_multi_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  message_generation
  marble_artifact_detection_msgs
  octomap_merger
)

add_service_files(
  FILES
  GetMapDiffs.srv
  )

add_message_files(
  FILES
  CommsCheck.msg
  CommsCheckArray.msg
  Beacon.msg
  BeaconArray.msg
  AgentArtifact.msg
  AgentMsg.msg
  NeighborMsg.msg
  Goal.msg
  GoalArray.msg
  MapDiffsReq.msg
  MapDiffsResp.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  marble_artifact_detection_msgs
  octomap_merger
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
