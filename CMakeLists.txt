cmake_minimum_required(VERSION 2.8.3)
project(bobcat)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  message_generation
  marble_artifact_detection_msgs
  marble_mapping
)

add_message_files(
  FILES
  CommsCheck.msg
  CommsCheckArray.msg
  Beacon.msg
  BeaconArray.msg
  AgentArtifact.msg
  AgentMsg.msg
  AgentReset.msg
  ArtifactScore.msg
  NeighborMsg.msg
  Goal.msg
  GoalCompressed.msg
  GoalArray.msg
  PointArray.msg
  PointArrays.msg
  DMReq.msg
  DMReqArray.msg
  DMResp.msg
  DMRespArray.msg
  BobcatStatus.msg
  Monitor.msg
  Objective.msg
  Behavior.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  marble_artifact_detection_msgs
  marble_mapping
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
