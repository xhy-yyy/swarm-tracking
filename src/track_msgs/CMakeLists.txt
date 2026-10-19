cmake_minimum_required(VERSION 3.0.2)
project(track_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  std_msgs
)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  TargetState.msg
  TargetArray.msg
  TrackingTarget.msg
  TrackingView.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs std_msgs
)
