cmake_minimum_required(VERSION 2.8.3)
project(psdk_msgs)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  DroneState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

