cmake_minimum_required(VERSION 3.10)
project(fake_controller)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fake_controller
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_rc_node src/fake_rc_pub.cpp)
target_link_libraries(fake_rc_node ${catkin_LIBRARIES})

add_executable(fake_pose_node src/fake_pose_pub.cpp)
target_link_libraries(fake_pose_node ${catkin_LIBRARIES})