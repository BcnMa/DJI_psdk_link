cmake_minimum_required(VERSION 3.10)
project(psdk)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs  
)


add_library(psdk_library src/hal_uart.c src/p_link.cpp src/p_link_cmp.cpp)
target_link_libraries(psdk_library ${catkin_LIBRARIES})

add_executable(psdk_node src/psdk_node.cpp)
target_link_libraries(psdk_node psdk_library ${catkin_LIBRARIES})

