cmake_minimum_required(VERSION 2.8.3)
project(melle_refactored)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  MellE_msg.msg
  PC_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS 
)

include_directories(include/melle_refactored)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(melle_refactored src/pc_node.cpp src/PID.cpp src/PID_horz.cpp)
target_link_libraries(melle_refactored ${catkin_LIBRARIES} )