cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Navigation.msg
  Debug.msg
  Arm.msg
  Arduino.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS 
)

include_directories(include/navigation)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(navigation src/navigation.cpp src/PID.cpp src/PID_horz.cpp src/GPS.cpp src/Motor.cpp src/Visual_Servo.cpp src/waypoints.cpp)
add_dependencies(navigation ${catkin_EXPORTED_TARGETS})
add_dependencies(navigation melle_obstacle_avoidance_generate_messages_cpp)
add_dependencies(navigation heartbeat_generate_messages_cpp)
add_dependencies(navigation yolo2_generate_messages_cpp)
target_link_libraries(navigation ${catkin_LIBRARIES} )