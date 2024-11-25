cmake_minimum_required(VERSION 3.0.2)
project(robot_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  GetFreeRobots.srv
  GetTaskCost.srv
)

add_message_files(
  FILES
  Task.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
