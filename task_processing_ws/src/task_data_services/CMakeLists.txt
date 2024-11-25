cmake_minimum_required(VERSION 3.0.2)
project(task_data_services)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  QueryObjectLocations.srv
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
