cmake_minimum_required(VERSION 3.0.2)
project(task_data_services)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  robot_manager
)

add_service_files(
  FILES
  QueryTaskData.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  robot_manager
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime robot_manager
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
