cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  robot_manager
  task_data_services
)

add_message_files(
  FILES
  Task.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime robot_manager task_data_services
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
