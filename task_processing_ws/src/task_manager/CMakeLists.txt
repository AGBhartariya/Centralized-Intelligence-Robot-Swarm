cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robot_manager
  task_data_services
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime robot_manager task_data_services
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
