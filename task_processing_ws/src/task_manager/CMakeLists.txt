cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robot_manager
  object_pose_database
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime robot_manager object_pose_database
)

catkin_install_python(PROGRAMS
  src/task_assigner.py
  src/task_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
DIRECTORY launch 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
