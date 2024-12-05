cmake_minimum_required(VERSION 3.0.2)
project(robot_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  visualization_msgs
  path_planner
  object_pose_database
)

add_service_files(
  FILES
  GetFreeRobots.srv
  GetTaskCost.srv
  GetState.srv
)

add_message_files(
  FILES
  Task.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime geometry_msgs visualization_msgs path_planner object_pose_database
)

catkin_install_python(PROGRAMS
  src/individual_robot_state.py  # Path to your Python executable
  src/robot_manager.py
  src/robot_state_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
DIRECTORY launch 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
