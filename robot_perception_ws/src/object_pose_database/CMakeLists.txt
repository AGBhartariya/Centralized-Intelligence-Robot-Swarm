cmake_minimum_required(VERSION 3.0.2)
project(object_pose_database)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  GetObjectPose.srv
  UpdatePose.srv
  UpdateDatabase.srv
  QueryObjectLocations.srv
)

add_message_files(
  FILES
  DetectObject.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

install(
DIRECTORY launch 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/pose_database_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
