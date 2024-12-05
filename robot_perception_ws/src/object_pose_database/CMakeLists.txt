cmake_minimum_required(VERSION 3.0.2)
project(object_pose_database)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  task_data_services
)

add_service_files(
  FILES
  GetObjectPose.srv
  UpdatePose.srv
  UpdateDatabase.srv
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
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime task_data_services
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
