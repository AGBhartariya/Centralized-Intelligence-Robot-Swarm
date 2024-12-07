cmake_minimum_required(VERSION 3.0.2)
project(object_pose_estimator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  object_pose_database
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs object_pose_database
)

catkin_install_python(PROGRAMS
  src/yolo_pose_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
DIRECTORY launch 
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
