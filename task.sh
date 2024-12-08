#!/bin/bash

# Check if argument is provided
if [ $# -eq 0 ]; then
  echo "Usage: $0 <n>"
  exit 1
fi

N=$1

# Ensure N is a positive integer
if ! [[ $N =~ ^[0-9]+$ ]] || [ "$N" -le 0 ]; then
  echo "Error: <n> must be a positive integer."
  exit 1
fi

# Function to open a new terminal and run a command
open_terminal() {
  local title="$1"
  local command="$2"
  gnome-terminal --title="$title" -- bash -c "$command; exec bash"
}

# Launch individual robot state nodes
for ((i=1; i<=N; i++)); do
  open_terminal "Robot State UGV$i" "roslaunch robot_manager individual_robot_state.launch robot_namespace:=ugv$i"
  sleep 1
done

# Launch YOLO detection nodes
for ((i=1; i<=N; i++)); do
  open_terminal "YOLO Detect UGV$i" "roslaunch object_pose_estimator yolo_detect.launch robot_namespace:=ugv$i"
  sleep 1
done

# Launch additional nodes
open_terminal "Object Pose Database" "roslaunch object_pose_database database.launch no_of_robots:=$N"
open_terminal "Robot Manager" "roslaunch robot_manager robot_manager.launch num_robots:=$N"
open_terminal "Task Manager" "roslaunch task_manager task.launch"

echo "Launched $N robot instances across multiple terminals."