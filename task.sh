# Check if the correct number of arguments is provided
# if [ "$#" -ne 3 ]; then
#   echo "Usage: $0 <node_name> <param_name> <n>"
#   exit 1
# fi

# Extract arguments
# NODE_NAME=$1
# PARAM_NAME=$2
N=$1

# Ensure N is a positive integer
if ! [[ $N =~ ^[0-9]+$ ]] || [ "$N" -le 0 ]; then
  echo "Error: <n> must be a positive integer."
  exit 1
fi

# Launch the ROS node N times
for ((i=1; i<=N; i++)); do
  # echo "Launching $NODE_NAME with $PARAM_NAME=$i"
  roslaunch robot_manager  individual_robot_state.launch robot_namespace:="ugv$i" &
  sleep 1
  # Optionally add a delay if needed between launches
  # sleep 1
done

# Launch the ROS node N times
for ((i=1; i<=N; i++)); do
  # echo "Launching $NODE_NAME with $PARAM_NAME=$i"
  roslaunch object_pose_estimator yolo_detect.launch robot_namespace:="ugv$i" &
  sleep 1
  # Optionally add a delay if needed between launches
  # sleep 1
done

roslaunch robot_manager robot_manager.launch num_robots:=$N
roslaunch task_manager task.launch

# Optionally wait for all background processes to complete
wait

echo "All $N instances of $NODE_NAME launched successfully."