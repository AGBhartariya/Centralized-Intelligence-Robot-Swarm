#!/usr/bin/env python

import rospy
from robot_manager.msg import Task
from robot_manager.srv import GetFreeRobots, GetTaskCost
from object_pose_database.srv import QueryObjectLocations, QueryObjectLocationsRequest
from geometry_msgs.msg import PoseStamped
import heapq
import numpy as np
from scipy.optimize import linear_sum_assignment
import time
import json  # For serializing task description arrays
import itertools

# Priority queue and a dictionary to track task timestamps
priority_queue = []  # Stores (priority, task) tuples
task_timestamps = {}  # Maps task description to timestamp
_task_counter = itertools.count() 
task_pubs = {}

def pose_stamped_to_dict(pose_stamped: PoseStamped):
    """
    Converts a PoseStamped message to a dictionary.
    """
    return {
        "header": {
            "seq": pose_stamped.header.seq,
            "stamp": {
                "secs": pose_stamped.header.stamp.secs,
                "nsecs": pose_stamped.header.stamp.nsecs
            },
            "frame_id": pose_stamped.header.frame_id
        },
        "pose": {
            "position": {
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y,
                "z": pose_stamped.pose.position.z
            },
            "orientation": {
                "x": pose_stamped.pose.orientation.x,
                "y": pose_stamped.pose.orientation.y,
                "z": pose_stamped.pose.orientation.z,
                "w": pose_stamped.pose.orientation.w
            }
        }
    }

def task_to_dict(task: Task):
    """
    Converts the task object to a dictionary format.
    Handles PoseStamped serialization.
    """
    task_dict = {
        "priority": getattr(task, "priority", None),
        "description": getattr(task, "description", None),
    }

    # Handle locations which might contain PoseStamped
    locations = getattr(task, "locations", None)
    if locations:
        # Convert locations to a list of dictionaries if they are PoseStamped
        task_dict["locations"] = [
            pose_stamped_to_dict(loc) if hasattr(loc, 'header') else loc 
            for loc in locations
        ]

    return task_dict

def serialize_task_description(task: Task):
    """Serializes the task description (object) into a string."""
    task_dict = task_to_dict(task)
    return json.dumps(task_dict, allow_nan=True)

def task_callback(task: Task):
    # Serialize task description for consistent storage and logging
    rospy.loginfo(f"Received task {task}")
    serialized_description = serialize_task_description(task)

    # Add the task to the priority queue and record its timestamp
    heapq.heappush(priority_queue, (task.priority, next(_task_counter), task))
    task_timestamps[serialized_description] = time.time()


def increment_task_priorities():
    """Periodically increments the priorities of unassigned tasks."""
    k = rospy.get_param("~task_reassignment_interval", 10)  # Time interval in minutes
    n = rospy.get_param("~task_priority_increment", 1)  # Priority increment amount
    current_time = time.time()

    # Convert `k` to seconds
    k_seconds = k * 60

    updated_tasks = []
    while priority_queue:
        priority, _, task = heapq.heappop(priority_queue)
        serialized_description = serialize_task_description(task)
        timestamp = task_timestamps[serialized_description]

        # Check if the task has exceeded the `k`-minute threshold
        if current_time - timestamp >= k_seconds:
            rospy.loginfo(
                f"Incrementing priority for task '{serialized_description}' by {n}."
            )
            priority -= n  # Lower priority value means higher priority
            task_timestamps[serialized_description] = current_time  # Update timestamp

        # Reinsert the task into the queue with updated priority
        updated_tasks.append((priority, task))

    # Rebuild the priority queue with updated priorities
    for priority, task in updated_tasks:
        heapq.heappush(priority_queue, (priority, next(_task_counter), task))


def assign_tasks():
    last_execution_time = 0  # Timestamp of the last execution

    while not rospy.is_shutdown():
        # Increment task priorities
        increment_task_priorities()

        k = rospy.get_param(
            "~task_execution_interval", 0
        )  # Execution interval in minutes
        k_seconds = k * 60
        current_time = time.time()

        # Check if enough time has elapsed since the last execution
        if current_time - last_execution_time < k_seconds:
            rospy.sleep(1)
            continue
        
        rospy.loginfo("Trying to reassign tasks")
        last_execution_time = current_time  # Update the last execution time

        if not priority_queue:
            rospy.loginfo("No tasks available retrying after another cycle")
            rospy.sleep(1)
            continue

        # Retrieve all tasks with the highest priority
        highest_priority = priority_queue[0][0] if priority_queue else None
        tasks = []
        while priority_queue and priority_queue[0][0] == highest_priority:
            _, _, task = heapq.heappop(priority_queue)
            tasks.append(task)

        if not tasks:
            continue

        # Get free robots
        try:
            free_robots_client = rospy.ServiceProxy(
                "get_free_robots", GetFreeRobots
            )
            free_robots_response = free_robots_client()
            free_robot_ids = free_robots_response.robot_ids

            if not free_robot_ids:
                rospy.loginfo("No free robots available, re-queueing tasks...")
                for task in tasks:
                    heapq.heappush(priority_queue, (task.priority, next(_task_counter), task))
                continue
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get free robots: {e}")
            for task in tasks:
                heapq.heappush(priority_queue, (task.priority, next(_task_counter), task))
            continue

        # Create a cost matrix and store object locations
        num_tasks = len(tasks)
        num_robots = len(free_robot_ids)
        cost_matrix = np.full((num_tasks, num_robots), np.inf)
        task_object_locations = [
            [None for _ in range(num_robots)] for _ in range(num_tasks)
        ]

        try:
            for i, task in enumerate(tasks):
                data = task.description
                task_locations = task.locations
                # Get metadata for the task
                object_locations_client = rospy.ServiceProxy("/query_loc", QueryObjectLocations)
                request=QueryObjectLocationsRequest()
                request.objectType=data[1]
                object_locations = object_locations_client(request).locations

                rospy.loginfo(f"Found {len(object_locations)} object {data[1]} at {object_locations}")
                # Calculate costs for each robot
                for j, robot_id in enumerate(free_robot_ids):
                    task_cost_client = rospy.ServiceProxy(
                        "get_task_cost", GetTaskCost
                    )
                    response = task_cost_client(
                        task_type=data[0],
                        robot_id=robot_id,
                        objectlocations=object_locations,
                        tasklocations=task_locations,
                    )
                    cost = response.cost
                    object_location = response.object_location  # PoseStamped

                    cost_matrix[i, j] = cost
                    task_object_locations[i][j] = object_location

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed during cost calculation: {e}")
            # Re-queue tasks in case of a failure
            for task in tasks:
                heapq.heappush(priority_queue, (task.priority, next(_task_counter), task))
            continue

        # Solve the assignment problem
        rospy.loginfo(f"Task cost matrix formed \n {cost_matrix}")
        task_indices, robot_indices = linear_sum_assignment(cost_matrix)

        # Assign tasks to robots
        for task_idx, robot_idx in zip(task_indices, robot_indices):
            if cost_matrix[task_idx, robot_idx] < np.inf:
                task = tasks[task_idx]
                robot_id = free_robot_ids[robot_idx]
                assigned_location = task_object_locations[task_idx][robot_idx]
                task = generateTaskMsg(task, assigned_location)
                if robot_id not in task_pubs:
                    task_pubs[robot_id] = rospy.Publisher(f"/{robot_id}/start_task", Task, queue_size=1)
                    rospy.sleep(1)
                    
                task_pubs[robot_id].publish(task)
                rospy.loginfo(f"Published task {task} on /{robot_id}/start_task")
            else:
                rospy.loginfo("No valid assignment found for some tasks.")

        # Requeue any tasks that couldn't be assigned
        for i in range(num_tasks):
            if i not in task_indices:
                heapq.heappush(priority_queue, (tasks[i].priority, next(_task_counter), tasks[i]))

def generateTaskMsg(task: Task, assigned_location: PoseStamped) -> Task:
    # Format the task as the per the individual robot state requirement
    if task.description[0] == 1:
        task.locations[0] = assigned_location
    elif task.description[0] == 2:
        pass
    elif task.description[0] == 3:
        pass
    elif task.description[0] == 4:
        task.locations[0] = assigned_location
    elif task.description[0] == 5:
        pass
    return task

if __name__ == "__main__":
    rospy.init_node("task_assigner")
    rospy.loginfo("Started task assignment node")
    rospy.Subscriber("/task_topic", Task, task_callback)
    assign_tasks()
