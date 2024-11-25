#!/usr/bin/env python

import rospy
from robot_manager.msg import Task
from robot_manager.srv import GetFreeRobots, GetTaskCost
from task_data_services.srv import QueryTaskData
import heapq
import numpy as np
from scipy.optimize import linear_sum_assignment

# Priority queue to store tasks as (priority, task_description) tuples
priority_queue = []

def task_callback(task):
    # Push the task into the priority queue
    heapq.heappush(priority_queue, (task.priority, task))

def assign_tasks():
    while not rospy.is_shutdown():
        if not priority_queue:
            rospy.sleep(1)
            continue

        # Retrieve all tasks with the highest priority
        highest_priority = priority_queue[0][0] if priority_queue else None
        tasks = []
        while priority_queue and priority_queue[0][0] == highest_priority:
            _, task = heapq.heappop(priority_queue)
            tasks.append(task)

        if not tasks:
            continue

        # Get free robots
        try:
            free_robots_client = rospy.ServiceProxy('/robot_manager/GetFreeRobots', GetFreeRobots)
            free_robots_response = free_robots_client()
            free_robot_ids = free_robots_response.robot_ids

            if not free_robot_ids:
                rospy.loginfo("No free robots available, re-queueing tasks...")
                for task in tasks:
                    heapq.heappush(priority_queue, (task.priority, task))
                rospy.sleep(1)
                continue
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get free robots: {e}")
            for task in tasks:
                heapq.heappush(priority_queue, (task.priority, task))
            rospy.sleep(1)
            continue

        # Create a cost matrix
        num_tasks = len(tasks)
        num_robots = len(free_robot_ids)
        cost_matrix = np.full((num_tasks, num_robots), np.inf)

        try:
            for i, task in enumerate(tasks):
                # Get metadata for the task
                metadata_client = rospy.ServiceProxy('/task_data_service/QueryTaskData', QueryTaskData)
                metadata = metadata_client(task.description).metadata

                # Calculate costs for each robot
                for j, robot_id in enumerate(free_robot_ids):
                    task_cost_client = rospy.ServiceProxy('/robot_manager/GetTaskCost', GetTaskCost)
                    cost = task_cost_client(task=task.description, robot_id=robot_id).cost
                    cost_matrix[i, j] = cost
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed during cost calculation: {e}")
            # Re-queue tasks in case of a failure
            for task in tasks:
                heapq.heappush(priority_queue, (task.priority, task))
            continue

        # Solve the assignment problem
        task_indices, robot_indices = linear_sum_assignment(cost_matrix)

        # Assign tasks to robots
        for task_idx, robot_idx in zip(task_indices, robot_indices):
            if cost_matrix[task_idx, robot_idx] < np.inf:
                task = tasks[task_idx]
                robot_id = free_robot_ids[robot_idx]
                rospy.loginfo(f"Assigned task '{task.description}' to robot {robot_id}")
            else:
                rospy.loginfo("No valid assignment found for some tasks.")

        # Requeue any tasks that couldn't be assigned
        for i in range(num_tasks):
            if i not in task_indices:
                heapq.heappush(priority_queue, (tasks[i].priority, tasks[i]))

if __name__ == "__main__":
    rospy.init_node('task_assigner')
    rospy.Subscriber('/task_topic', Task, task_callback)
    assign_tasks()
