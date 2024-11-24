#!/usr/bin/env python

import rospy
from task_manager.msg import Task
from robot_manager.srv import GetFreeRobots, GetTaskCost
from task_data_services.srv import QueryTaskData
import heapq
from collections import deque

# Task priority queue and queue for unprocessed tasks
priority_queue = []
unprocessed_tasks = deque()

def task_callback(task):
    # Add the task to the unprocessed queue
    unprocessed_tasks.append(task)

def assign_tasks():
    while not rospy.is_shutdown():
        if not unprocessed_tasks:
            rospy.sleep(1)
            continue

        # Pop the next task
        current_task = unprocessed_tasks.popleft()

        try:
            # Get metadata for cost calculation
            metadata_client = rospy.ServiceProxy('/task_data_service/QueryTaskData', QueryTaskData)
            metadata = metadata_client(current_task.description).metadata


            # Get free robots
            free_robots_client = rospy.ServiceProxy('/robot_manager/GetFreeRobots', GetFreeRobots)
            free_robots_response = free_robots_client()
            free_robot_count = free_robots_response.free_robot_count
            free_robot_ids = free_robots_response.robot_ids

            # Calculate task cost for each free robot
            costs = []
            for robot_id in free_robot_ids:
                task_cost_client = rospy.ServiceProxy('/robot_manager/GetTaskCost', GetTaskCost)
                cost = task_cost_client(task_description=current_task.description, robot_id=robot_id).cost
                costs.append((cost, robot_id))

            # Assign the task to the robot with the least cost
            if costs:
                heapq.heapify(costs)
                best_robot = heapq.heappop(costs)[1]
                rospy.loginfo(f"Assigned task '{current_task.description}' to robot {best_robot}")
            else:
                rospy.loginfo("No free robots available, task will be retried.")
                unprocessed_tasks.append(current_task)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            unprocessed_tasks.append(current_task)

if __name__ == "__main__":
    rospy.init_node('task_assigner')
    rospy.Subscriber('/task_topic', Task, task_callback)
    assign_tasks()
