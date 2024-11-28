#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from robot_manager.srv import GetState
from geometry_msgs.msg import Point
import random
import time
import numpy as np
from scipy.optimize import linear_sum_assignment


class RobotStateManager:
    def __init__(self):
        # Initialize the node
        rospy.init_node("robot_state_manager", anonymous=True)

        # Get number of robots from ROS parameter
        self.num_robots = rospy.get_param("~numRobots", 1)

        # Time interval for point sampling (in minutes, converted to seconds)
        self.sampling_interval = rospy.get_param("~sampling_interval", 1) * 60

        # List of robot namespaces
        self.robot_namespaces = [f"ugv{i + 1}" for i in range(self.num_robots)]
        self.robot_states = {ns: 0 for ns in self.robot_namespaces}
        self.no_gain_received = {ns: False for ns in self.robot_namespaces}

        # Timestamp of the last NoGain message
        self.last_no_gain_time = None
        self.patrol = False
        # Subscriptions to /update topics
        self.subscribers = []
        for ns in self.robot_namespaces:
            topic = f"/{ns}/expl_marker_controller/update"
            self.subscribers.append(
                rospy.Subscriber(topic, String, self.update_callback, callback_args=ns)
            )

        # Timer for assigning tasks (not started until all robots send NoGain)
        self.assignment_timer = None
        self.is_patrolling_pub = rospy.Publisher(f"isPatrolling", Bool, queue_size=1)

        rospy.loginfo(f"RobotStateManager initialized for {self.num_robots} robots.")

    def update_callback(self, msg, robot_namespace):
        # Check if the robot sent any message other than "NoGain"
        if all(self.no_gain_received.values()) and self.patrol:
            return
        
        if msg.data != "NoGain":
            # Reset the robot state if the message comes after the last NoGain and within t minutes
            if self.last_no_gain_time and (time.time() - self.last_no_gain_time <= self.sampling_interval):
                rospy.loginfo(f"Resetting state for {robot_namespace} due to activity after last NoGain.")
                self.robot_states[robot_namespace] = 1
                self.no_gain_received[robot_namespace] = False

                # Kill the existing timer if it's still running
                if self.assignment_timer:
                    rospy.logwarn(f"Killing existing timer due to activity from {robot_namespace}.")
                    self.assignment_timer.shutdown()
                    self.assignment_timer = None

                # Reset no_gain_received to require all robots to send NoGain again
                self.no_gain_received = {ns: False for ns in self.robot_namespaces}

        # Handle "NoGain" message
        if msg.data == "NoGain":
            if not self.no_gain_received[robot_namespace]:
                rospy.loginfo(f"Received NoGain from {robot_namespace}")
                self.no_gain_received[robot_namespace] = True
                self.robot_states[robot_namespace] = 0

                # Update the last no-gain timestamp
                self.last_no_gain_time = time.time()

        # Check if all robots have sent NoGain
        if all(self.no_gain_received.values()):
            rospy.loginfo("All robots have sent NoGain. Broadcasting isPatrolling status.")

            # Start the assignment timer after the specified sampling interval
            if self.assignment_timer:
                self.assignment_timer.shutdown()
            self.assignment_timer = rospy.Timer(
                rospy.Duration(self.sampling_interval),
                self.assign_points,
                oneshot=True,
            )

    def sample_points(self, num_points=10):
        # Placeholder for map boundaries; adjust as needed
        return [Point(x=random.uniform(0, 100), y=random.uniform(0, 100), z=0.0) for _ in range(num_points)]

    def calculate_cost(self, robot_position, goal_position):
        # Compute Euclidean distance as the cost
        dx = robot_position.x - goal_position.x
        dy = robot_position.y - goal_position.y
        return (dx ** 2 + dy ** 2) ** 0.5

    def assign_points(self, event):
        self.patrol = True
        self.is_patrolling_pub.publish(True)
        # Reset no_gain_received to require all robots to resend NoGain
        self.no_gain_received = {ns: False for ns in self.robot_namespaces}

        # Check the states of all robots by calling their GetState service
        free_robots = []
        for robot in self.robot_namespaces:
            service_name = f"/{robot}/get_state"
            try:
                rospy.wait_for_service(service_name, timeout=5)
                get_state_service = rospy.ServiceProxy(service_name, GetState)
                self.robot_states[robot] = get_state_service().state
                if self.robot_states[robot] == 0:
                    free_robots.append(robot)
                else:
                    rospy.loginfo(f"{robot} is busy (state: {self.robot_states[robot]})")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call to {service_name} failed: {e}")
            except rospy.ROSException:
                rospy.logwarn(f"Timeout while waiting for service: {service_name}")

        # If no free robots are available, log and return
        if not free_robots:
            rospy.loginfo("No free robots available for point assignment.")
            return

        # Sample points from the map
        num_points = len(free_robots)  # Ensure there are as many points as free robots
        sampled_points = self.sample_points(num_points=num_points)

        # Retrieve positions of free robots
        robot_positions = {
            robot: Point(
                x=random.uniform(0, 100), y=random.uniform(0, 100), z=0.0
            )
            for robot in free_robots
        }  # Replace with actual position retrieval logic

        # Calculate cost matrix
        cost_matrix = np.zeros((len(free_robots), len(sampled_points)))
        for i, robot in enumerate(free_robots):
            for j, point in enumerate(sampled_points):
                cost_matrix[i][j] = self.calculate_cost(robot_positions[robot], point)

        # Use Hungarian Algorithm to find the optimal assignment
        robot_indices, point_indices = linear_sum_assignment(cost_matrix)

        # Assign points to robots based on the optimal assignment
        assignments = {}
        for i in range(len(robot_indices)):
            robot = free_robots[robot_indices[i]]
            point = sampled_points[point_indices[i]]
            assignments[robot] = point

        # Update robot states and log assignments
        for robot, point in assignments.items():
            self.robot_states[robot] = 2
            rospy.loginfo(f"Assigned {robot} to point {point}")

        # Publish or send goals to robots (example placeholder for sending commands)
        for robot, point in assignments.items():
            rospy.loginfo(f"Sending goal to {robot}: {point}")

        # Restart the timer for the next task assignment
        self.assignment_timer = rospy.Timer(
            rospy.Duration(self.sampling_interval),
            self.assign_points,
            oneshot=True,
        )


def main():
    try:
        # Initialize the RobotStateManager
        manager = RobotStateManager()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RobotStateManager terminated.")


if __name__ == "__main__":
    main()
