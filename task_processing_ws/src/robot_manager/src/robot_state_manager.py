#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from robot_manager.srv import GetState
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarkerUpdate
import random
import time
import tf
import numpy as np
from scipy.optimize import linear_sum_assignment
from trajectory_control_msgs.srv import GetCostPath, GetCostPathRequest


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
                rospy.Subscriber(
                    topic,
                    InteractiveMarkerUpdate,
                    self.update_callback,
                    callback_args=ns,
                )
            )

        # Timer for assigning tasks (not started until all robots send NoGain)
        self.assignment_timer = None
        self.is_patrolling_pub = rospy.Publisher(f"isPatrolling", Bool, queue_size=1)

        rospy.loginfo(f"RobotStateManager initialized for {self.num_robots} robots.")

    def update_callback(self, msg, robot_namespace):
        # Skip processing if all robots have sent NoGain and patrolling is active
        if all(self.no_gain_received.values()) and self.patrol:
            return

        # Check for menu entries with title "NoGain"
        no_gain_detected = False
        for marker in msg.markers:
            if marker.menu_entries:
                for entry in marker.menu_entries:
                    if entry.title == "NoGain":
                        no_gain_detected = True
                        break

        if no_gain_detected:
            if not self.no_gain_received[robot_namespace]:
                rospy.loginfo(f"Received NoGain from {robot_namespace}")
                self.no_gain_received[robot_namespace] = True
                self.robot_states[robot_namespace] = 0

                # Update the last no-gain timestamp
                self.last_no_gain_time = time.time()
        else:
            # Reset the robot state if a non-NoGain message comes after the last NoGain and within t minutes
            if self.last_no_gain_time and (
                time.time() - self.last_no_gain_time <= self.sampling_interval
            ):
                rospy.loginfo(
                    f"Resetting state for {robot_namespace} due to activity after last NoGain."
                )
                self.robot_states[robot_namespace] = 1
                self.no_gain_received[robot_namespace] = False

                # Kill the existing timer if it's still running
                if self.assignment_timer:
                    rospy.logwarn(
                        f"Killing existing timer due to activity from {robot_namespace}."
                    )
                    self.assignment_timer.shutdown()
                    self.assignment_timer = None

            # Reset no_gain_received for the required robot
            self.no_gain_received[robot_namespace] = False

        # Check if all robots have sent NoGain
        if all(self.no_gain_received.values()):
            rospy.loginfo(
                "All robots have sent NoGain. Broadcasting isPatrolling status."
            )

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
        return [
            Point(x=random.uniform(0, 100), y=random.uniform(0, 100), z=0.0)
            for _ in range(num_points)
        ]

    def calculate_cost(self, robot_position, goal_position):
        # Compute Euclidean distance as the cost
        rospy.wait_for_service('cost_path')
        try:
            cost_path = rospy.ServiceProxy('/trajectory_control_msgs/cost_path', GetCostPath)
            request = GetCostPathRequest()
            goal_point=goal_position.pose.position
            request.task.waypoints = [robot_position, goal_point] 
            request.task.header.frame_id='odom'
            request.task.segment_count=1
            response = cost_path(request)
            total_cost=0
            for path in response.path_list:
                poses = path.poses
                for i in range(len(poses) - 1):
                    # Extract positions of successive waypoints
                    p1 = poses[i].pose.position
                    p2 = poses[i + 1].pose.position

                    distance = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)**0.5
                    total_cost += distance
            return total_cost
        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)
           return None

    def getRobotPos(self,robot_id):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("odom", f"{robot_id}/base_link", rospy.Time(0), rospy.Duration(5.0))
            (trans, rot) = listener.lookupTransform("odom", f"{robot_id}/base_link", rospy.Time(0))
            return Point(x=trans[0],y=trans[1],z=trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error getting transform: %s", e)
            return None

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
                    rospy.loginfo(
                        f"{robot} is busy (state: {self.robot_states[robot]})"
                    )
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
            robot: self.getRobotPos(robot)
            for robot in free_robots
        }

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
