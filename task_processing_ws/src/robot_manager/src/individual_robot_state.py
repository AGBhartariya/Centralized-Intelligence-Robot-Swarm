#!/usr/bin/env python

import math
import tf
import rospy
from std_msgs.msg import Bool
from robot_manager.srv import GetState, GetStateResponse
from robot_manager.msg import Task
from geometry_msgs.msg import Twist, Point
from trajectory_control_msgs.msg import PlanningTask 


class RobotStateNode:
    def __init__(self, robot_namespace):
        # Initialize the node
        rospy.init_node("robot_state", anonymous=True)

        # Robot namespace
        self.namespace = robot_namespace

        # Robot state: 0 : Idle, 1 : Exploring, 2 : Patrolling, 3 : Tasking, 4 : Charging
        self.default_state = 1
        self.state = self.default_state

        self.tf_listener = tf.TransformListener()  # TF listener to get transforms
        rospy.sleep(1) 

        # Battery threshold for low battery state
        self.battery_threshold = 10
        self.full_battery = 100

        # Publishers and Subscribers
        self.task_sub = rospy.Subscriber(
            f"{self.namespace}/start_task", Task, self.start_task_callback
        )

        # Service for getting the robot's state
        self.get_state_srv = rospy.Service(
            f"{self.namespace}/get_state", GetState, self.get_state_service
        )

        # Publisher to indicate exploration status
        self.expl_marker_pub = rospy.Publisher(
            f"/{self.namespace}/expl_marker_update/update", Bool, queue_size=1
        )

        # Publisher to stop the robot when charging (sending zero velocity)
        self.cmd_vel_pub = rospy.Publisher(
            f"/{self.namespace}/battery/cmd_vel", Twist, queue_size=1
        )

        # Start with publishing False (indicating not in exploration)
        self.expl_marker_pub.publish(False)

        # Start with a subscriber to check with exploration is finished
        self.patrol_sub = rospy.Subscriber("/isPatrolling", Bool, self.patrol_callback)

        # Start the battery checking loop
        self.check_battery_rate = (
            rospy.get_param("~battery_check_interval", 1) * 60
        )  # in seconds (default to 1 min)
        rospy.Timer(rospy.Duration(self.check_battery_rate), self.check_battery)

        # Publishers for the topics
        self.waypoint_pub = rospy.Publisher(f"/{self.namespace}/planner/waypoints/server", Point, queue_size=10)
        self.task_pub = rospy.Publisher(f"/{self.namespace}/planner/tasks/append", PlanningTask, queue_size=10)
        self.cancel_pub = rospy.Publisher(f"/{self.namespace}/planner/tasks/remove", PlanningTask, queue_size=10)

        rospy.loginfo("Waypoint Publisher Node Initialized!")

        rospy.loginfo(
            f"RobotStateNode for namespace '{self.namespace}' initialized. Default state: '{self.state}'"
        )

        self.timeout_exceeded = False  # Variable to track timeout
        self.timer = None  # To store the timer instance

    def timeout_callback(self, event, task_object):
        """
        Callback triggered when the task timeout is exceeded.
        """
        self.timeout_exceeded = True
        rospy.logwarn(f"Timeout exceeded for task involving {task_object}. Stopping current operation.")

    def publish_waypoint(self, waypoint, task_type):
        # Cancel the previous task
        self.cancel_current_task()

        # Publish the waypoint to /ugv1/planner/waypoints/server/update
        rospy.loginfo(f"Publishing waypoint to /ugv1/planner/waypoints/server: {waypoint}")
        self.waypoint_pub.publish(waypoint)

        # Publish a task for navigation to the waypoint
        rospy.loginfo(f"Publishing waypoint to /ugv1/planner/tasks/append for navigation: {waypoint}")
        task_msg = PlanningTask()  # Create a PlanningTask message
        task_msg.name = "navigate_to_waypoint"
        task_msg.segment_id = 1  # Set a unique segment ID
        task_msg.segment_count = 1  # Only one waypoint in this task
        task_msg.type = task_type  # Either normal or cyclic type
        task_msg.waypoints = [waypoint]  # Add the waypoint to the waypoints array

        # Publish the task to the planner
        self.task_pub.publish(task_msg)

        # Timeout status
        self.timeout_exceeded = False  # Class attribute to track timeout
        self.timer = None  # Timer object reference

    def execute_task(self, task):
        """
        Executes a task based on its type.
        """
        rospy.loginfo(f"Executing task: {task}")
        self.state = 3  # Tasking state

        task_type = task.task_type  # Task type
        task_object = task.task_object  # Task object
        location_1 = task.location_1  # x3
        location_2 = task.location_2  # x4

        k = 180  # Timeout duration in seconds

        if task_type == 1:  # Bring Object
            rospy.loginfo(f"Executing 'Bring Object' task: Bringing {task_object} to {location_1.position}.")
            self.timer = rospy.Timer(rospy.Duration(k), lambda event: self.timeout_callback(event, task_object), oneshot=True)
            self.goToPoint(location_1)
            self.timer.shutdown()
            if self.timeout_exceeded:
                rospy.loginfo(f"Reassigning 'Bring Object' task: {task_object} to {location_1.position}.")
                return

        elif task_type == 2:  # Inspect/Interact
            rospy.loginfo(f"Executing 'Inspect/Interact' task: Interacting with {task_object} at {location_1.position}.")
            self.goToPoint(location_1)
            self.timer = rospy.Timer(rospy.Duration(k), lambda event: self.timeout_callback(event, task_object), oneshot=True)
            self.timer.shutdown()
            if self.timeout_exceeded:
                rospy.loginfo(f"Reassigning 'Inspect/Interact' task: {task_object} to {location_1.position}.")
                return

        elif task_type == 3:  # Move Object
            rospy.loginfo(f"Executing 'Move Object' task: Moving {task_object} from {location_1.position} to {location_2.position}.")
            self.timer = rospy.Timer(rospy.Duration(k), lambda event: self.timeout_callback(event, task_object), oneshot=True)
            self.goToPoint(location_1)
            if self.timeout_exceeded:
                rospy.loginfo(f"Reassigning 'Move Object' task: {task_object} from {location_1.position} to {location_2.position}.")
                self.timer.shutdown()
                return
            self.goToPoint(location_2)
            self.timer.shutdown()
            if self.timeout_exceeded:
                rospy.loginfo(f"Reassigning 'Move Object' task: {task_object} from {location_1.position} to {location_2.position}.")
                return

        elif task_type == 4:  # Find Object
            rospy.loginfo(f"Executing 'Find Object' task: Searching for {task_object}.")
            self.goToPoint(location_1)
            self.timer = rospy.Timer(rospy.Duration(k), lambda event: self.timeout_callback(event, task_object), oneshot=True)
            self.timer.shutdown()
            if self.timeout_exceeded:
                rospy.loginfo(f"Reassigning 'Find Object' task: Search {task_object} at {location_1.position}.")
                return

        elif task_type == 5 :
            rospy.loginfo(f"Executing 'Go to location' task: Moving to location {location_1.position}")  
            self.goToPoint(location_1)
            self.timer = rospy.Timer(rospy.Duration(k), lambda event: self.timeout_callback(event, task_object), oneshot=True)
            self.timer.shutdown()
            if self.timeout_exceeded:
                rospy.loginfo(f"Reassigning 'Go to location' task: Moving to location {location_1.position}.")
                return  

        else:
            rospy.logwarn(f"Unknown task type: {task_type}. Skipping task.")

        # Task complete
        rospy.loginfo(f"Task completed. Returning to default state: {self.default_state}.")
        self.state = self.default_state


    def goToPoint(self, Point):
        """
        Simulates movement to a location specified.
        """
        waypoint = [{Point.x}, {Point.y}, {Point.z}]
        rospy.loginfo(f"Simulating movement to location: {waypoint}. Publishing waypoint.")

        # Publish the waypoint and start navigation
        self.publish_waypoint(waypoint, task_type=0)  # Assuming task_type=0 is normal task type

        # You can also simulate a delay for the task if needed (e.g., waiting for navigation completion)
        rospy.sleep(3)

    # def calculate_distance_map_frame(self, target_x, target_y):
    #     try:
    #         # Get the robot's position in the "map" frame
    #         (trans, _) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
    #         robot_x, robot_y = trans[0], trans[1]
    #         distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
    #         return distance
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         rospy.logwarn("TF lookup failed.")
    #         return None

    def start_rotation(self):
        """
        Rotates the robot on its axis.
        """
        rate = rospy.Rate(10)  # 10 Hz loop rate
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Rotate at a constant angular velocity

        rospy.loginfo("Rotating on axis.")
        for _ in range(50):  # Rotate for a fixed duration (adjust as needed)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

    def check_battery(self, event):
        # Simulate battery checking; retrieve the battery level from ROS parameters
        battery_level = rospy.get_param(
            f"/{self.namespace}/battery", 100
        )  # Default to 100% if not set

        if self.state != 3:  # Don't check battery while performing a task
            if battery_level < self.battery_threshold and self.state != 4:
                rospy.logwarn(
                    f"{self.namespace} battery low: {battery_level}%. Switching to Charging state."
                )
                self.state = 4
                self.expl_marker_pub.publish(True)  # Indicating charging status
            elif battery_level == self.full_battery and self.state == 4:
                rospy.loginfo(
                    f"{self.namespace} battery fully charged: {battery_level}%. Returning to default state."
                )
                self.state = self.default_state
                self.expl_marker_pub.publish(True)  # Indicating back to exploration

    def publish_zero_velocity(self):
        """Publish zero velocity to /cmd_vel when charging."""
        if self.state == 4:  # Charging state
            twist_msg = Twist()
            # Twist message with zero linear and angular velocities
            self.cmd_vel_pub.publish(twist_msg)

    def get_state_service(self, req):
        rospy.loginfo(
            f"Robot '{self.namespace}' state requested. Current state: '{self.state}'"
        )
        return GetStateResponse(self.state)


def main():
    try:
        # Retrieve the namespace from the parameter server or default to 'robot'
        robot_namespace = rospy.get_param("~robot_namespace", "robot")

        # Initialize the RobotStateNode
        robot_state_node = RobotStateNode(robot_namespace)

        # Set the timer to publish zero velocity at 50Hz if charging
        rospy.Timer(
            rospy.Duration(1.0 / 50), robot_state_node.publish_zero_velocity
        )  # 50 Hz

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RobotStateNode terminated.")


if __name__ == "__main__":
    main()
