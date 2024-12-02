#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from robot_manager.srv import GetState, GetStateResponse
from robot_manager.msg import Task
from geometry_msgs.msg import Twist


class RobotStateNode:
    def __init__(self, robot_namespace):
        # Initialize the node
        rospy.init_node("robot_state", anonymous=True)

        # Robot namespace
        self.namespace = robot_namespace

        # Robot state: 0 : Idle, 1 : Exploring, 2 : Patrolling, 3 : Tasking, 4 : Charging
        self.default_state = 1
        self.state = self.default_state

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

        rospy.loginfo(
            f"RobotStateNode for namespace '{self.namespace}' initialized. Default state: '{self.state}'"
        )

    def patrol_callback(self, msg):
        if msg:
            self.default_state = 0



    def start_task_callback(self, msg):
         """
            Callback to execute tasks when a task is received.
            param msg: Task message containing [p, [x1, x2], [x3, x4]]
         """

        # Add the task to the min-heap
        heapq.heappush(self.task_queue, (msg.priority, msg))
        self.process_next_task()

        rospy.loginfo(f"Received task: {msg}")
    
        # Parse task components
        priority = msg.p  # p
        task_type = msg.x1  # x1
        task_object = msg.x2  # x2
        location_1 = msg.x3  # x3
        location_x4 = msg.x4  # x4


        def process_next_task(self):
        """
        Processes the next task in the priority queue, if available.
        """
        if self.state == 3:  # Skip processing if already performing a task
            return

        if self.task_queue:
            # Get the task with the lowest priority value (highest actual priority)
            _, task = heapq.heappop(self.task_queue)
            self.execute_task(task)
 
         
        def execute_task(self, task):
        """
        Executes a task based on its type.
        """
        rospy.loginfo(f"Executing task: {task}")
        self.state = 3  # Tasking state

        task_type = task.task_type  # Task type
        task_object = task.task_object  # Task object
        location_1 = task.location_1  # PoseStamped location 1
        location_2 = task.location_2  # PoseStamped location 2 (if applicable)

        if task_type == 1:  # Bring Object
            rospy.loginfo(f"Executing 'Bring Object' task: Bringing {task_object} to {location_1.pose.position}.")
            self.simulate_movement(location_1)
        elif task_type == 2:  # Inspect/Interact
            rospy.loginfo(f"Executing 'Inspect/Interact' task: Interacting with {task_object} at {location_1.pose.position}.")
            self.simulate_movement(location_1)
        elif task_type == 3:  # Move Object
            rospy.loginfo(f"Executing 'Move Object' task: Moving {task_object} from {location_1.pose.position} to {location_2.pose.position}.")
            self.simulate_movement(location_1)
            self.simulate_movement(location_2)
        elif task_type == 4:  # Find Object
            rospy.loginfo(f"Executing 'Find Object' task: Searching for {task_object}.")
            self.simulate_search(task_object)
        else:
            rospy.logwarn(f"Unknown task type: {task_type}. Skipping task.")

        # Task complete
        rospy.loginfo(f"Task completed. Returning to default state: {self.default_state}.")
        self.state = self.default_state

        # Process the next task in the queue
        self.process_next_task()

        def simulate_movement(self, pose_stamped):
           """
           Simulates movement to a location specified
           """

           rospy.loginfo(f"Simulating movement to location: {pose_stamped.pose.position}.")
           rospy.sleep(2)  # Simulate delay for movement
       


        def simulate_search(self, object_type):
            """
            Simulates searching for an object type.
            :param object_type: The type of object to search for
            """
            rospy.loginfo(f"Simulating search for object of type: {object_type}.")
            rospy.sleep(3)  # Simulate delay for search


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
