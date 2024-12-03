import math
import tf
import rospy
from std_msgs.msg import Bool, Int32
from robot_manager.srv import GetState, GetStateResponse
from robot_manager.msg import Task
from geometry_msgs.msg import Twist, Point
from trajectory_control_msgs.msg import PlanningTask

class RobotStateNode:
    def __init__(self, robot_namespace):
        rospy.init_node("robot_state", anonymous=True)

        self.namespace = robot_namespace

        # Robot state: 0 : Idle, 1 : Exploring, 2 : Patrolling, 3 : Tasking, 4 : Charging
        self.default_state = 1
        self.state = self.default_state
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)

        self.battery_threshold = 10
        self.battery_level = 100
        self.retry_attempts = 3  # Maximum retries for a task
        self.timeout_duration = 180  # Timeout duration in seconds
        self.tolerance = 0.5  # Distance tolerance to consider "reached"
        self.rotation_attempts = 5  # Number of rotations at the location

        self.task_sub = rospy.Subscriber(f"{self.namespace}/start_task", Task, self.execute_task)
        self.get_state_srv = rospy.Service(f"{self.namespace}/get_state", GetState, self.get_state_service)
        self.patrol_sub = rospy.Subscriber(f"{self.namespace}/patrol_waypoint", Point, self.go_to_patrol_waypoint)
        self.isPatrolsub = rospy.Subscriber("isPatrolling", Bool, self.isPatroCallback)
        
        self.battery_sub = rospy.Subscriber(f"{self.namespace}/battery_level", Int32, self.battery_callback)
        self.expl_pause_pub = rospy.Publisher(f"{self.namespace}/expl_pause_topic", Bool, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher(f"{self.namespace}/cmd_vel", Twist, queue_size=1)
        self.task_pub = rospy.Publisher(f"{self.namespace}/planner/tasks/append", PlanningTask, queue_size=10)
        self.cancel_pub = rospy.Publisher(f"{self.namespace}/planner/tasks/remove", PlanningTask, queue_size=10)

        rospy.loginfo(f"RobotStateNode for namespace '{self.namespace}' initialized.")

    def battery_callback(self, msg):
        """
        Update the battery level based on the message received.
        """
        self.battery_level = msg.data
        rospy.loginfo(f"Battery level updated: {self.battery_level}%")

        if self.battery_level <= self.battery_threshold:
            if self.state != 3 or self.state != 4:  # If the robot is not executing a task
                rospy.logwarn("Battery level critical and not executing a task. Returning to charging station.")
                self.return_to_charging_station()
            else:
                rospy.logwarn("Battery level critical during task execution. Will return to charging station after task completion.")

    def return_to_charging_station(self):
        """
        Navigate the robot back to a predefined charging station.
        """
        self.state = 4
        charging_station_location = Point(0, 0, 0)  # Replace with actual coordinates
        if self.navigate_to_point(charging_station_location):
            rospy.loginfo("Successfully reached charging station.")
            self.expl_pause_pub.publish(True)  # Pause exploration

            # Simulate charging until battery is full
            while self.battery_level < self.full_battery:
                self.publish_zero_cmd_vel()
                rospy.sleep(1)  # Wait for battery to charge (simulated)

            rospy.loginfo("Battery fully charged.")
            self.expl_pause_pub.publish(False)  # Resume exploration
            self.state = self.default_state
        else:
            rospy.logwarn("Failed to reach charging station.")

    def publish_zero_cmd_vel(self):
        """
        Publish zero velocity as a failsafe.
        """
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)

    def isPatroCallback(self, msg):
        if msg.data:
            self.default_state = 0

    def execute_task(self, task):
        if self.battery_level <= self.battery_threshold:
            self.abort_task("Battery too low to execute task.")
            return
        
        task_type, task_object, location_1, location_2 = task.task_type, task.task_object, task.location_1, task.location_2
        retry_count = 0
        self.state = 3
        if task_type == 1:  # Bring Object
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1} to pick up {task_object}.")
                return

            if not self.perform_rotation_check(task_object):
                self.update_database(task_object, found=False)
                self.abort_task(f"Object {task_object} not found at {location_1}.")
                return

            if not self.navigate_to_point(location_2):
                if not self.navigate_to_point(location_1):
                    self.abort_task(f"Failed to return {task_object} to {location_1}.")
                    return

                self.leave_object_at_current_location()
                self.abort_task(f"Task aborted: Unable to deliver {task_object} to {location_2}.")

        elif task_type == 2:  # Inspect/Interact
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1} to interact with {task_object}.")
                return

            if not self.perform_rotation_check(task_object):
                self.update_database(task_object, found=False)
                self.abort_task(f"Object {task_object} not found at {location_1}.")
                return

        elif task_type == 3:  # Move Object
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1} to pick up {task_object}.")
                return

            if not self.perform_rotation_check(task_object):
                self.update_database(task_object, found=False)
                self.abort_task(f"Object {task_object} not found at {location_1}.")
                return

            if not self.navigate_to_point(location_2):
                if not self.navigate_to_point(location_1):
                    self.abort_task(f"Failed to return {task_object} to {location_1}.")
                    return

                self.leave_object_at_current_location()
                self.abort_task(f"Task aborted: Unable to move {task_object} to {location_2}.")

        elif task_type == 4:  # Find Object
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach search location {location_1}.")
                return

            if not self.perform_rotation_check(task_object):
                self.update_database(task_object, found=False)
                self.abort_task(f"Object {task_object} not found.")

        elif task_type == 5:  # Go to Location
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1}.")
                return

        self.state = self.default_state  # Return to default state after task
        if self.battery_level <= self.battery_threshold:
            rospy.logwarn("Battery low after task. Returning to charging station.")
            self.return_to_charging_station()

        rospy.loginfo("Task completed successfully.")

    def navigate_to_point(self, location):
        """
        Navigate to the specified location within retry limits.
        """
        for attempt in range(self.retry_attempts):
            rospy.loginfo(f"Attempt {attempt + 1}/{self.retry_attempts} to navigate to {location}.")
            self.publish_waypoint(location)
            start_time = rospy.Time.now()

            while (rospy.Time.now() - start_time).to_sec() < self.timeout_duration:
                if self.is_within_tolerance(location):
                    rospy.loginfo(f"Successfully reached location {location}.")
                    return True
                rospy.sleep(1)

        rospy.logwarn(f"Failed to reach location {location} after {self.retry_attempts} attempts.")
        return False

    def perform_rotation_check(self, task_object):
        """
        Rotate at the current location to search for the specified object.
        """
        rospy.loginfo(f"Performing rotation check for object {task_object}.")
        for _ in range(self.rotation_attempts):
            self.start_rotation()
            if self.detect_object(task_object):
                rospy.loginfo(f"Object {task_object} found.")
                return True

        rospy.logwarn(f"Object {task_object} not found after {self.rotation_attempts} rotations.")
        return False

    def publish_waypoint(self, location):
        waypoint_msg = Point(x=location.x, y=location.y, z=location.z)
        self.task_pub.publish(waypoint_msg)

    def is_within_tolerance(self, location):
        try:
            (trans, _) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            distance = math.sqrt((location.x - trans[0]) ** 2 + (location.y - trans[1]) ** 2)
            return distance <= self.tolerance
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed.")
            return False

    def start_rotation(self):
        cmd = Twist()
        cmd.angular.z = 0.5
        rate = rospy.Rate(10)
        for _ in range(10):
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

    def detect_object(self, task_object):
        """
        Stub for object detection logic. Replace with actual implementation.
        """
        rospy.loginfo(f"Detecting object {task_object} (stubbed logic).")
        return False  # Replace with actual detection logic

    def update_database(self, task_object, found):
        """
        Stub for database update logic. Replace with actual implementation.
        """
        rospy.loginfo(f"Updating database: Object {task_object}, Found: {found} (stubbed logic).")

    def leave_object_at_current_location(self):
        rospy.logwarn("Leaving object at current location.")

    def abort_task(self, reason):
        rospy.logerr(f"Task aborted: {reason}")

    def get_state_service(self, req):
        return GetStateResponse(self.state)

    def go_to_patrol_waypoint(self, waypoint):
        """
        Navigate to a patrol waypoint received from the topic.
        """
        self.state = 2
        rospy.loginfo(f"Received patrol waypoint: {waypoint}")
        if self.navigate_to_point(waypoint):
            rospy.loginfo(f"Successfully reached patrol waypoint {waypoint}.")
        else:
            rospy.logwarn(f"Failed to reach patrol waypoint {waypoint}.")
        self.state = self.default_state

def main():
    try:
        robot_namespace = rospy.get_param("~robot_namespace", "robot")
        robot_state_node = RobotStateNode(robot_namespace)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RobotStateNode terminated")

if __name__ == "__main__":
    main()
