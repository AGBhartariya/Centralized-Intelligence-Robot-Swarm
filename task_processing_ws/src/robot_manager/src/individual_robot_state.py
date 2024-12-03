import math
import tf
import rospy
from std_msgs.msg import Bool, Int32
from robot_manager.srv import GetState, GetStateResponse
from robot_manager.msg import Task, DetectedObject # TODO: Create Detected object Msg
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

        self.current_object = None
        self.task_state = None
        self.current_object_location = None
        self.acquired_object = False

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
        self.detect_object_sub = rospy.Subscriber(f"{self.namespace}/detect_object", DetectedObject, self.detect_object_callback)

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

    def detect_object_callback(self, msg):
        """
        Handle detected objects published on the detect_object topic.
        """
        object_id = msg.object_id
        location = msg.location

        # Update or add the detected object in the database
        if self.current_object == object_id and self.task_type in [1, 4] and not self.acquired_object:
            self.acquired_object = True
            self.update_database(self.current_object, self.current_object_location, 'free')
            self.current_object_location = location

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

        task_type, task_object, location_1, location_2 = (
            task.task_type,
            task.task_object,
            task.location_1,
            task.location_2,
        )
        self.current_task_object = task_object
        self.current_task_location = location_1

        # Mark the task's initial object location as occupied
        self.current_object = task_object
        self.current_object_location = location_1
        self.task_state = task_type
        self.state = 3
        self.update_database(self.current_object, self.current_object_location, "occupied")

        if task_type in [1, 4]:  # Bring Object or Find Object
            rospy.loginfo(f"Executing task type {task_type} for object {task_object}.")
            
            # Go to location 1 while checking for free objects
            state = self.navigate_and_check_for_objects(task_object, self.current_object_location)
            if state == 0:
                self.abort_task(f"Failed to reach location {self.current_object_location} or find object {task_object}.")
                self.cleanup_after_task()
                return

            # Rotate at location 1 and check for the object
            if state == 1 and not self.perform_rotation_check(task_object):
                self.update_database(task_object, self.current_object_location, False)
                self.abort_task(f"Object {task_object} not found at {self.current_object_location}.")
                self.cleanup_after_task()
                return

            # If task type is 1, proceed to location 2
            if self.task_state == 1 and not self.navigate_to_point(location_2):
                if not self.navigate_to_point(self.current_object_location):
                    self.abort_task(f"Failed to return {task_object} to {self.current_object_location}.")
                    self.cleanup_after_task()
                    return

                self.leave_object_at_current_location()
                self.update_database(self.current_object, self.current_object_location, "free")
                self.abort_task(f"Task aborted: Unable to move {task_object} to {location_2}.")
                self.cleanup_after_task()
                return

        else:  # For task types 2, 3, and 5
            rospy.loginfo(f"Executing task type {task_type}.")
            if not self.handle_other_tasks(task_type, location_1, location_2, task_object):
                return

        self.current_object_location = location_2
        self.update_database(task_object, self.current_object_location, "free")
        self.cleanup_after_task()

    def handle_other_tasks(self, task_type, location_1, location_2, task_object):
        """
        Handle task types other than 1 and 4.
        """
        if task_type == 2:  # Inspect/Interact
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1} to interact with {task_object}.")
                self.cleanup_after_task()
                return

            if not self.perform_rotation_check(task_object):
                self.update_database(self.current_object, self.current_object_location, False)
                self.abort_task(f"Object {task_object} not found at {location_1}.")
                self.cleanup_after_task()
                return

        elif task_type == 3:  # Move Object
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1} to pick up {task_object}.")
                self.cleanup_after_task()
                return

            if not self.perform_rotation_check(task_object):
                self.update_database(self.current_object, self.current_object_location, False)
                self.abort_task(f"Object {task_object} not found at {location_1}.")
                self.cleanup_after_task()
                return

            if not self.navigate_to_point(location_2):
                if not self.navigate_to_point(location_1):
                    self.abort_task(f"Failed to return {task_object} to {location_1}.")
                    return

                self.leave_object_at_current_location()
                self.update_database(self.current_object, self.current_object_location, "free")
                self.abort_task(f"Task aborted: Unable to move {task_object} to {location_2}.")
                self.cleanup_after_task()

        elif task_type == 5:  # Go to Location
            if not self.navigate_to_point(location_1):
                self.abort_task(f"Failed to reach location {location_1}.")
                self.cleanup_after_task()
                return
        
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

    def navigate_and_check_for_objects(self, task_object, location_1):
        """
        Navigate to location 1 while checking for a free object of the same type.
        """
        rospy.loginfo(f"Navigating to location {location_1} to search for object {task_object}.")
        for attempt in range(self.retry_attempts):
            rospy.loginfo(f"Attempt {attempt + 1}/{self.retry_attempts} to navigate to {location_1}.")
            self.publish_waypoint(location_1)
            start_time = rospy.Time.now()

            while (rospy.Time.now() - start_time).to_sec() < self.timeout_duration:
                # Check for a free object of the required type
                if self.acquired_object:
                    (trans, _) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                    self.current_object_location = trans
                    return 2

                if self.is_within_tolerance(location_1):
                    rospy.loginfo(f"Reached location {location_1}.")
                    return 1
                rospy.sleep(1)

        rospy.logwarn(f"Failed to reach location {location_1} after {self.retry_attempts} attempts.")
        return 0

    def perform_rotation_check(self, task_object):
        """
        Rotate at the current location to search for the specified object.
        """
        rospy.loginfo(f"Performing rotation check for object {task_object}.")
        for attempt in range(self.rotation_attempts):
            rospy.loginfo(f"Rotation attempt {attempt + 1}/{self.rotation_attempts}.")
            self.start_rotation()

            if self.acquired_object:
                return True

        rospy.logwarn(f"Object {task_object} not found after {self.rotation_attempts} rotations.")
        return False

    def cleanup_after_task(self):
        """
        Reset states and mark the handled object as free.
        """
        rospy.loginfo("Cleaning up after task.")
        self.current_object_location = None
        self.current_object = None
        self.acquired_object = False
        self.state = self.default_state

    def publish_waypoint(self, location):
        # TODO
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
        # TODO
        rospy.loginfo(f"Detecting object {task_object} (stubbed logic).")
        return False  # Replace with actual detection logic

    def update_database(self, task_object, task_location, status):
        """
        Stub for database update logic. Replace with actual implementation.
        """
        # TODO
        return

    def leave_object_at_current_location(self):
        # TODO
        rospy.logwarn("Leaving object at current location.")

    def abort_task(self, reason):
        # TODO
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
