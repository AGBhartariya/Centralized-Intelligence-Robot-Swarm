import math
import tf
import rospy
from std_msgs.msg import Bool, Int32, String
from robot_manager.srv import GetState, GetStateResponse
from robot_manager.msg import Task, DetectedObject
from geometry_msgs.msg import Twist, Point, PoseStamped
from trajectory_control_msgs.msg import PlanningTask
from pymongo.server_api import ServerApi
from pymongo.mongo_client import MongoClient
uri = "mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment"
client = MongoClient(uri, server_api=ServerApi('1'))
try:
    client.admin.command('ping')
    print("Pinged your deployment. You successfully connected to MongoDB!")
except Exception as e:
    print(e)
db=client['world']
collection =db['object']

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
        self.task_type = None
        self.current_object_location = None
        self.acquired_object = False

        self.battery_threshold = 10
        self.battery_level = 100
        self.retry_attempts = 3  # Maximum retries for a task
        self.timeout_duration = 180  # Timeout duration in seconds
        self.tolerance = 0.5  # Distance tolerance to consider "reached"
        self.rotation_attempts = 5  # Number of rotations at the location

        self.task_sub = rospy.Subscriber(
            f"{self.namespace}/start_task", Task, self.execute_task
        )
        self.get_state_srv = rospy.Service(
            f"{self.namespace}/get_state", GetState, self.get_state_service
        )
        self.patrol_sub = rospy.Subscriber(
            f"{self.namespace}/patrol_waypoint", Point, self.go_to_patrol_waypoint
        )
        self.isPatrolsub = rospy.Subscriber("isPatrolling", Bool, self.isPatroCallback)
        self.detect_object_sub = rospy.Subscriber(
            f"{self.namespace}/detect_object",
            DetectedObject,
            self.detect_object_callback,
        )

        self.battery_sub = rospy.Subscriber(
            f"{self.namespace}/battery_level", Int32, self.battery_callback
        )
        self.expl_pause_pub = rospy.Publisher(
            f"{self.namespace}/expl_pause_topic", Bool, queue_size=1
        )

        self.cmd_vel_pub = rospy.Publisher(
            f"{self.namespace}/cmd_vel", Twist, queue_size=1
        )
        self.task_pub = rospy.Publisher(
            f"{self.namespace}/planner/tasks/append", PlanningTask, queue_size=10
        )
        self.cancel_pub = rospy.Publisher(
            f"{self.namespace}/planner/tasks/remove", PlanningTask, queue_size=10
        )

        rospy.loginfo(f"RobotStateNode for namespace '{self.namespace}' initialized.")

    def battery_callback(self, msg: Int32):
        """
        Update the battery level based on the message received.
        """
        self.battery_level = msg.data
        rospy.loginfo(f"Battery level updated: {self.battery_level}%")

        if self.battery_level <= self.battery_threshold:
            if (
                self.state != 3 or self.state != 4
            ):  # If the robot is not executing a task
                rospy.logwarn(
                    "Battery level critical and not executing a task. Returning to charging station."
                )
                self.return_to_charging_station()
            else:
                rospy.logwarn(
                    "Battery level critical during task execution. Will return to charging station after task completion."
                )

    def detect_object_callback(self, msg: DetectedObject):
        """
        Handle detected objects published on the detect_object topic.
        """
        object_id = msg.objectId.data
        location = msg.pose

        # Update or add the detected object in the database
        if (
            self.current_object == object_id
            and self.task_type in [1, 4]
            and not self.acquired_object
        ):
            self.acquired_object = True
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
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

    def execute_task(self, task: Task):
        if self.battery_level <= self.battery_threshold:
            self.abort_task("Battery too low to execute task.")
            return

        task_type, task_object, location_1, location_2 = (
            task.description[0].data,
            task.description[1].data,
            task.locations[0],
            task.locations[1],
        )

        # Mark the task's initial object location as occupied
        self.current_object = task_object
        self.current_object_location = location_1
        self.task_type = task_type
        self.state = 3
        self.update_database(
            self.current_object, self.current_object_location, "occupied"
        )

        if self.task_type == 1:
            self.bring_object(location_1, location_2)
        elif self.task_type == 2:
            self.interact_object(location_1, location_2)
        elif self.task_type == 3:
            self.move_object(location_1, location_2)
        elif self.task_type == 4:
            self.find_object(location_1, location_2)
        elif self.task_type == 5:
            self.go_to_location(location_1, location_2)

    # Task type 1
    def bring_object(self, location_1: PoseStamped, location_2: PoseStamped):
        rospy.loginfo(
            f"Executing task type {self.task_type} for object {self.current_object}."
        )
        # Go to location 1 while checking for free objects
        state = self.navigate_and_check_for_objects(
            self.current_object, self.current_object_location
        )
        # Failed to reach the loction or find object, aborting task
        if state == 0:
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
            self.abort_task(
                f"Failed to reach location {self.current_object_location} or find object {self.current_object}."
            )
            self.cleanup_after_task()
            return

        # Reached the deignated location 1 rotating to find the object
        if state == 1 and not self.perform_rotation_check(self.current_object):
            self.update_database(
                self.current_object, self.current_object_location, "remove"
            )
            self.abort_task(
                f"Object {self.current_object} not found at {self.current_object_location}."
            )
            self.cleanup_after_task()
            return

        rospy.loginfo(
            f"Object {self.current_object} found at {self.current_object_location}."  # Found object
        )

        # If task type is 1, pick up the object and proceed to location 2
        self.update_database(
            self.current_object, self.current_object_location, "remove"
        )
        if not self.navigate_to_point(location_2):
            if self.navigate_to_point(self.current_object_location):
                self.update_database(
                    self.current_object, self.current_object_location, "free"
                )
                self.abort_task(
                    f"Failed to reach location {location_2} for object {self.current_object}. Kept object back at {self.current_object_location}."
                )
                self.cleanup_after_task()
                return

            self.leave_object_at_current_location()
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
            self.abort_task(
                f"Task aborted: Unable to move {self.current_object} to {location_2}. Keep it at {self.current_object_location}."
            )
            self.cleanup_after_task()
            return
        # Task of type 1 completed successfully
        rospy.loginfo(f"Kept object {self.current_object} at {location_2}.")
        self.update_database(self.current_object, location_2, "free")
        self.cleanup_after_task()
        return

    # Task type 2
    def interact_object(self, location_1: PoseStamped, location_2: PoseStamped):
        rospy.loginfo(f"Going to {location_1}")
        self.update_database(
            self.current_object, self.current_object_location, "occupied"
        )
        if not self.navigate_to_point(location_1):
            self.abort_task(
                f"Failed to reach location {location_1} to interact with {self.current_object}."
            )
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
            self.cleanup_after_task()
            return

        if not self.perform_rotation_check(self.current_object):
            self.update_database(
                self.current_object, self.current_object_location, "remove"
            )
            self.abort_task(f"Object {self.current_object} not found at {location_1}.")
            self.cleanup_after_task()
            return
        rospy.loginfo(f"Interacted with the object {self.current_object}")
        self.update_database(self.current_object, self.current_object_location, "free")
        self.cleanup_after_task()
        return

    # Task type 3
    def move_object(self, location_1: PoseStamped, location_2: PoseStamped):
        rospy.loginfo(f"Going to {location_1} for getting {self.current_object}")
        self.update_database(
            self.current_object, self.current_object_location, "occupied"
        )
        if not self.navigate_to_point(location_1):
            self.abort_task(
                f"Failed to reach location {location_1} to pick up {self.current_object}."
            )
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
            self.cleanup_after_task()
            return

        if not self.perform_rotation_check(self.current_object):  # Checking for object
            self.update_database(
                self.current_object, self.current_object_location, "remove"
            )
            self.abort_task(f"Object {self.current_object} not found at {location_1}.")
            self.cleanup_after_task()
            return

        self.update_database(
            self.current_object, self.current_object_location, "remove"
        )  # Picking up the object
        rospy.loginfo(
            f"Picked object {self.current_object} from {location_1}. Going to {location_2}"
        )
        if not self.navigate_to_point(location_2):  # Going to location 2
            if self.navigate_to_point(
                location_1
            ):  # Failed to go to location 2 going back to location 1
                self.update_database(
                    self.current_object, self.current_object_location, "free"
                )
                self.abort_task(
                    f"Failed to return to go to {location_2}. Returned object at {self.current_object_location}"
                )
                self.cleanup_after_task
                return

            self.leave_object_at_current_location()  # Failed to go to location 2 keeping object back at current place
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
            self.abort_task(
                f"Task aborted: Unable to move {self.current_object} to {location_2}. Keep in {self.current_object_location}"
            )
            self.cleanup_after_task()
        # Reached location 2
        self.update_database(
            self.current_object, location_2, "free"
        )  # Keeping the object at location 2
        rospy.loginfo(f"Kept object at {location_2}")
        self.cleanup_after_task()
        return

    # Task type 4
    def find_object(self, location_1: PoseStamped, location_2: PoseStamped):
        rospy.loginfo(
            f"Executing task type {self.task_type} for object {self.current_object}."
        )

        # Go to location 1 while checking for free objects
        state = self.navigate_and_check_for_objects(
            self.current_object, self.current_object_location
        )
        # Failed to reach the loction or find object, aborting task
        if state == 0:
            self.update_database(
                self.current_object, self.current_object_location, "free"
            )
            self.abort_task(
                f"Failed to reach location {self.current_object_location} or find object {self.current_object}."
            )
            self.cleanup_after_task()
            return

        # Reached the deignated location 1 rotating to find the object
        if state == 1 and not self.perform_rotation_check(self.current_object):
            self.update_database(
                self.current_object, self.current_object_location, "remove"
            )
            self.abort_task(
                f"Object {self.current_object} not found at {self.current_object_location}."
            )
            self.cleanup_after_task()
            return

        rospy.loginfo(
            f"Object {self.current_object} found at {self.current_object_location}."  # Found object
        )
        self.update_database(self.current_object, self.current_object_location, "free")
        self.cleanup_after_task()
        return

    # Task type 5
    def go_to_location(self, location_1: PoseStamped, location_2: PoseStamped):
        # Go to waypoint
        if not self.navigate_to_point(location_2):
            self.abort_task(f"Failed to reach location {location_2}.")
            self.cleanup_after_task()
            return
        rospy.loginfo(f"Reached {location_2}")
        self.cleanup_after_task()
        return

    def navigate_to_point(self, location: PoseStamped):
        """
        Navigate to the specified location within retry limits.
        """
        for attempt in range(self.retry_attempts):
            rospy.loginfo(
                f"Attempt {attempt + 1}/{self.retry_attempts} to navigate to {location}."
            )
            self.publish_waypoint(location)
            start_time = rospy.Time.now()

            while (rospy.Time.now() - start_time).to_sec() < self.timeout_duration:
                if self.is_within_tolerance(location):
                    rospy.loginfo(f"Successfully reached location {location}.")
                    return True
                rospy.sleep(1)

        rospy.logwarn(
            f"Failed to reach location {location} after {self.retry_attempts} attempts."
        )
        return False

    def navigate_and_check_for_objects(self, task_object: int, location_1: PoseStamped):
        """
        Navigate to location 1 while checking for a free object of the same type.
        """
        rospy.loginfo(
            f"Navigating to location {location_1} to search for object {task_object}."
        )
        for attempt in range(self.retry_attempts):
            rospy.loginfo(
                f"Attempt {attempt + 1}/{self.retry_attempts} to navigate to {location_1}."
            )
            self.publish_waypoint(location_1)
            start_time = rospy.Time.now()

            while (rospy.Time.now() - start_time).to_sec() < self.timeout_duration:
                # Check for a free object of the required type
                if self.acquired_object:
                    (trans, rot) = self.tf_listener.lookupTransform(
                        "map", "base_link", rospy.Time(0)
                    )
                    self.current_object_location = PoseStamped()
                    self.current_object_location.header.frame_id = "map"
                    self.current_object_location.header.stamp = rospy.Time(0)
                    self.current_object_location.pose.position.x = trans[0]
                    self.current_object_location.pose.position.y = trans[1]
                    self.current_object_location.pose.position.z = trans[2]
                    self.current_object_location.pose.orientation.x = rot[0]
                    self.current_object_location.pose.orientation.y = rot[1]
                    self.current_object_location.pose.orientation.z = rot[2]
                    return 2

                if self.is_within_tolerance(location_1):
                    rospy.loginfo(f"Reached location {location_1}.")
                    return 1
                rospy.sleep(1)

        rospy.logwarn(
            f"Failed to reach location {location_1} after {self.retry_attempts} attempts."
        )
        return 0

    def perform_rotation_check(self, task_object: int):
        """
        Rotate at the current location to search for the specified object.
        """
        rospy.loginfo(f"Performing rotation check for object {task_object}.")
        for attempt in range(self.rotation_attempts):
            rospy.loginfo(f"Rotation attempt {attempt + 1}/{self.rotation_attempts}.")
            self.start_rotation()

            if self.acquired_object:
                return True

        rospy.logwarn(
            f"Object {task_object} not found after {self.rotation_attempts} rotations."
        )
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

    def publish_waypoint(self, location: PoseStamped):
        waypoint_msg = Point(
            x=location.pose.position.x,
            y=location.pose.position.y,
            z=location.pose.position.z,
        )
        task_msg = PlanningTask()
        task_msg.name = "navigate_to_waypoint"
        task_msg.segment_id = 1  # Set a unique segment ID
        task_msg.segment_count = 1  # Only one waypoint in this task
        task_msg.type = 0  # Either normal or cyclic type
        task_msg.waypoints = [waypoint_msg]
        self.task_pub.publish(task_msg)

    def is_within_tolerance(self, location: PoseStamped):
        try:
            (trans, _) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )
            distance = math.sqrt(
                (location.pose.position.x - trans[0]) ** 2
                + (location.pose.position.y - trans[1]) ** 2
                + (location.pose.position.z - trans[1]) ** 2
            )
            return distance <= self.tolerance
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
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
        self.detect_object_sub(task_object)
        return False  # Replace with actual detection logic

    def update_database(self, task_object, object_location, status):
        """
        Stub for database update logic. Replace with actual implementation.
        """
        # TODO
        if status == "free":
            data = {'object_type': task_object, 'location': object_location, 'status': status}
        
        insert_doc=collection.insert_one(data)
        print(f"inserted Document ID : {insert_doc.inserted_id}")
        return

    def leave_object_at_current_location(self):
        (trans, rot) = self.tf_listener.lookupTransform(
            "map", "base_link", rospy.Time(0)
        )
        self.current_object_location = PoseStamped()
        self.current_object_location.header.frame_id = "map"
        self.current_object_location.header.stamp = rospy.Time(0)
        self.current_object_location.pose.position.x = trans[0]
        self.current_object_location.pose.position.y = trans[1]
        self.current_object_location.pose.position.z = trans[2]
        self.current_object_location.pose.orientation.x = rot[0]
        self.current_object_location.pose.orientation.y = rot[1]
        self.current_object_location.pose.orientation.z = rot[2]
        rospy.logwarn("Leaving object at current location.")

    def abort_task(self, reason: str):
        abort = PlanningTask()
        abort.name = "abort_task"
        abort.segment_id = 0
        abort.segment_count = 0
        abort.type = 0  # Assuming 0 (NORMAL) is used for cancel tasks
        abort.waypoints = []  # Empty waypoints to indicate cancellation
        self.cancel_pub.publish(abort)
        self.cleanup_after_task()
        rospy.logerr(f"Task aborted: {reason}")

    def get_state_service(self, req):
        return GetStateResponse(self.state)

    def go_to_patrol_waypoint(self, waypoint: PoseStamped):
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
