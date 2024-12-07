#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from trajectory_control_msgs.msg import PlanningTask  # Correct message type for /ugv1/planner/tasks/append

class WaypointPublisher:
    def __init__(self):
        rospy.init_node("waypoint_terminal_publisher", anonymous=True)

        # Publishers for the topics
        self.waypoint_pub = rospy.Publisher("/ugv1/planner/waypoints/server", Point, queue_size=10)
        self.task_pub = rospy.Publisher("/ugv1/planner/tasks/append", PlanningTask, queue_size=10)
        self.cancel_pub = rospy.Publisher("/ugv1/planner/tasks/remove", PlanningTask, queue_size=10)

        rospy.loginfo("Waypoint Publisher Node Initialized!")

    def get_waypoint_from_terminal(self):
        try:
            x = float(input("Enter X coordinate of the waypoint: "))
            y = float(input("Enter Y coordinate of the waypoint: "))
            z = float(input("Enter Z coordinate of the waypoint: "))
            return Point(x=x, y=y, z=z)
        except ValueError:
            rospy.logerr("Invalid input. Please enter numeric values for coordinates.")
            return None

    def ask_user_for_navigation(self):
        response = input("Navigate to this waypoint? (yes/no): ").strip().lower()
        return response == "yes"

    def ask_user_for_more_waypoints(self):
        """Ask the user if they want to add another waypoint."""
        response = input("Do you want to add another waypoint? (yes/no): ").strip().lower()
        return response == "yes"

    def cancel_current_task(self):
        rospy.loginfo("Cancelling the current task...")
        cancel_msg = PlanningTask()  # Create a PlanningTask message for cancel
        cancel_msg.name = "cancel_task"
        cancel_msg.segment_id = 0
        cancel_msg.segment_count = 0
        cancel_msg.type = 0  # Assuming 0 (NORMAL) is used for cancel tasks
        cancel_msg.waypoints = []  # Empty waypoints to indicate cancellation
        self.cancel_pub.publish(cancel_msg)
        rospy.loginfo("Current task cancelled.")

    def publish_waypoint(self, waypoint, task_type):
        # Cancel the previous task
        self.cancel_current_task()

        # Publish the waypoint to /ugv1/planner/waypoints/server/update
        # rospy.loginfo(f"Publishing waypoint to /ugv1/planner/waypoints/server: {waypoint}")
        # self.waypoint_pub.publish(waypoint)

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

    def run(self):
        """Main loop to get waypoints from the terminal and publish them."""
        rospy.loginfo("Enter waypoints in the terminal. Press Ctrl+C to exit.")
        while not rospy.is_shutdown():
            waypoint = self.get_waypoint_from_terminal()
            if waypoint:
                task_type = 0  # Default to NORMAL task type
                # Ask the user if the task should be cyclic or not
                cyclic_response = input("Should this task be cyclic? (yes/no): ").strip().lower()
                if cyclic_response == "yes":
                    task_type = 1  # Assuming 1 is used for cyclic tasks
                self.publish_waypoint(waypoint, task_type)

                add_more = self.ask_user_for_more_waypoints()
                if not add_more:
                    rospy.loginfo("No more waypoints to add. Exiting node.")
                    break


if __name__ == "__main__":
    try:
        publisher = WaypointPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
