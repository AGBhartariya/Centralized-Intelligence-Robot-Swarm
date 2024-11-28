#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from robot_manager.srv import GetState, GetStateResponse
from robot_manager.msg import Task
import time

class RobotStateNode:
    def __init__(self, robot_namespace):
        # Initialize the node
        rospy.init_node('robot_state', anonymous=True)

        # Robot namespace
        self.namespace = robot_namespace

        # Robot state: 0 : Idle, 1 : Exploring, 2 : Patrolling, 3 : Tasking, 4 : Charging
        self.default_state = 1
        self.state = self.default_state

        # Battery threshold for low battery state
        self.battery_threshold = 10
        self.full_battery = 100

        # Publishers and Subscribers
        self.task_sub = rospy.Subscriber(f"{self.namespace}/start_task", Task, self.start_task_callback)
        
        # Service for getting the robot's state
        self.get_state_srv = rospy.Service(f"{self.namespace}/get_state", GetState, self.get_state_service)

        # Publisher to indicate exploration status
        self.expl_marker_pub = rospy.Publisher(f"/{self.namespace}/expl_marker_update/update", Bool, queue_size=1)

        # Start with publishing False (indicating not in exploration)
        self.expl_marker_pub.publish(False)

        # Start with a subscriber to check with exploration is finished
        self.patrol_sub = rospy.Subscriber("/isPatrolling", Bool, self.patrol_callback)
        
        # Start the battery checking loop
        self.check_battery_rate = rospy.get_param("~battery_check_interval", 1) * 60  # in seconds (default to 1 min)
        rospy.Timer(rospy.Duration(self.check_battery_rate), self.check_battery)

        rospy.loginfo(f"RobotStateNode for namespace '{self.namespace}' initialized. Default state: '{self.state}'")

    def patrol_callback(self, msg):
        if msg:
            self.default_state = 0

    def start_task_callback(self, msg):
        # TODO: Implement Task Completion function here
        self.state = self.default_state

    def check_battery(self, event):
        # Simulate battery checking; retrieve the battery level from ROS parameters
        battery_level = rospy.get_param(f'/{self.namespace}/battery', 100)  # Default to 100% if not set

        if self.state != 3:  # Don't check battery while performing a task
            if battery_level < self.battery_threshold and self.state != 4:
                rospy.logwarn(f"{self.namespace} battery low: {battery_level}%. Switching to Charging state.")
                self.state = 4
                self.expl_marker_pub.publish(True)  # Indicating charging status
            elif battery_level == self.full_battery and self.state == 4:
                rospy.loginfo(f"{self.namespace} battery fully charged: {battery_level}%. Returning to default state.")
                self.state = self.default_state
                self.expl_marker_pub.publish(True)  # Indicating back to exploration

    def get_state_service(self, req):
        rospy.loginfo(f"Robot '{self.namespace}' state requested. Current state: '{self.state}'")
        return GetStateResponse(self.state)

def main():
    try:
        # Retrieve the namespace from the parameter server or default to 'robot'
        robot_namespace = rospy.get_param('~robot_namespace', 'robot')

        # Initialize the RobotStateNode
        robot_state_node = RobotStateNode(robot_namespace)

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RobotStateNode terminated.")

if __name__ == '__main__':
    main()
