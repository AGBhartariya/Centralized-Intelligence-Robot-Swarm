#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from object_pose_database.srv import UpdatePose, UpdatePoseRequest

class PoseEstimatorNode:
    def __init__(self):
        rospy.init_node('pose_estimator_node')

        # Image topic subscription
        self.image_topics = rospy.get_param("~image_topics", [])
        self.image_subs = [rospy.Subscriber(topic, Image, self.image_callback, callback_args=topic)
                           for topic in self.image_topics]

        # Service client for updating the pose in the database
        self.update_pose_service = rospy.ServiceProxy('/update_pose', UpdatePose)

        rospy.loginfo("Pose Estimator Node initialized.")

    def image_callback(self, image_msg, topic):
        rospy.loginfo(f"Processing image from topic: {topic}")

        # Simulate pose estimation (replace with model inference code)
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 2.0
        pose.position.z = 3.0
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        # Update the database with the estimated pose
        self.update_pose_database("example_object", pose)

    def update_pose_database(self, object_name, pose):
        try:
            rospy.wait_for_service('/update_pose')
            update_pose_request = UpdatePoseRequest(object_name=object_name, pose=pose)
            response = self.update_pose_service(update_pose_request)
            if response.success:
                rospy.loginfo(f"Successfully updated pose for {object_name}.")
            else:
                rospy.logerr(f"Failed to update pose: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    PoseEstimatorNode()
    rospy.spin()
