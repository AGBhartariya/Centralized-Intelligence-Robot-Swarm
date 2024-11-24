#!/usr/bin/env python

import rospy
import json
import os
from geometry_msgs.msg import Pose
from object_pose_database.srv import UpdatePose, UpdatePoseResponse

class PoseDatabaseNode:
    def __init__(self):
        rospy.init_node('pose_database_node')

        # Path to the external database
        self.database_path = rospy.get_param("~database_path", "/path/to/pose_data.json")

        # Ensure the database file exists
        if not os.path.exists(self.database_path):
            with open(self.database_path, 'w') as db_file:
                json.dump({}, db_file)

        # Load the database
        with open(self.database_path, 'r') as db_file:
            self.database = json.load(db_file)

        # Advertise the UpdatePose service
        self.service = rospy.Service('/update_pose', UpdatePose, self.handle_update_pose)
        rospy.loginfo("Pose Database Node initialized.")

    def handle_update_pose(self, req):
        # Update the database with the new pose
        pose_dict = {
            "position": {
                "x": req.pose.position.x,
                "y": req.pose.position.y,
                "z": req.pose.position.z,
            },
            "orientation": {
                "w": req.pose.orientation.w,
                "x": req.pose.orientation.x,
                "y": req.pose.orientation.y,
                "z": req.pose.orientation.z,
            },
        }
        self.database[req.object_name] = pose_dict

        # Save the database to the external file
        try:
            with open(self.database_path, 'w') as db_file:
                json.dump(self.database, db_file, indent=4)
            return UpdatePoseResponse(success=True, message="Pose updated successfully.")
        except Exception as e:
            return UpdatePoseResponse(success=False, message=str(e))

if __name__ == "__main__":
    PoseDatabaseNode()
    rospy.spin()
