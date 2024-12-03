#!/usr/bin/env python

import rospy
from pymongo import MongoClient
from pymongo.errors import OperationFailure
from geometry_msgs.msg import Pose
from object_pose_database.srv import UpdatePose, UpdatePoseResponse

class PoseDatabaseNode:
    def _init_(self):
        # Initialize the ROS node
        rospy.init_node("pose_database_node")

        # MongoDB connection details
        self.mongo_uri = rospy.get_param("~mongo_uri", "mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment")
        self.database_name = rospy.get_param("~database_name", "pose_database")
        self.collection_name = rospy.get_param("~collection_name", "poses")

        # Connect to MongoDB
        self._connect_to_mongo()

        # Advertise the UpdatePose service
        self.service = rospy.Service("update_pose", UpdatePose, self.handle_update_pose)
        rospy.loginfo("Pose Database Node initialized and ready to receive requests.")

    def _connect_to_mongo(self):
        """Connect to the MongoDB instance."""
        try:
            self.client = MongoClient(self.mongo_uri)
            self.db = self.client[self.database_name]
            self.collection = self.db[self.collection_name]
            rospy.loginfo(f"Connected to MongoDB at {self.mongo_uri}, using database: {self.database_name}.")
        except ConnectionError as e:
            rospy.logerr(f"Failed to connect to MongoDB: {e}")
            raise
        except OperationFailure as e:
            rospy.logerr(f"MongoDB operation failed: {e}")
            raise

    def handle_update_pose(self, req):
        """Callback for the UpdatePose service."""
        rospy.loginfo(f"Received update request for object: {req.object_name}")

        # Convert Pose message to a dictionary
        pose_dict = {
            "object_name": req.object_name,
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

        # Insert or update the pose in the database
        try:
            result = self.collection.update_one(
                {"object_name": req.object_name},
                {"$set": pose_dict},
                upsert=True
            )
            if result.upserted_id:
                rospy.loginfo(f"Inserted new object pose with ID: {result.upserted_id}")
            else:
                rospy.loginfo(f"Updated existing object pose for: {req.object_name}")
            return UpdatePoseResponse(success=True, message="Pose updated successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to update pose in the database: {e}")
            return UpdatePoseResponse(success=False, message=f"Database error: {e}")

if _name_ == "_main_":
    try:
        PoseDatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Database Node shutting down.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
