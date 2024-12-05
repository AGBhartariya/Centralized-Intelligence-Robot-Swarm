#!/usr/bin/env python

import rospy
from pymongo import MongoClient
from pymongo.errors import OperationFailure
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32
from object_pose_database.srv import UpdatePose, UpdatePoseResponse
import math

class PoseDatabaseNode:
    def _init_(self):
        # Initialize the ROS node
        rospy.init_node("pose_database_node")

        self.x_pose = None
        self.y_pose = None
        self.z_pose = None
        self.label = None

        # MongoDB connection details
        self.mongo_uri = rospy.get_param("~mongo_uri", "mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment")
        self.database_name = rospy.get_param("~database_name", "world")
        self.collection_name = rospy.get_param("~collection_name", "object")

        # Tolerance and epsilon
        self.tolerance = rospy.get_param("~tolerance", 0.1) 
        self.epsilon = rospy.get_param("~epsilon", 0.5)

        # Connect to MongoDB
        self._connect_to_mongo()

        # Advertise the UpdatePose service
        self.pose_sub = rospy.Subscriber("/detected_pose", PoseStamped, self.pose_callback)
        self.label_sub = rospy.Subscriber("/detected_label", Int32, self.label_callback)
        
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

    def pose_callback(self,msg: PoseStamped):
         self.x_pose=msg.pose.position.x
         self.y_pose=msg.pose.position.y
         self.z_pose=msg.pose.position.z

    def callback(data, subscriber_id):
        rospy.loginfo(f"Subscriber {subscriber_id} received: {data.data}")
        def label_callback(self,msg):
            self.label=msg
            
    def handle_update_pose(self, req):
        """Callback for the UpdatePose service."""
        rospy.loginfo(f"Received update request for object: {self.label}")

        # Convert Pose message to a dictionary
        pose_dict = {
            "object_name": self.label,
            "position": {
                "x": self.x_pose,
                "y": self.y_pose,
                "z": self.z_pose
            }
        }
        confidence_y = req.confidence
        matching_objects = self.find_matching_objects(pose_dict["position"])

        for obj in matching_objects:
            obj_pose = obj.get("position")
            distance = self.euclidean_distance(pose_dict["position"], obj_pose)

        if distance < self.epsilon:
                confidence_z = obj.get("confidence", 0.8)
                if confidence_y > confidence_z:
                    rospy.loginfo(f"Updating object {req.object_name} in database with new pose and confidence.")
                    self.update_object_pose(obj, pose_dict, confidence_y)
                    return UpdatePoseResponse(success=True, message="Pose updated successfully.")
        return UpdatePoseResponse(success=False, message="No matching object found or confidence not improved.")

    def find_matching_objects(self, new_position):
            query = {
                "position": {
                    "$near": {
                        "$geometry": {
                            "type": "Point",
                            "coordinates": [new_position["x"], new_position["y"], new_position["z"]]
                        },
                        "$maxDistance": self.tolerance  # Set tolerance distance
                    }
                }
            }
            return list(self.collection.find(query))
    
        # Insert or update the pose in the database
    def euclidean_distance(self, pose1, pose2):
            x1, y1, z1 = pose1["x"], pose1["y"], pose1["z"]
            x2, y2, z2 = pose2["x"], pose2["y"], pose2["z"]
            return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    
    def update_object_pose(self, obj, new_pose, confidence_y):
        """Update the pose and confidence of the object in the database."""
        query = {"_id": obj["_id"]}  # Query using the object ID
        update_data = {
            "$set": {
                "position": new_pose["position"],
                "orientation": new_pose["orientation"],
                "confidence": confidence_y,
            }
        }

        try:
            result = self.collection.update_one(query, update_data)
            if result.matched_count > 0:
                rospy.loginfo(f"Object {obj['object_name']} updated in the database.")
            else:
                rospy.logwarn(f"Object {obj['object_name']} was not found for update.")
        except Exception as e:
            rospy.logerr(f"Failed to update pose in the database: {e}")

if __name__ == "_main_":
    try:
        PoseDatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Database Node shutting down.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
