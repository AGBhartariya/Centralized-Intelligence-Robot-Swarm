#!/usr/bin/env python

import rospy
from pymongo import MongoClient
from pymongo.errors import OperationFailure
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32
from object_pose_database.msg import DetectObject
from object_pose_database.srv import UpdateDatabase
from task_data_services.srv import QueryObjectLocations 
from sklearn.cluster import KMeans
from random import sample
import numpy as np
from object_pose_database.srv import ClusterAndSample, ClusterAndSampleResponse
import math

class PoseDatabaseNode:
    def _init_(self):
        # Initialize the ROS node
        rospy.init_node("pose_database_node")

        # MongoDB connection details
        self.mongo_uri = rospy.get_param("~mongo_uri", "mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment")
        self.database_name = rospy.get_param("~database_name", "world")
        self.collection_name = rospy.get_param("~collection_name", "object")

        # Tolerance and epsilon
        self.tolerance = rospy.get_param("~tolerance", 0.1) 
        self.epsilon = rospy.get_param("~epsilon", 0.5)

        # Connect to MongoDB
        self._connect_to_mongo()

        self.service = rospy.Service("update_data", UpdateDatabase, self.update_object_in_database)
        self.service = rospy.Service("query_loc", QueryObjectLocations, self.q_callback )
        
        rospy.loginfo("Pose Database Node initialized and ready to receive requests.")
        
        n = rospy.get_param("~no_of_robots", "4")

        self.create_subscribers(n)

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

    def create_subscribers(self, n):
        subscribers = []
        for i in range(n):
            topic_name = f"/ugv{i}/detected_object"
            data_sub = rospy.Subscriber(topic_name, DetectObject, self.callback, callback_args=i)
            subscribers.append(data_sub)
    
    def callback(self, msg: DetectObject, subscriber_id):
        self.handle_update_pose(msg)

    def handle_update_pose(self, data):
        """Callback for the UpdatePose service."""
        rospy.loginfo(f"Received update request for object")

        # Convert Pose message to a dictionary
        pose_dict = {
            "object_Id": data.objectId,
            "position": {
                "x": data.pose.position.x,
                "y": data.pose.position.y,
                "z": data.pose.position.z
            }
        }
        confidence_y = data.confidence
        matching_objects = self.find_matching_objects(pose_dict["position"])

        for obj in matching_objects:
            obj_pose = obj.get("position")
            distance = self.euclidean_distance(pose_dict["position"], obj_pose)

        if distance < self.epsilon:
                confidence_z = obj.get("confidence", 0.8)
                if confidence_y > confidence_z:
                    rospy.loginfo(f"Updating object {data.objectId} in database with new pose and confidence.")
                    self.update_object_pose(obj, pose_dict, confidence_y)

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
                rospy.loginfo(f"Object {obj['object_Id']} updated in the database.")
            else:
                rospy.logwarn(f"Object {obj['object_Id']} was not found for update.")
        except Exception as e:
            rospy.logerr(f"Failed to update pose in the database: {e}")

    def update_object_in_database(self, req):
        task_object=req.task_object
        pose= req.pose
        status=req.status
        if status == "free":
            data = {
                "object_Id": task_object,
                "position": pose,
                "status": status,
                "confidence": 0.8
            }
            insert_doc = self.collection.insert_one(data)
            return f"Inserted Document ID: {insert_doc.inserted_id}"

        elif status == "occupied":
            query = {
                "object_Id": task_object,
                "position": pose,
                "status": "free"
            }
            update_data = {
                "$set": {
                    "status": "occupied",
                }
            }
            result = self.collection.update_one(query, update_data)
            if result.matched_count > 0:
                return f"Successfully updated {task_object} at {pose} to 'occupied'."
            else:
                return f"No matching document found to update for {task_object} at {pose}."

        elif status == "remove":
            query = {
                "object_Id": task_object,
                "position": pose
            }
            result = self.collection.delete_one(query)
            if result.deleted_count > 0:
                return f"Successfully removed {task_object} from location {pose}."
            else:
                return f"No matching document found for {task_object} at location {pose}."

    def q_callback(self):
        pass #TODO

    def query_free_objects(self, task_object, pose):

        query = {
            "object_id": task_object,
            "status": "free",
            "position": {
                "x": {"$gte": pose["position"]["x"] - self.tolerance, "$lte": pose["position"]["x"] + self.tolerance},
                "y": {"$gte": pose["position"]["y"] - self.tolerance, "$lte": pose["position"]["y"] + self.tolerance},
                "z": {"$gte": pose["position"]["z"] - self.tolerance, "$lte": pose["position"]["z"] + self.tolerance},
            },
        }

        try:
            # Perform the query
            results = list(self.collection.find(query))
            rospy.loginfo(f"Found {len(results)} matching documents with status='free' for object ID {object_id}.")
            return results
        except Exception as e:
            rospy.logerr(f"Failed to query database: {e}")
            return []

    def cluster_and_sample_callback(self, req):
        num_points_per_cluster = req.num_points_per_cluster

        # Fetch all points from the database
        try:
            cursor = self.collection.find({}, {"position": 1, "_id": 0})
            all_points = [
                (point["position"]["x"], point["position"]["y"], point["position"]["z"])
                for point in cursor
            ]
        except Exception as e:
            rospy.logerr(f"Failed to fetch points from the database: {e}")
            return ClusterAndSampleResponse(success=False, message="Database fetch error.", points=[], cluster_ids=[])

        if len(all_points) < 1:
            rospy.logwarn("No points found in the database to cluster.")
            return ClusterAndSampleResponse(success=False, message="No points available.", points=[], cluster_ids=[])

        points_array = np.array(all_points)
        k = self.fetch_cluster_count()

        try:
            kmeans = KMeans(n_clusters=k, random_state=42)
            labels = kmeans.fit_predict(points_array)
        except Exception as e:
            rospy.logerr(f"Clustering algorithm failed: {e}")
            return ClusterAndSampleResponse(success=False, message="Clustering error.", points=[], cluster_ids=[])

        cluster_points = defaultdict(list)
        for label, point in zip(labels, points_array):
            cluster_points[label].append(point)

        # Sample points from each cluster
        response_points = []
        cluster_ids = []
        for cluster_id, points in cluster_points.items():
            rospy.loginfo(f"Cluster {cluster_id} has {len(points)} points.")

            # Randomly sample points from this cluster (or take fewer if not enough points)
            sampled_points = sample(points, min(num_points_per_cluster, len(points)))

            # Convert sampled points to geometry_msgs/Point
            response_points.extend([Point(x=p[0], y=p[1], z=p[2]) for p in sampled_points])
            cluster_ids.extend([cluster_id] * len(sampled_points))

        rospy.loginfo(f"Returning {len(response_points)} sampled points.")
        return ClusterAndSampleResponse(success=True, points=response_points, cluster_ids=cluster_ids)


if __name__ == "_main_":
    try:
        PoseDatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Database Node shutting down.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
