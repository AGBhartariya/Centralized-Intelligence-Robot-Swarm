import rospy
import pymongo
import math
import tf
import numpy as np
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped, Point
from object_pose_database.srv import UpdateDatabase, QueryObjectLocations, GetObjectPose, ClusterAndSample, UpdateDatabaseResponse, QueryObjectLocationsResponse, GetObjectPoseResponse, ClusterAndSampleResponse, UpdateDatabaseRequest, QueryObjectLocationsRequest, GetObjectPoseRequest, ClusterAndSampleRequest
from object_pose_database.msg import DetectObject

class MongoDBInterface:
    def __init__(self):
        # Get parameters
        self.tolerance = rospy.get_param("~tolerance", 0.1)
        self.default_confidence = rospy.get_param("~default_confidence", 0.5)
        self.database_name = rospy.get_param("~database_name", "world")
        self.collection_name = rospy.get_param("~collection_name", "object")
        self.epsilon = rospy.get_param("~epsilon", 0.5)
        # Initialize MongoDB connection
        self.client = pymongo.MongoClient("mongodb+srv://all:simpledb@environment.wfwxr.mongodb.net/?retryWrites=true&w=majority&appName=Environment") # Aditya's Database
        self.db = self.client[self.database_name]
        self.collection = self.db[self.collection_name]

    def euclidean_distance(self, pose1, pose2):
        return math.sqrt((pose1.position.x - pose2.position.x)**2 +
                         (pose1.position.y - pose2.position.y)**2 +
                         (pose1.position.z - pose2.position.z)**2)

    def update_data(self, req: UpdateDatabaseRequest):
        rospy.loginfo("Received update_data request.")
        query = {"label": req.task_object}
        objects = self.collection.find(query)

        for obj in objects:
            obj_pose = PoseStamped()
            obj_pose.pose.position.x = obj["object_pose"]["x"]
            obj_pose.pose.position.y = obj["object_pose"]["y"]
            obj_pose.pose.position.z = obj["object_pose"]["z"]
            if self.euclidean_distance(obj_pose.pose, req.pose.pose) < self.tolerance:
                if req.status == "remove":
                    self.collection.delete_one({"_id": obj["_id"]})
                    rospy.loginfo("Object removed from database.")
                    return UpdateDatabaseResponse("Object removed.")
                self.collection.update_one(
                    {"_id": obj["_id"]},
                    {"$set": {"status": req.status}}
                )
                rospy.loginfo("Object status updated in database.")
                return UpdateDatabaseResponse("Object status updated.")

        if req.status == "free" and req.flag:
            listener = tf.TransformListener()
            listener.waitForTransform("map", f"{req.robot_id}/base_link", rospy.Time(0), rospy.Duration(4.0))
            (trans, _) = listener.lookupTransform("map", f"{req.robot_id}/base_link", rospy.Time(0))
            robot_pose = {"x": trans[0], "y": trans[1], "z": trans[2]}
            new_object = {
                "label": req.task_object,
                "object_pose": {"x": req.pose.pose.position.x, "y": req.pose.pose.position.y, "z": req.pose.pose.position.z},
                "robot_pose_at_detection": robot_pose,
                "confidence": self.default_confidence,
                "status": "free"
            }
            self.collection.insert_one(new_object)
            rospy.loginfo("New object added to database.")
            return UpdateDatabaseResponse("Object added to database.")

        rospy.loginfo("No action performed in update_data.")
        return UpdateDatabaseResponse("No action performed.")

    def query_loc(self, req: QueryObjectLocationsRequest):
        rospy.loginfo("Received query_loc request.")
        query = {"label": req.objectType, "status": "free"}
        objects = self.collection.find(query)
        locations = []

        for obj in objects:
            pose = PoseStamped()
            pose.pose.position.x = obj["robot_pose_at_detection"]["x"]
            pose.pose.position.y = obj["robot_pose_at_detection"]["y"]
            pose.pose.position.z = obj["robot_pose_at_detection"]["z"]
            locations.append(pose)

        rospy.loginfo(f"Returning {len(locations)} locations for free objects of type {req.objectType}.")
        return QueryObjectLocationsResponse(locations)

    def get_object_pose(self, req: GetObjectPoseRequest):
        rospy.loginfo("Received get_object_pose request.")
        query = {"label": req.objectId, "status": "free"}
        objects = self.collection.find(query)

        for obj in objects:
            robot_pose = PoseStamped()
            robot_pose.pose.position.x = obj["robot_pose_at_detection"]["x"]
            robot_pose.pose.position.y = obj["robot_pose_at_detection"]["y"]
            robot_pose.pose.position.z = obj["robot_pose_at_detection"]["z"]
            if self.euclidean_distance(robot_pose.pose, req.robot_location.pose) < self.epsilon:
                pose = PoseStamped()
                pose.pose.position.x = obj["object_pose"]["x"]
                pose.pose.position.y = obj["object_pose"]["y"]
                pose.pose.position.z = obj["object_pose"]["z"]
                pose.header.frame_id = "map"
                rospy.loginfo("Returning object pose.")
                return GetObjectPoseResponse(pose)

        rospy.loginfo("No matching object found for get_object_pose.")
        return GetObjectPoseResponse()

    def clustering(self, req: ClusterAndSampleRequest):
        rospy.loginfo("Received clustering request.")
        objects = list(self.collection.find())
        points = []

        for obj in objects:
            points.append([
                obj["robot_pose_at_detection"]["x"],
                obj["robot_pose_at_detection"]["y"],
                obj["robot_pose_at_detection"]["z"]
            ])

        if len(points) < req.num_points:
            rospy.loginfo("Not enough points for clustering.")
            return ClusterAndSampleResponse(False, "Not enough points for clustering.", [], [])

        points_array = np.array(points)
        kmeans = KMeans(n_clusters=req.num_points)
        kmeans.fit(points_array)
        cluster_centers = kmeans.cluster_centers_
        cluster_ids = kmeans.labels_

        cluster_points = [Point(x=center[0], y=center[1], z=center[2]) for center in cluster_centers]

        rospy.loginfo("Clustering completed successfully.")
        return ClusterAndSampleResponse(True, "Clustering completed.", cluster_points, list(cluster_ids))

    def detect_object_callback(self, msg: DetectObject, namespace: str):
        rospy.loginfo(f"Received detect_object message from namespace {namespace}.")
        query = {"label": msg.objectId}
        objects = self.collection.find(query)

        listener = tf.TransformListener()
        listener.waitForTransform("map", f"{namespace}/base_link", rospy.Time(0), rospy.Duration(4.0))
        (trans, _) = listener.lookupTransform("map", f"{namespace}/base_link", rospy.Time(0))
        robot_pose = {"x": trans[0], "y": trans[1], "z": trans[2]}

        object = {
            "label": msg.objectId,
            "object_pose": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
            "robot_pose_at_detection": robot_pose,
            "confidence": msg.confidence,
            "status": "free"
        }

        for obj in objects:
            obj_pose = PoseStamped()
            obj_pose.pose.position.x = obj["object_pose"]["x"]
            obj_pose.pose.position.y = obj["object_pose"]["y"]
            obj_pose.pose.position.z = obj["object_pose"]["z"]
            if self.euclidean_distance(obj_pose.pose, msg.pose.pose) < self.tolerance:
                if msg.confidence > obj["confidence"]:
                    self.collection.update_one(
                        {"_id": obj["_id"]},
                        {"$set": object}
                    )
                    rospy.loginfo("Updated object confidence in database.")
                return

        self.collection.insert_one(object)
        rospy.loginfo("New object added to database from detect_object message.")


if __name__ == "__main__":
    rospy.init_node("ros_mongodb")
    mongodb_interface = MongoDBInterface()

    rospy.Service("/update_data", UpdateDatabase, mongodb_interface.update_data)
    rospy.Service("/query_loc", QueryObjectLocations, mongodb_interface.query_loc)
    rospy.Service("/getObjectPose", GetObjectPose, mongodb_interface.get_object_pose)
    rospy.Service("/clustering", ClusterAndSample, mongodb_interface.clustering)

    num_robots = rospy.get_param("~no_of_robots", 1)
    for i in range(1, num_robots + 1):
        rospy.Subscriber(f"/ugv{i}/detect_object", DetectObject, mongodb_interface.detect_object_callback, callback_args=f"ugv{i}")

    rospy.loginfo("MongoDB ROS node is up and running for {num_robots}.")
    rospy.spin()
