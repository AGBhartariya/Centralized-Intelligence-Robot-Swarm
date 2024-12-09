import rospy
import pymongo
import math
import tf
import numpy as np
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped, Point
from object_pose_database.srv import UpdateDatabase, QueryObjectLocations, GetObjectPose, ClusterAndSample, UpdateDatabaseResponse, QueryObjectLocationsResponse, GetObjectPoseResponse, ClusterAndSampleResponse, UpdateDatabaseRequest, QueryObjectLocationsRequest, GetObjectPoseRequest, ClusterAndSampleRequest
from object_pose_database.msg import DetectObject

# MongoDB configuration
client = pymongo.MongoClient("localhost", 27017)
db = client.ros_workspace
collection = db.objects

# Helper function to calculate Euclidean distance
def euclidean_distance(pose1, pose2):
    return math.sqrt((pose1.position.x - pose2.position.x)**2 +
                     (pose1.position.y - pose2.position.y)**2 +
                     (pose1.position.z - pose2.position.z)**2)

def update_data(req: UpdateDatabaseRequest):
    tolerance = rospy.get_param("tolerance", 0.1)
    default_confidence = rospy.get_param("default_confidence", 0.5)
    query = {"label": req.task_object}
    objects = collection.find(query)

    for obj in objects:
        obj_pose = PoseStamped()
        obj_pose.pose.position.x = obj["object_pose"]["x"]
        obj_pose.pose.position.y = obj["object_pose"]["y"]
        obj_pose.pose.position.z = obj["object_pose"]["z"]
        if euclidean_distance(obj_pose.pose, req.pose.pose) < tolerance:
            if req.status == "remove":
                collection.delete_one({"_id": obj["_id"]})
                return UpdateDatabaseResponse("Object removed.")
            collection.update_one(
                {"_id": obj["_id"]},
                {"$set": {"status": req.status}}
            )
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
            "confidence": default_confidence,
            "status": "free"
        }
        collection.insert_one(new_object)
        return UpdateDatabaseResponse("Object added to database.")

    return UpdateDatabaseResponse("No action performed.")

def query_loc(req: QueryObjectLocationsRequest):
    query = {"label": req.objectType, "status": "free"}
    objects = collection.find(query)
    locations = []

    for obj in objects:
        pose = PoseStamped()
        pose.pose.position.x = obj["robot_pose_at_detection"]["x"]
        pose.pose.position.y = obj["robot_pose_at_detection"]["y"]
        pose.pose.position.z = obj["robot_pose_at_detection"]["z"]
        locations.append(pose)

    return QueryObjectLocationsResponse(locations)

def get_object_pose(req: GetObjectPoseRequest):
    tolerance = rospy.get_param("tolerance", 0.1)
    query = {"label": req.objectId}
    objects = collection.find(query)

    for obj in objects:
        robot_pose = PoseStamped()
        robot_pose.pose.position.x = obj["robot_pose_at_detection"]["x"]
        robot_pose.pose.position.y = obj["robot_pose_at_detection"]["y"]
        robot_pose.pose.position.z = obj["robot_pose_at_detection"]["z"]
        if euclidean_distance(robot_pose.pose, req.robot_location.pose) < tolerance:
            pose = PoseStamped()
            pose.pose.position.x = obj["object_pose"]["x"]
            pose.pose.position.y = obj["object_pose"]["y"]
            pose.pose.position.z = obj["object_pose"]["z"]
            return GetObjectPoseResponse(pose)

    return GetObjectPoseResponse()

def clustering(req: ClusterAndSampleRequest):
    objects = list(collection.find())
    points = []

    for obj in objects:
        points.append([
            obj["robot_pose_at_detection"]["x"],
            obj["robot_pose_at_detection"]["y"],
            obj["robot_pose_at_detection"]["z"]
        ])

    if len(points) < req.num_points:
        return ClusterAndSampleResponse(False, "Not enough points for ClusterAndSample.", [], [])

    points_array = np.array(points)
    kmeans = KMeans(n_clusters=req.num_points)
    kmeans.fit(points_array)
    cluster_centers = kmeans.cluster_centers_
    cluster_ids = kmeans.labels_

    cluster_points = [Point(x=center[0], y=center[1], z=center[2]) for center in cluster_centers]

    return ClusterAndSampleResponse(True, "ClusterAndSample completed.", cluster_points, list(cluster_ids))

def detect_object_callback(msg: DetectObject, namespace: str):
    tolerance = rospy.get_param("tolerance", 0.1)
    query = {"label": msg.objectId}
    objects = collection.find(query)

    for obj in objects:
        obj_pose = PoseStamped()
        obj_pose.pose.position.x = obj["object_pose"]["x"]
        obj_pose.pose.position.y = obj["object_pose"]["y"]
        obj_pose.pose.position.z = obj["object_pose"]["z"]
        if euclidean_distance(obj_pose.pose, msg.pose.pose) < tolerance:
            if msg.confidence > obj["confidence"]:
                collection.update_one(
                    {"_id": obj["_id"]},
                    {"$set": {"confidence": msg.confidence}}
                )
            return

    listener = tf.TransformListener()
    listener.waitForTransform("map", f"{namespace}/base_link", rospy.Time(0), rospy.Duration(4.0))
    (trans, _) = listener.lookupTransform("map", f"{namespace}/base_link", rospy.Time(0))
    robot_pose = {"x": trans[0], "y": trans[1], "z": trans[2]}

    new_object = {
        "label": msg.objectId,
        "object_pose": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
        "robot_pose_at_detection": robot_pose,
        "confidence": msg.confidence,
        "status": "free"
    }
    collection.insert_one(new_object)

if __name__ == "__main__":
    rospy.init_node("ros_mongodb")

    rospy.Service("/update_data", UpdateDatabase, update_data)
    rospy.Service("/query_loc", QueryObjectLocations, query_loc)
    rospy.Service("/getObjectPose", GetObjectPose, get_object_pose)
    rospy.Service("/clustering", ClusterAndSample, clustering)

    num_robots = rospy.get_param("num_robots", 1)
    for i in range(1, num_robots + 1):
        rospy.Subscriber(f"/ugv{i}/detect_object", DetectObject, detect_object_callback, callback_args=f"ugv{i}")

    rospy.spin()
