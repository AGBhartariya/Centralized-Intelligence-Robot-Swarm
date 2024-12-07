import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import random

def point_to_pose(point):
    """
    Convert a 3D point from a point cloud to a geometry_msgs/Pose.

    Args:
        point (tuple): A tuple containing the x, y, z coordinates of the point.

    Returns:
        geometry_msgs/Pose: A Pose message with the position set to the point.
    """
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]
    # Orientation is set to default (no rotation)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose

def pointcloud_callback(msg):
    """
    Callback function to process a PointCloud2 message and extract a Pose.

    Args:
        msg (sensor_msgs/PointCloud2): The PointCloud2 message.
    """
    # Extract points from the PointCloud2 message
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    if points:
        # Take the first point for this example
        point = points[int(random.random() * len(points))]  # Extract x, y, z
        rospy.loginfo(f"Extracted point: {point}")

        # Convert the point to a Pose
        pose = point_to_pose(point)
        rospy.loginfo(f"Converted Pose: {pose}")

        # Do something with the pose, e.g., publish it or use it
    else:
        rospy.logwarn("No valid points found in the PointCloud2 message.")

def main():
    rospy.init_node("pointcloud_to_pose_converter")
    rospy.Subscriber("/ugv1/trav/traversability", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
