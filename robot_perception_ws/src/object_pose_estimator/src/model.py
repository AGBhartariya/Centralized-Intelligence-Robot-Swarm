#!/usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
np.float = float  # Temporary alias for compatibility
import ros_numpy
import cv2

def project_point_to_image(point_3d, camera_matrix):
    """Project a 3D point to the 2D image plane using the camera intrinsic matrix."""
    x, y, z = point_3d
    u = int((camera_matrix[0, 0] * x + camera_matrix[0, 2] * z) / z)
    v = int((camera_matrix[1, 1] * y + camera_matrix[1, 2] * z) / z)
    return (u, v)

def visualize_pose(rgb_image, bbox, centroid, orientation, camera_matrix):
    """Overlay the bounding box and pose visualization on the RGB image."""
    # Draw the bounding box
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(rgb_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    # Project the centroid onto the image
    centroid_2d = project_point_to_image(centroid, camera_matrix)
    cv2.circle(rgb_image, centroid_2d, 5, (0, 0, 255), -1)

    # Visualize orientation (example: draw one principal axis)
    axis_length = 0.05  # Adjust length for visualization
    axis_vector = orientation[:, 0] * axis_length  # First principal axis
    end_point = project_point_to_image(centroid + axis_vector, camera_matrix)
    cv2.line(rgb_image, centroid_2d, end_point, (255, 0, 0), 2)

    return rgb_image

class PoseEstimator:
    def __init__(self):
        rospy.init_node("pose_estimator", anonymous=True)

        # Subscribers
        self.rgb_sub = rospy.Subscriber("/realsense/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/realsense/depth/image_rect_raw", Image, self.depth_callback)
        self.pc_sub = rospy.Subscriber("/realsense/depth/color/points", PointCloud2, self.pc_callback)
        self.camera_info_sub = rospy.Subscriber("/realsense/color/camera_info", CameraInfo, self.camera_info_callback)

        # Publisher
        self.pose_pub = rospy.Publisher("/object_pose", Pose, queue_size=10)

        # Utilities
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.pointcloud = None
        self.camera_matrix = None

        # Bounding box coordinates (example, replace with actual detection results)
        self.bbox = [100, 150, 200, 250]  # [x_min, y_min, x_max, y_max]

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def pc_callback(self, msg):
        self.pointcloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

    def camera_info_callback(self, msg):
        # Extract the camera matrix from the CameraInfo message
        self.camera_matrix = np.array(msg.K).reshape(3, 3)

    def extract_points_from_bbox(self):
        if self.rgb_image is None or self.depth_image is None or self.pointcloud is None or self.camera_matrix is None:
            rospy.logwarn("Waiting for inputs (RGB, Depth, PointCloud, Camera Info)...")
            return None

        x_min, y_min, x_max, y_max = self.bbox
        points = []

        # Intrinsic parameters from the camera matrix
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        for v in range(y_min, y_max):
            for u in range(x_min, x_max):
                z = self.depth_image[v, u]
                if z > 0:  # Ignore invalid depths
                    # Convert pixel to 3D points using intrinsic parameters
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])

        return np.array(points)

    def estimate_pose(self, points):
        if len(points) == 0:
            rospy.logwarn("No points found in bounding box region!")
            return None

        # Convert points to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Downsample and remove noise
        pcd = pcd.voxel_down_sample(voxel_size=0.005)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # Compute centroid and orientation using PCA
        points = np.asarray(pcd.points)
        mean = np.mean(points, axis=0)
        cov_matrix = np.cov(points - mean, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # Pose
        centroid = mean
        orientation = eigenvectors  # Convert this to quaternion later

        return centroid, orientation

    def publish_pose(self, centroid, orientation):
        # Convert orientation matrix to quaternion
        q = self.rotation_matrix_to_quaternion(orientation)

        pose_msg = Pose()
        pose_msg.position.x = centroid[0]
        pose_msg.position.y = centroid[1]
        pose_msg.position.z = centroid[2]
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)
        rospy.loginfo("Published Pose: Position: {} Orientation: {}".format(centroid, q))

    @staticmethod
    def rotation_matrix_to_quaternion(rot_matrix):
        """Convert a rotation matrix to a quaternion."""
        w = np.sqrt(1.0 + rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]) / 2.0
        x = (rot_matrix[2, 1] - rot_matrix[1, 2]) / (4.0 * w)
        y = (rot_matrix[0, 2] - rot_matrix[2, 0]) / (4.0 * w)
        z = (rot_matrix[1, 0] - rot_matrix[0, 1]) / (4.0 * w)
        return [x, y, z, w]

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            points = self.extract_points_from_bbox()
            if points is not None:
                pose = self.estimate_pose(points)
                if pose:
                    centroid, orientation = pose
                    self.publish_pose(centroid, orientation)
                                # Visualize and display the pose
                    if self.rgb_image is not None:
                        vis_image = visualize_pose(self.rgb_image.copy(), self.bbox, centroid, orientation, self.camera_matrix)
                        cv2.imshow("Pose Visualization", vis_image)
                        cv2.waitKey(1)
            rate.sleep()


if __name__ == "__main__":
    try:
        estimator = PoseEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass
