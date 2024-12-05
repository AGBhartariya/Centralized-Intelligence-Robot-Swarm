#!/usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image, CameraInfo, PointCloud2 # TODO: Create the detected object message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
np.float = float
from ultralytics import YOLO
import ros_numpy
import cv2

class ObjectDetectorPoseEstimator:
    def __init__(self):
        # Get namespace from ROS arguments
        namespace = rospy.get_param("yolo_detect/robot_namespace")

        rospy.init_node(f"object_detector_pose_estimator_{namespace}")

        # YOLO model setup
        self.model = YOLO("yolo11n.pt")  # Load pretrained YOLO model
        self.model.to("cuda")  # Use GPU if available

        # Utilities
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.pointcloud = None
        self.camera_matrix = None
        
        # Subscribers
        self.rgb_sub = rospy.Subscriber("realsense/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("realsense/depth/image_rect_raw", Image, self.depth_callback)
        self.pc_sub = rospy.Subscriber("realsense/depth/color/points", PointCloud2, self.pc_callback)
        self.camera_info_sub = rospy.Subscriber("realsense/color/camera_info", CameraInfo, self.camera_info_callback)

        # Publishers
        self.pose_pub = rospy.Publisher("detected_pose", PoseStamped, queue_size=10)  # TODO: Use the detect object message rate for the rostopic rather than 2 separate topics
        self.label_pub = rospy.Publisher("detected_label", Int32, queue_size=10)
        self.image_pub = rospy.Publisher("detectedImage", Image, queue_size=10)

        # ROS parameters
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.8)
        self.visualize_detected_pose = rospy.get_param("~visualizeDetectedPose", True)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.detect_and_estimate()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def pc_callback(self, msg):
        self.pointcloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

    def camera_info_callback(self, msg):
        # Extract the camera matrix from the CameraInfo message
        self.camera_matrix = np.array(msg.K).reshape(3, 3)

    def extract_points_from_bbox(self, bbox):
        if self.rgb_image is None or self.depth_image is None or self.pointcloud is None or self.camera_matrix is None:
            rospy.logwarn("Waiting for inputs (RGB, Depth, PointCloud, Camera Info)...")
            return None

        x_min, y_min, x_max, y_max = bbox
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

    def rotation_matrix_to_quaternion(self, rot_matrix):
        w = np.sqrt(1.0 + rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]) / 2.0
        x = (rot_matrix[2, 1] - rot_matrix[1, 2]) / (4.0 * w)
        y = (rot_matrix[0, 2] - rot_matrix[2, 0]) / (4.0 * w)
        z = (rot_matrix[1, 0] - rot_matrix[0, 1]) / (4.0 * w)
        return [x, y, z, w]

    def visualize_pose(self, bbox, centroid):
        x_min, y_min, x_max, y_max = bbox
        visualized_image = self.rgb_image.copy()
        cv2.rectangle(visualized_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.circle(visualized_image, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
        return visualized_image

    def detect_and_estimate(self):
        if self.rgb_image is None:
            return

        # Run YOLO detection
        results = self.model(self.rgb_image)
        detected = False  # Track if any detection occurred

        for result in results:
            for box in result.boxes:
                confidence = box.conf
                if confidence >= self.confidence_threshold:
                    detected = True
                    bbox = box.xyxy.int().tolist()[0]
                    class_idx = int(box.cls.item())  # Convert tensor to integer
                    label = result.names[class_idx]
                    rospy.loginfo(bbox)
                    points = self.extract_points_from_bbox(bbox)
                    if points is not None:
                        pose = self.estimate_pose(points)
                        if pose:
                            centroid, orientation = pose
                            q = self.rotation_matrix_to_quaternion(orientation)

                            # Publish PoseStamped message
                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = rospy.Time.now()
                            pose_msg.header.frame_id = "camera_frame"
                            pose_msg.pose.position.x = centroid[0]
                            pose_msg.pose.position.y = centroid[1]
                            pose_msg.pose.position.z = centroid[2]
                            pose_msg.pose.orientation.x = q[0]
                            pose_msg.pose.orientation.y = q[1]
                            pose_msg.pose.orientation.z = q[2]
                            pose_msg.pose.orientation.w = q[3]

                            self.pose_pub.publish(pose_msg)
                            
                            # Publish label
                            label_msg = Int32()
                            label_msg.data = class_idx
                            self.label_pub.publish(label_msg)

                            # rospy.loginfo(f"Detected {label} with pose: {pose_msg}")

                            # Visualize pose if parameter is true
                            if self.visualize_detected_pose:
                                visualized_image = self.visualize_pose(bbox, centroid)
                                image_msg = self.bridge.cv2_to_imgmsg(visualized_image, "bgr8")
                                self.image_pub.publish(image_msg)

        # If no detections, publish the unaltered RGB image
        if not detected:
            rospy.loginfo("No detections found. Publishing original image.")
            image_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, "bgr8")
            self.image_pub.publish(image_msg)

if __name__ == "__main__":
    try:
        detector = ObjectDetectorPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
