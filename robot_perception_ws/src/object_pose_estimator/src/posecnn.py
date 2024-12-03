import torch
import torch.nn as nn
import torchvision.transforms as transforms
from torchvision.models.detection import maskrcnn_resnet50_fpn
import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R

class PoseCNN(nn.Module):
    def __init__(self, num_classes):
        super(PoseCNN, self).__init__()
        # Object detection (Mask R-CNN backbone)
        self.detector = maskrcnn_resnet50_fpn(pretrained=True)
        self.detector.roi_heads.box_predictor.cls_score = nn.Linear(
            self.detector.roi_heads.box_predictor.cls_score.in_features, num_classes
        )
        self.detector.roi_heads.box_predictor.bbox_pred = nn.Linear(
            self.detector.roi_heads.box_predictor.bbox_pred.in_features, num_classes * 4
        )

        # 3D coordinate regression network
        self.coord_regressor = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 3, kernel_size=1)  # Predict 3D coordinates
        )

    def forward(self, images):
        # Object detection
        detections = self.detector(images)
        return detections

def estimate_pose(image, depth_image, detections, intrinsic_matrix, object_models):
    """
    Estimates the 6-DoF pose of the detected objects.
    """
    results = []
    for detection in detections:
        bbox = detection['boxes'][0].cpu().numpy().astype(int)
        label = detection['labels'][0].cpu().item()
        score = detection['scores'][0].cpu().item()
        x_min, y_min, x_max, y_max = bbox

        # Crop depth and compute 3D points
        depth_crop = depth_image[y_min:y_max, x_min:x_max]
        ys, xs = np.mgrid[y_min:y_max, x_min:x_max]
        z = depth_crop
        x = (xs - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
        y = (ys - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]
        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

        # Fit the object model (assume the model is provided)
        obj_model = object_models[label]
        pose = align_object_model(points, obj_model)

        results.append({
            "class": label,
            "bbox": bbox,
            "pose": pose
        })

    return results

def align_object_model(points, obj_model):
    """
    Aligns the object model with the observed point cloud using ICP.
    """
    observed_pcd = o3d.geometry.PointCloud()
    observed_pcd.points = o3d.utility.Vector3dVector(points)

    obj_pcd = o3d.geometry.PointCloud()
    obj_pcd.points = o3d.utility.Vector3dVector(obj_model)

    # Perform ICP alignment
    threshold = 0.02
    transformation = o3d.pipelines.registration.registration_icp(
        observed_pcd, obj_pcd, threshold,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    ).transformation

    rotation_matrix = transformation[:3, :3]
    translation = transformation[:3, 3]
    quaternion = R.from_matrix(rotation_matrix).as_quat()

    return {
        'rotation': quaternion,
        'translation': translation
    }

def main():
    # Model and dataset setup
    num_classes = 21  # Example: 20 objects + background
    model = PoseCNN(num_classes=num_classes)
    model.eval()

    # Example camera intrinsic matrix
    intrinsic_matrix = np.array([
        [525.0, 0, 319.5],
        [0, 525.0, 239.5],
        [0, 0, 1]
    ])

    # Example object models
    object_models = {0: np.random.rand(1000, 3)}  # Replace with actual object models

    # Load an example image and depth
    image = cv2.imread('rgb_image.png')
    depth_image = cv2.imread('depth_image.png', cv2.IMREAD_UNCHANGED)

    # Preprocess image for detection
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    input_image = transform(image).unsqueeze(0)

    # Perform detection
    detections = model(input_image)

    # Estimate poses
    results = estimate_pose(image, depth_image, detections[0], intrinsic_matrix, object_models)

    for result in results:
        print("Detected Object:")
        print(f"Class: {result['class']}")
        print(f"Bounding Box: {result['bbox']}")
        print(f"Pose: {result['pose']}")

if __name__ == "__main__":
    main()

